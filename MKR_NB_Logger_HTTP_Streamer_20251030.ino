#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <MKRNB.h>
#include <NBUDP.h>
#include <NBSSLClient.h>
#include <RTCZero.h>
#include <Adafruit_BME280.h>
#include "config.h"
#include <math.h>
#include <climits>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>

#ifndef LED_INVERT
#define LED_INVERT 1
#endif
#ifndef BUZZER_ACTIVE
#define BUZZER_ACTIVE 0
#endif
#ifndef BOOT_STEP_GAP_MS
#define BOOT_STEP_GAP_MS 600
#endif

// ---- Sensirion SHT3x ----
#include <SensirionI2cSht3x.h>
SensirionI2cSht3x sht3x;
static bool hasSHT = false;
static bool sht_use_single_shot = false;   // Fallback, falls Periodic-API fehlt

// ====== Globals ======
NB nb; GPRS gprs; NBUDP udp; RTCZero rtc;
static bool g_netReady = false;
static bool g_udpReady = false;

// ====== LED (State Machine) ======
#if LED_ACTIVE
enum LedMode {
  LED_MODE_OFF,
  LED_MODE_SOLID_ON,        // Power an
  LED_MODE_SLOW_BLINK,      // LTE+PDP Aufbau
  LED_MODE_FAST_BLINK,      // Time-Sync
  LED_MODE_MEAS_PULSE,      // Messbetrieb: Puls via led_pulse_ms()
  LED_MODE_UPLOAD_2HZ,      // Upload: 2 Blitze/sec
  LED_MODE_NO_LTE_SHORT_OFF // Kein LTE: kurz aus/sek
};

static LedMode      g_ledMode = LED_MODE_OFF;
static LedMode      g_ledSaved = LED_MODE_OFF;
static bool         g_ledHasSaved = false;

static unsigned long g_ledT0 = 0;
static unsigned long g_ledPulseUntil = 0;
static bool          g_ledState = false;


// ---------- LED low-level (genau 1x im Sketch behalten) ----------
static inline void led_hw(bool on) {
  // Berechne den tatsächlichen Pegel am Pin
  // on=true  -> physisch an
  // on=false -> physisch aus
  uint8_t level = on
    ? (LED_INVERT ? LOW  : HIGH)   // active-low: LOW schaltet an
    : (LED_INVERT ? HIGH : LOW);   // active-low: HIGH schaltet aus

  static bool lastOn = !on;
  if (lastOn != on) {
    lastOn = on;
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, level);
  }
}

static inline void led_set_mode(LedMode m) {
  g_ledMode = m;
  g_ledT0 = millis();
  g_ledPulseUntil = 0;
  pinMode(LED_PIN, OUTPUT);
  led_hw(m == LED_MODE_SOLID_ON);
}
static inline void led_begin_activity(LedMode m) {
  if (!g_ledHasSaved) { g_ledSaved = g_ledMode; g_ledHasSaved = true; }
  led_set_mode(m);
}
static inline void led_end_activity() {
  if (g_ledHasSaved) { led_set_mode(g_ledSaved); g_ledHasSaved = false; }
}
static inline void led_pulse_ms(uint16_t ms) {
  unsigned long now = millis();
  g_ledPulseUntil = now + ms;
  led_hw(true);
}
static inline void led_update() {
  unsigned long now = millis();
  switch (g_ledMode) {
    case LED_MODE_OFF: led_hw(false); break;
    case LED_MODE_SOLID_ON: led_hw(true); break;
    case LED_MODE_SLOW_BLINK: {
      unsigned long slot = g_ledState ? LED_SLOW_ON_MS : LED_SLOW_OFF_MS;
      if (now - g_ledT0 >= slot) { g_ledT0 = now; led_hw(!g_ledState); }
    } break;
    case LED_MODE_FAST_BLINK: {
      unsigned long slot = g_ledState ? LED_FAST_ON_MS : LED_FAST_OFF_MS;
      if (now - g_ledT0 >= slot) { g_ledT0 = now; led_hw(!g_ledState); }
    } break;
    case LED_MODE_MEAS_PULSE:
      if (g_ledPulseUntil && (long)(now - g_ledPulseUntil) >= 0) {
        g_ledPulseUntil = 0; led_hw(false);
      }
      break;
    case LED_MODE_UPLOAD_2HZ: {
      unsigned long phase = (now - g_ledT0) % LED_UPLOAD_PERIOD_MS;
      led_hw(phase < LED_UPLOAD_PULSE_MS);
    } break;
    case LED_MODE_NO_LTE_SHORT_OFF: {
      unsigned long phase = (now - g_ledT0) % LED_NOLTE_PERIOD_MS;
      led_hw(!(phase < LED_NOLTE_OFF_MS));
    } break;
  }
}
#else
  #define led_set_mode(x)       do{}while(0)
  #define led_begin_activity(x) do{}while(0)
  #define led_end_activity()    do{}while(0)
  #define led_pulse_ms(x)       do{}while(0)
  #define led_update()          do{}while(0)
  static inline void led_hw(bool) {}
#endif

// ===== Flags vom RTC-Interrupt =====
volatile bool g_sample_due = false;
volatile bool g_led_pulse_req = false;

// ===== RTC ISR =====
void rtcSecondISR() {
  g_sample_due = true;
#if LED_ACTIVE
  g_led_pulse_req = true;
#endif
  rtc.setAlarmSeconds((rtc.getSeconds() + 1) % 60); // nächstes Sekundengrid
}

// ===== Mini-yield =====
static void yield_with_led(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { led_update(); delay(1); }
}

// ===== Logging =====
#if defined(ENABLE_SERIAL_LOG) && ENABLE_SERIAL_LOG
  static bool DBG_lineStart = true;
  static inline void DBG_prefix() {
    if (DBG_lineStart) {
      unsigned long ms = millis(), sec = ms/1000, rem = ms%1000;
      Serial.print("["); Serial.print(sec); Serial.print(".");
      if (rem < 100) Serial.print(rem < 10 ? "00" : "0");
      Serial.print(rem); Serial.print("] ");
      DBG_lineStart = false;
    }
  }
  #define LOG(x)   do{ DBG_prefix(); Serial.print(x);}while(0)
  #define LOGLN(x) do{ DBG_prefix(); Serial.println(x); DBG_lineStart = true;}while(0)
  #define LOG_SETUP() do{ Serial.begin(115200); unsigned long _t0=millis(); while(!Serial && (millis()-_t0)<1500){} }while(0)
#else
  #define LOG(x)        do{}while(0)
  #define LOGLN(x)      do{}while(0)
  #define LOG_SETUP()   do{}while(0)
#endif

// ===== Zeit/CSV/Queue =====
#define MIN_VALID_EPOCH   1700000000UL
#define MAX_CATCHUP_SEC   120UL
static inline bool clockValid() { return rtc.getEpoch() >= MIN_VALID_EPOCH; }

static uint32_t lastNtpSyncEpoch = 0;
static long     lastNtpDriftSec  = 0;
static bool     emitTimeDiffOnce = false;

static unsigned long nextNtpTryAtMs = 0;
static uint8_t       ntpRetryCount  = 0;

const char* DAILY_CSV_HEADER = "id;date;time;ntc_t;bme_t;bme_h;bme_p;sht_t;sht_h;vbat;time_diff\n";

#ifndef QSIZE
#define QSIZE 8192
#endif
static char   qbuf[QSIZE];
static size_t qlen = 0;

String   currentCsv;
uint32_t lastYmd = 0;
static unsigned long burstUntilMs = 0;

// ===== Sensoren =====
Adafruit_BME280 bme;
static bool hasBME = false;

static String two(uint8_t v){ char b[3]; sprintf(b,"%02u",v); return String(b); }
static uint32_t ymdKey() { int y=rtc.getYear()+2000, m=rtc.getMonth(), d=rtc.getDay(); return (uint32_t)(y*10000 + m*100 + d); }
static String dayStampYYMMDD(){ return two((rtc.getYear()+2000)%100) + two(rtc.getMonth()) + two(rtc.getDay()); }

static void ensureDayCsv() {
  uint32_t ymd = ymdKey();
  if (ymd != lastYmd || currentCsv.length()==0) {
    lastYmd = ymd;
    currentCsv = dayStampYYMMDD() + ".csv";
    LOGLN(String("Day CSV: ") + currentCsv);
    File f = SD.open(currentCsv, FILE_WRITE);
    if (f) {
      if (f.size() == 0) { f.println(DAILY_CSV_HEADER); f.flush(); }
      f.close();
    }
  }
}

// ---- PDP & HTTP helpers ----
static bool bringupPDP() {
  for (int i=1; i<=5; ++i) {
    LOG("NB.begin attempt "); LOGLN(i);
    if (nb.begin(SIM_PIN, APN, APN_USER, APN_PASS) != NB_READY) { yield_with_led(1200); continue; }
    if (gprs.attachGPRS() != GPRS_READY) { yield_with_led(1200); continue; }
    IPAddress ip = gprs.getIPAddress();
    if (ip[0]||ip[1]||ip[2]||ip[3]) { LOG("PDP IP: "); LOGLN(ip); return true; }
  }
  return false;
}

static bool readLine(Client &c, String &line, unsigned long ms) {
  line = ""; unsigned long t0 = millis();
  while (millis()-t0 < ms) {
    while (c.available()) {
      char ch = (char)c.read();
      if (ch=='\r') continue;
      if (ch=='\n') return true;
      line += ch;
    }
    led_update(); delay(2);
  }
  return line.length()>0;
}

static bool readHttpHeaders(Client &c, int &status, String &location, unsigned long timeout_ms=15000) {
  status = 0; location = ""; String line;
  if (!readLine(c, line, timeout_ms)) return false;
  if (line.startsWith("HTTP/1.")) {
    int sp = line.indexOf(' ');
    if (sp>0 && sp+3 < (int)line.length()) status = line.substring(sp+1, sp+4).toInt();
  }
  unsigned long t0 = millis();
  while (millis()-t0 < timeout_ms) {
    if (!readLine(c, line, 2000)) { continue; }
    if (line.length()==0) break;
    if (line.startsWith("Location: ") || line.startsWith("location: "))
      location = line.substring(line.indexOf(':')+2);
  }
  return (status>0);
}

static void resolveRedirect(const String &loc, const String &baseHost, const String &basePath,
                            String &outHost, String &outPath) {
  outHost = baseHost; outPath = basePath;
  if (loc.startsWith("https://")) {
    int s = 8, slash = loc.indexOf('/', s);
    if (slash > 0) { outHost = loc.substring(s, slash); outPath = loc.substring(slash); }
    else { outHost = loc.substring(s); outPath = "/"; }
  } else if (loc.startsWith("/")) outPath = loc;
  else {
    if (!outPath.endsWith("/")) { int p=outPath.lastIndexOf('/'); outPath=(p>=0)? outPath.substring(0,p+1)+loc : String("/") + loc; }
    else outPath += loc;
  }
}

static bool httpsPostAppsScript(const char* host, uint16_t port, String path,
                                const uint8_t* body, size_t len, int maxHops = 4) {
  String h = host, p = path;
  for (int hop=0; hop<=maxHops; ++hop) {
    NBSSLClient cli; cli.setTimeout(25000);
    if (!cli.connect(h.c_str(), port)) { LOGLN("HTTPS connect failed"); return false; }
    cli.print("POST "); cli.print(p); cli.print(" HTTP/1.1\r\n");
    cli.print("Host: "); cli.print(h); cli.print("\r\n");
    cli.print("User-Agent: MKRNB-logger/1\r\n");
    cli.print("Accept: */*\r\n");
    cli.print("Content-Type: application/x-ndjson\r\n");
    cli.print("Content-Length: "); cli.print((unsigned long)len); cli.print("\r\n");
    cli.print("Connection: close\r\n\r\n");
    if (cli.write(body, len) != len) { LOGLN("short write"); cli.stop(); return false; }
    int status = 0; String loc;
    if (!readHttpHeaders(cli, status, loc)) { LOGLN("no HTTP headers"); cli.stop(); return false; }
    cli.stop();
    if (status >= 200 && status < 300) return true;
    if (status == 302 || status == 303) { LOGLN("Apps Script 302/303 -> OK"); return true; }
    if ((status == 307 || status == 308) && loc.length()) {
      String nh, np; resolveRedirect(loc, h, p, nh, np);
      h = nh; p = np; continue;
    }
    return false;
  }
  return false;
}

static bool httpsGetOnce(const char* host, uint16_t port, const String &path,
                         int &statusOut, String &locationOut) {
  NBSSLClient cli; cli.setTimeout(20000);
  if (!cli.connect(host, port)) { LOGLN("HTTPS connect failed"); return false; }
  cli.print("GET "); cli.print(path); cli.print(" HTTP/1.1\r\n");
  cli.print("Host: "); cli.print(host); cli.print("\r\n");
  cli.print("User-Agent: MKRNB-logger/1\r\n");
  cli.print("Accept: */*\r\n");
  cli.print("Connection: close\r\n\r\n");
  int status=0; String loc;
  if (!readHttpHeaders(cli, status, loc)) { LOGLN("no HTTP headers"); cli.stop(); return false; }
  cli.stop(); statusOut=status; locationOut=loc; return true;
}

static bool probePing() {
  String p = "/macros/s/" + String(DEPLOY_ID) + "/exec?mode=diag&token=" + String(TOKEN);
  int st=0; String loc;
  bool ok = httpsGetOnce(APP_HOST, APP_PORT, p, st, loc);
  LOGLN(ok ? String("PING status=")+st : "PING failed");
  if (ok && (st == 302 || st == 303)) LOGLN(String("Redirect to: ") + loc);
  return ok && st>=200 && st<400;
}


// ===== Queue helpers =====
static void dropOldestLineUntil(size_t need) {
  while (qlen + need > QSIZE && qlen > 0) {
    size_t i = 0; while (i < qlen && qbuf[i] != '\n') ++i;
    if (i < qlen) { size_t cut = i+1; memmove(qbuf, qbuf + cut, qlen - cut); qlen -= cut; }
    else { qlen = 0; }
  }
}
static void normalizeQueueHead() {
  if (qlen == 0) return;
  if (qbuf[0] == '\n') { memmove(qbuf, qbuf + 1, qlen - 1); qlen -= 1; return; }
  if (qbuf[0] != '{' && qbuf[0] != '[') {
    size_t i = 0; while (i < qlen && qbuf[i] != '\n') ++i;
    if (i < qlen) { size_t cut = i + 1; memmove(qbuf, qbuf + cut, qlen - cut); qlen -= cut; }
    else { qlen = 0; }
  }
}
static size_t bytesUpToLastNewline(size_t maxBytes) {
  if (maxBytes == 0 || qlen == 0) return 0;
  size_t lim = (maxBytes < qlen) ? qlen : maxBytes;
  for (size_t i = lim; i > 0; --i) if (qbuf[i-1] == '\n') return i;
  return 0;
}

// ===== Upload =====
static bool postBufferedLines() {
  if (qlen == 0) return true;
  normalizeQueueHead();
  if (qlen == 0) return true;

  size_t cap     = (qlen < (size_t)SEND_MAX_BYTES) ? qlen : (size_t)SEND_MAX_BYTES;
  size_t sendLen = bytesUpToLastNewline(cap);
  if (sendLen == 0) { LOGLN("post: no newline within cap"); return true; }

  String path = "/macros/s/"; path += DEPLOY_ID;
  path += "/exec?token="; path += TOKEN;
  path += "&device=";     path += DEVICE_ID;

  led_begin_activity(LED_MODE_UPLOAD_2HZ);
  bool ok = httpsPostAppsScript(APP_HOST, APP_PORT, path, (const uint8_t*)qbuf, sendLen, 4);
  led_end_activity();

  if (ok) {
    memmove(qbuf, qbuf + sendLen, qlen - sendLen);
    qlen -= sendLen;
    LOGLN(String("post ok, qlen->") + qlen);
  } else {
    LOGLN("post failed");
  }
  return ok;
}

static void serviceUploadsNonBlocking() {
  if (!g_netReady) return;
  static unsigned long nextTick = 0;
  if (millis() < nextTick && qlen == 0) return;

  const bool inBurst = (millis() < burstUntilMs);
  unsigned long budget = (inBurst || qlen > (SEND_MAX_BYTES*2/3)) ? SEND_BUDGET_MS_FAST : SEND_BUDGET_MS_SLOW;

  unsigned long t0 = millis();
  uint8_t chunks = 0;
  do {
    if (!postBufferedLines()) break;
    ++chunks;
    if (MAX_CHUNKS_PER_CYCLE > 0 && chunks >= MAX_CHUNKS_PER_CYCLE) break;
  } while (qlen > 0 && (millis() - t0) < budget);

  nextTick = millis() + (qlen > 0 ? SEND_FAST_PERIOD_MS : SEND_PERIOD_MS);
}

// ===== NTP =====
#if USE_NTP
static bool ntpSyncNow() {
  led_begin_activity(LED_MODE_FAST_BLINK);
  uint8_t pkt[48] = {0};
  pkt[0] = 0b11100011; pkt[2] = 6; pkt[3] = 0xEC; pkt[12]=49; pkt[13]=0x4E; pkt[14]=49; pkt[15]=52;

  if (!udp.beginPacket(NTP_HOST, NTP_PORT)) { LOGLN("NTP: beginPacket failed"); led_end_activity(); return false; }
  if (!udp.write(pkt, sizeof(pkt)))         { LOGLN("NTP: write failed");       led_end_activity(); return false; }
  if (!udp.endPacket())                     { LOGLN("NTP: endPacket failed");   led_end_activity(); return false; }

  unsigned long t0 = millis();
  while (millis() - t0 < 5000UL) {
    int sz = udp.parsePacket();
    if (sz >= 48) {
      uint32_t before = rtc.getEpoch();
      udp.read(pkt, 48);
      uint32_t s = (uint32_t)pkt[40]<<24 | (uint32_t)pkt[41]<<16 | (uint32_t)pkt[42]<<8 | pkt[43];
      uint32_t epoch = s - 2208988800UL;

      lastNtpDriftSec  = (long)epoch - (long)before;
      rtc.setEpoch(epoch);
      lastNtpSyncEpoch = epoch;

      burstUntilMs = millis() + BOOT_BURST_SECONDS * 1000UL;
      lastYmd = 0; currentCsv = "";
      emitTimeDiffOnce = true;

      led_end_activity();
      return true;
    }
    led_update(); delay(20);
  }
  LOGLN("NTP timeout");
  led_end_activity();
  return false;
}

static void ntpSchedulerTick() {
  const unsigned long nowMs = millis();

  // Falls UDP noch nicht läuft: periodisch erneut versuchen (z.B. alle 10s)
  if (!g_udpReady) {
    if (nowMs >= nextNtpTryAtMs) {
      LOGLN("Scheduler: try udp.begin()");
      g_udpReady = udp.begin(2390);
      if (!g_udpReady) {
        LOGLN("Scheduler: udp.begin() failed; retry in 10s");
        nextNtpTryAtMs = nowMs + 10000UL;   // 10s
      } else {
        LOGLN("Scheduler: udp.begin() OK");
        nextNtpTryAtMs = nowMs + 1000UL;    // kurze Pause
      }
    }
    return;  // ohne UDP macht NTP kein Sinn
  }

  // Ab hier ist UDP bereit → NTP-Intervall-/Backoff-Logik
  const uint32_t nowEpoch = rtc.getEpoch();
  bool intervalDue = (lastNtpSyncEpoch == 0) || (nowEpoch - lastNtpSyncEpoch >= NTP_SYNC_INTERVAL_S);
  bool allowedByBackoff = (nowMs >= nextNtpTryAtMs);
  if (!intervalDue || !allowedByBackoff) return;

  bool ok = ntpSyncNow();
  if (ok) {
    ntpRetryCount   = 0;
    nextNtpTryAtMs  = nowMs + 1000UL;
    LOGLN("Scheduler: NTP sync OK");
  } else {
    unsigned long waitMs = (unsigned long)NTP_RETRY_MIN_MS << (ntpRetryCount < 6 ? ntpRetryCount : 6);
    if (waitMs > NTP_RETRY_MAX_MS) waitMs = NTP_RETRY_MAX_MS;
    ntpRetryCount++;
    nextNtpTryAtMs = nowMs + waitMs;
    LOGLN(String("Scheduler: NTP failed, retry in [ms]: ")+waitMs);
  }
}

#endif

// ===== Messung =====
static float readBattery() { int raw=analogRead(PIN_BAT); return raw * (2.0f * 3.3f / 1023.0f); } // 100k/100k
static float readNTC() {
  int raw = analogRead(PIN_NTC);
  float R = (10000.0f * raw) / (1023.0f - raw);
  float invT = log(R/10000.0f) / NTC_BETA + 1.0f/(25.0f+273.15f);
  return (1.0f/invT) - 273.15f + NTC_OFFSET;
}

// ==== SHT3x Compat-Makros ====
// Repeatability
#if defined(SHT3X_REPEATABILITY_HIGH)
  #define SHTX_REP_HIGH SHT3X_REPEATABILITY_HIGH
#elif defined(REPEATABILITY_HIGH)
  #define SHTX_REP_HIGH REPEATABILITY_HIGH
#elif defined(SHT35_REPEATABILITY_HIGH)
  #define SHTX_REP_HIGH SHT35_REPEATABILITY_HIGH
#else
  #define SHTX_REP_HIGH 2
#endif
// 1 Hz MPS-Mapping (viele Lib-Varianten):
#ifndef SHTX_MPS_1
  #if defined(MEASUREMENT_MPS_1)
    #define SHTX_MPS_1 MEASUREMENT_MPS_1
  #elif defined(SHT3X_MPS_1HZ)
    #define SHTX_MPS_1 SHT3X_MPS_1HZ
  #elif defined(SHT3X_MPS_1_HZ)
    #define SHTX_MPS_1 SHT3X_MPS_1_HZ
  #elif defined(SHT3X_MPS_1)
    #define SHTX_MPS_1 SHT3X_MPS_1
  #endif
#endif
// 1-Parameter-API?
#if defined(SHT3X_PERIODIC_1MPS_HIGH) || defined(SHT3X_PERIODIC_1MPS_MEDIUM) || defined(SHT3X_PERIODIC_1MPS_LOW)
  #define SHTX_HAS_1PARAM_API 1
#endif

static bool initSensors() {
  Wire.begin(); Wire.setClock(100000);

  // --- BME280 ---
  hasBME = bme.begin(BME_ADDRESS, &Wire);
  if (!hasBME) {
    LOGLN("BME280 not found");
  } else {
    bme.setSampling(
      Adafruit_BME280::MODE_NORMAL,
      Adafruit_BME280::SAMPLING_X2,      // Temp
      Adafruit_BME280::SAMPLING_X4,      // Druck
      Adafruit_BME280::SAMPLING_X2,      // Feuchte
      Adafruit_BME280::FILTER_X2,
      Adafruit_BME280::STANDBY_MS_1000   // 1 Hz
    );
  }

  // --- SHT85 / SHT3x ---
  sht3x.begin(Wire, 0x44);  // SHT85: meist 0x44
  hasSHT = false;
  sht_use_single_shot = false;

  // 1) Versuche 1-Parameter-API (kombinierte Konstante)
  #if defined(SHTX_HAS_1PARAM_API)
    {
      int16_t err = sht3x.startPeriodicMeasurement(SHT3X_PERIODIC_1MPS_HIGH);
      if (err == 0) {
        hasSHT = true;
        LOGLN("SHT3x periodic: 1-param API (1 Hz, HIGH)");
      } else {
        LOGLN(String("SHT3x periodic(1param) err=") + err);
      }
    }
  #endif

  // 2) Falls noch nicht aktiv: versuche 2-Parameter-API
  #if !defined(SHTX_HAS_1PARAM_API)
    #if defined(SHTX_MPS_1)
      if (!hasSHT) {
        int16_t err = sht3x.startPeriodicMeasurement(SHTX_REP_HIGH, SHTX_MPS_1);
        if (err == 0) {
          hasSHT = true;
          LOGLN("SHT3x periodic: 2-param API (1 Hz, HIGH)");
        } else {
          LOGLN(String("SHT3x periodic(2param) err=") + err);
        }
      }
    #endif
  #endif

  // 3) Fallback: Single-Shot
  if (!hasSHT) {
    sht_use_single_shot = true;
    hasSHT = true;  // wir probieren Single-Shot im Betrieb
    LOGLN("SHT3x fallback: single-shot mode (1 sample/sec)");
  }

  return hasBME || hasSHT;
}

// ==== SHT lesen (periodic oder single-shot) ====
static void readSHT85(float &t, float &h) {
  t = NAN; h = NAN;
  if (!hasSHT) return;

  int16_t err = 0;

  if (sht_use_single_shot) {
    // === Single-Shot: deine Lib will (Repeatability, clockStretching, float&, float&)
    float tf = 0.0f, hf = 0.0f;

    #if defined(REPEATABILITY_HIGH)
      err = sht3x.measureSingleShot(REPEATABILITY_HIGH, false, tf, hf);
    #elif defined(SHT3X_REPEATABILITY_HIGH)
      // manche älteren Varianten
      err = sht3x.measureSingleShot(SHT3X_REPEATABILITY_HIGH, false, tf, hf);
    #elif defined(SHT35_REPEATABILITY_HIGH)
      err = sht3x.measureSingleShot(SHT35_REPEATABILITY_HIGH, false, tf, hf);
    #else
      // Falls deine Lib keinen der Namen hat, nimm MEDIUM als Fallback:
      #if defined(REPEATABILITY_MEDIUM)
        err = sht3x.measureSingleShot(REPEATABILITY_MEDIUM, false, tf, hf);
      #else
        // letzter Ausweg: HIGH = 2 ist in Sensirion-Enums üblich
        err = sht3x.measureSingleShot((Repeatability)2, false, tf, hf);
      #endif
    #endif

    if (err == 0) { t = tf; h = hf; }
  } else {
    // === Periodic: deine Header zeigen readMeasurement(uint16_t&, uint16_t&)
    uint16_t tTicks = 0, rhTicks = 0;
    err = sht3x.readMeasurement(tTicks, rhTicks);
    if (err == 0) {
      t = -45.0f + 175.0f * (float)tTicks / 65535.0f;   // °C
      h = 100.0f * (float)rhTicks / 65535.0f;           // %RH
    }
  }

  // Plausibilitätscheck
  if (err != 0) { t = NAN; h = NAN; return; }
  if (!(t > -45.0f && t < 130.0f)) t = NAN;
  if (!(h >= 0.0f  && h <= 100.0f)) h = NAN;
}


// ==== Linien schreiben ====
static void epochToDateParts(uint32_t epoch, uint8_t& YY, uint8_t& MO, uint8_t& DD,
                             uint8_t& hh, uint8_t& mm, uint8_t& ss) {
  time_t t = (time_t)epoch;
  struct tm *ptm = gmtime(&t);
  int year = ptm->tm_year + 1900;
  YY = (uint8_t)(year % 100);
  MO = (uint8_t)(ptm->tm_mon + 1);
  DD = (uint8_t)ptm->tm_mday;
  hh = (uint8_t)ptm->tm_hour;
  mm = (uint8_t)ptm->tm_min;
  ss = (uint8_t)ptm->tm_sec;
}

static void writeOneSampleToSdAndQueueAt(uint32_t sampleEpoch,
                                         float ntc, float bmeT, float bmeH, float bmeP,
                                         float shtT, float shtH, float vbat)
{
  uint8_t YY, MO, DD, hh, mm, ss;
  epochToDateParts(sampleEpoch, YY, MO, DD, hh, mm, ss);
  char d6[7], hms[9];
  snprintf(d6,  sizeof(d6),  "%02u%02u%02u", YY, MO, DD);
  snprintf(hms, sizeof(hms), "%02u:%02u:%02u", hh, mm, ss);

  long tdiffToReport = LONG_MIN;
  if (emitTimeDiffOnce) {
    if (labs(lastNtpDriftSec) > 86400L) lastNtpDriftSec = 0;
    tdiffToReport = lastNtpDriftSec;
    emitTimeDiffOnce = false;
  }

  // CSV
  ensureDayCsv();
  if (File df = SD.open(currentCsv, FILE_WRITE)) {
    df.print(SENSOR_ID); df.print(';');
    df.print(d6);        df.print(';');
    df.print(hms);       df.print(';');
    auto csvNum = [&df](float v){ if (isfinite(v)) df.print(v, 2); df.print(';'); };
    csvNum(ntc); csvNum(bmeT); csvNum(bmeH); csvNum(bmeP);
    csvNum(shtT); csvNum(shtH); csvNum(vbat);
    if (tdiffToReport == LONG_MIN) df.println(); else df.println(tdiffToReport);
    df.flush(); df.close();
  }

  // NDJSON
  auto numOrNull = [](char* out, size_t outsz, float v){
    if (isfinite(v)) snprintf(out, outsz, "%.2f", v);
    else { strncpy(out, "null", outsz); out[outsz-1] = '\0'; }
  };
  char s_ntc[16], s_bmeT[16], s_bmeH[16], s_bmeP[16], s_shtT[16], s_shtH[16], s_vbat[16], s_tdiff[24];
  numOrNull(s_ntc,  sizeof(s_ntc),  ntc);
  numOrNull(s_bmeT, sizeof(s_bmeT), bmeT);
  numOrNull(s_bmeH, sizeof(s_bmeH), bmeH);
  numOrNull(s_bmeP, sizeof(s_bmeP), bmeP);
  numOrNull(s_shtT, sizeof(s_shtT), shtT);
  numOrNull(s_shtH, sizeof(s_shtH), shtH);
  numOrNull(s_vbat, sizeof(s_vbat), vbat);
  if (tdiffToReport == LONG_MIN) strcpy(s_tdiff, "null");
  else snprintf(s_tdiff, sizeof(s_tdiff), "%ld", tdiffToReport);

  char line[256];
  int n = snprintf(line, sizeof(line),
    "{\"id\":\"%s\",\"time\":%lu,"
    "\"ntc_t\":%s,\"bme_t\":%s,\"bme_h\":%s,\"bme_p\":%s,"
    "\"sht_t\":%s,\"sht_h\":%s,\"vbat\":%s,\"time_diff\":%s}\n",
    SENSOR_ID, (unsigned long)sampleEpoch,
    s_ntc, s_bmeT, s_bmeH, s_bmeP, s_shtT, s_shtH, s_vbat, s_tdiff);

  if (n > 0) {
    size_t nn = (size_t)n;
    if (nn <= QSIZE) {
      if (qlen + nn > QSIZE) dropOldestLineUntil(nn);
      if (qlen + nn <= QSIZE) { memcpy(qbuf + qlen, line, nn); qlen += nn; }
    }
  }
}

// ===== LED Boot-Helper =====
static void led_flash(uint8_t n, unsigned on_ms, unsigned off_ms) {
  for (uint8_t i = 0; i < n; ++i) {
    led_hw(true);  delay(on_ms);
    led_hw(false); delay(off_ms);
  }
}
static void led_show_step(uint8_t stepNo) {
  led_flash(stepNo, 200, 200);
  delay(BOOT_STEP_GAP_MS);
}

// ---------- Buzzer ----------
#if BUZZER_ACTIVE
static inline void buzz(unsigned freq, unsigned ms) {
  tone(BUZZER_PIN, freq, ms);
  delay(ms + 10);
  noTone(BUZZER_PIN);
}
static void beep_code(uint8_t n) { for (uint8_t i = 0; i < n; ++i) { buzz(1400, 90); delay(110); } delay(250); }
static void beep_ok()    { buzz(1600, 80); delay(60); buzz(1900, 120); }
static void beep_error() { buzz(400, 160); delay(80);  buzz(300, 220); }
#else
static inline void buzz(unsigned, unsigned) {}
static inline void beep_code(uint8_t) {}
static inline void beep_ok() {}
static inline void beep_error() {}
#endif

// ===== Boot-Sequencer =====
static bool run_boot_sequence() {
  // STEP 1: LTE attach
  led_show_step(1);
  bool lte_ok = false;
  for (int attempt = 1; attempt <= 5 && !lte_ok; ++attempt) {
    lte_ok = bringupPDP();
    if (!lte_ok) { led_flash(1, 500, 250); beep_error(); delay(1200); }
  }
  if (!lte_ok) return false;
  beep_ok();

  // STEP 2: PDP/IP sichtbar bestätigen
  led_show_step(2);
  delay(300);

  // STEP 3: UDP + NTP  (NICHT-FATAL, falls UDP/NTP zunächst scheitert)
led_show_step(3);

// UDP starten
g_udpReady = udp.begin(2390);
if (!g_udpReady) {
  LOGLN("udp.begin failed -> will retry later in scheduler");
  // Sichtbares Fehlerzeichen, aber KEIN Abbruch:
  led_flash(3, 500, 250); /*beep_error();*/
} else {
  // NTP versuchen (max 6x). Auch das ist nicht-fatal.
  bool ntp_ok = false;
  for (int i = 0; i < 6 && !ntp_ok; ++i) {
    ntp_ok = ntpSyncNow();             // zeigt FAST_BLINK während des Versuchs
    if (!ntp_ok) { led_flash(3, 400, 200); delay(1500); }
  }
  if (!ntp_ok) { LOGLN("NTP failed (boot) -> scheduler will retry"); /*beep_error();*/ }
  else         { LOGLN("NTP ok (boot)"); /*beep_ok();*/ }
}


  // STEP 4: SD + Sensoren
  led_show_step(4);
  bool sd_ok = SD.begin(PIN_SD_CS);
  bool sens_ok = initSensors();
  if (!sd_ok)   { led_flash(4, 600, 250); beep_error(); }
  if (!sens_ok) { led_flash(4, 600, 250); beep_error(); }
  if (sd_ok) ensureDayCsv();

  // STEP 5: RTC Sekundentakt
  led_show_step(5);
  rtc.attachInterrupt(rtcSecondISR);
  rtc.disableAlarm();
  rtc.setAlarmSeconds((rtc.getSeconds() + 1) % 60);
  rtc.enableAlarm(RTCZero::MATCH_SS);
  beep_ok();

  // STEP 6: Plausibilität – Dry-Run
  led_show_step(6);
  {
    float ntc = readNTC();
    float b_t = NAN, b_p = NAN, b_h = NAN, s_t = NAN, s_h = NAN;
    if (hasBME) {
      b_t = bme.readTemperature() + BME_TEMP_OFFSET;
      b_p = bme.readPressure()/100.0f + BME_PRESS_OFFSET;
      b_h = bme.readHumidity() + BME_HUM_OFFSET;
    }
    (void)ntc;
    readSHT85(s_t, s_h);

    bool plaus_ok = true;
    if (isfinite(b_t) && (b_t < TEMP_MIN || b_t > TEMP_MAX)) plaus_ok = false;
    if (isfinite(b_p) && (b_p < PRESS_MIN || b_p > PRESS_MAX)) plaus_ok = false;
    if (isfinite(b_h) && (b_h < HUM_MIN   || b_h > HUM_MAX))   plaus_ok = false;

    if (!plaus_ok) { led_flash(6, 600, 250); beep_error(); }
    else           { beep_ok(); }
  }

  led_show_step(7);
g_netReady = true;
bool pingok = probePing();
if (!pingok) { LOGLN("Apps Script ping failed"); led_flash(7, 600, 250); /*beep_error();*/ }
else { LOGLN("Apps Script ping ok"); /*beep_ok();*/ }

{
  const char* one =
    "{\"id\":\"" DEVICE_ID "\",\"time\":0,"
    "\"ntc_t\":null,\"bme_t\":null,\"bme_h\":null,\"bme_p\":null,"
    "\"sht_t\":null,\"sht_h\":null,\"vbat\":null,\"time_diff\":null}\n";
  size_t nn = strlen(one);
  if (nn <= QSIZE && qlen + nn <= QSIZE) { memcpy(qbuf + qlen, one, nn); qlen += nn; }
}
serviceUploadsNonBlocking();  // sollte TLS „warm“ machen und senden


  // STEP 8: Run mode
  led_show_step(8);
  led_set_mode(LED_MODE_MEAS_PULSE);
  beep_ok();

  return true;
}

// ===== Setup / Loop =====
void setup() {
  pinMode(PIN_SD_CS, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  // Sichtbarer LED-Selbsttest beim Boot (zeigt sofort, ob Invertierung stimmt)
led_hw(true);  delay(300);
led_hw(false); delay(200);
led_hw(true);  delay(300);
led_hw(false); delay(200);

  led_hw(true); // Power-On

  LOG_SETUP();
  LOGLN("== Boot sequencer start ==");

  rtc.begin();

  bool ok = run_boot_sequence();

  if (!ok) {
    while (true) {
      led_flash(9, 400, 200);
      beep_error();
      delay(1500);
    }
  }

  LOGLN("== Boot sequencer done; entering run mode ==");
}

void loop() {
  led_update();

#if USE_NTP
  ntpSchedulerTick();
#endif

  static unsigned long lastDayChk = 0;
  if (millis() - lastDayChk > 3000) { ensureDayCsv(); lastDayChk = millis(); }

#if LED_ACTIVE
  if (g_led_pulse_req) { g_led_pulse_req = false; led_pulse_ms(LED_PULSE_MS); }
#endif

  if (g_sample_due) {
    g_sample_due = false;
    const uint32_t sec = rtc.getEpoch();

    float ntc = readNTC();
    float b_t = hasBME ? (bme.readTemperature() + BME_TEMP_OFFSET) : NAN;
    float b_p = hasBME ? (bme.readPressure()/100.0f + BME_PRESS_OFFSET) : NAN;
    float b_h = hasBME ? (bme.readHumidity() + BME_HUM_OFFSET) : NAN;

    float s_t, s_h; readSHT85(s_t, s_h);
    float vb  = readBattery();

    writeOneSampleToSdAndQueueAt(sec, ntc, b_t, b_h, b_p, s_t, s_h, vb);
  }

  uint32_t msToNextSec = 1000 - (millis() % 1000);
  if (msToNextSec > 250) {
    serviceUploadsNonBlocking();
  }

  delay(2);
}
