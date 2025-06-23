#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <MKRNB.h>
#include <RTCZero.h>
#include "config.h"

#ifdef DEBUG_SERIAL
#  define DEBUG_PRINT(x)   Serial.print(x)
#  define DEBUG_PRINTLN(x) Serial.println(x)
#else
#  define DEBUG_PRINT(x)
#  define DEBUG_PRINTLN(x)
#endif

Adafruit_BME280 bme;
NB nbAccess;
NBClient client;
GPRS gprs;
NBFileUtils nbfs;
RTCZero rtc;
NBUDP Udp;
IPAddress ntpServer NTP_SERVER_IP;
unsigned int localPort = NTP_LOCAL_PORT;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
long timeDiffMs = 0; // difference between NTP time and RTC in ms

File dataFile;

unsigned long lastMinute = 0;
unsigned long lastHour = 0;
unsigned long startDay = 0;

// Helpers to access RTC values in a familiar way
int year() { return rtc.getYear() + 2000; }
int month() { return rtc.getMonth(); }
int day() { return rtc.getDay(); }
int hour() { return rtc.getHours(); }
int minute() { return rtc.getMinutes(); }
int second() { return rtc.getSeconds(); }

void blinkError(int hz) {
  int delayMs = 500 / hz;
  for (int i = 0; i < hz*2; ++i) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(delayMs);
  }
  digitalWrite(LED_PIN, LOW);
}

float readNTC() {
  int raw = analogRead(PIN_NTC);
  float resistance = (1023.0 / raw - 1.0) * 10000.0; // 10k pullup
  float steinhart;
  steinhart = resistance / 10000.0;       // (R/Ro)
  steinhart = log(steinhart);             // ln(R/Ro)
  steinhart /= NTC_BETA;                  // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25.0 + 273.15);     // + (1/To)
  steinhart = 1.0 / steinhart;            // Invert
  steinhart -= 273.15;                    // convert to C
  steinhart += NTC_OFFSET;
  return steinhart;
}

float readBattery() {
  int raw = analogRead(PIN_BAT);
  return raw * (2.0 * 3.3 / 1023.0); // 100k/100k divider
}

bool checkBME() {
  DEBUG_PRINTLN("Checking BME280...");
  if (!bme.begin(BME_ADDRESS)) {
    DEBUG_PRINTLN("BME280 not found");
    return false;
  }
  DEBUG_PRINTLN("BME280 found");
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0;
  float h = bme.readHumidity();
  return (t > TEMP_MIN && t < TEMP_MAX &&
          p > PRESS_MIN && p < PRESS_MAX &&
          h > HUM_MIN && h < HUM_MAX);
}

bool initLTE() {
  DEBUG_PRINTLN("Initializing LTE...");
  if (nbAccess.begin(SIM_PIN, SIM_APN) != NB_READY) {
    DEBUG_PRINTLN("LTE init failed");
    return false;
  }
  DEBUG_PRINTLN("LTE ready");
  return true;
}

unsigned long sendNTPpacket(IPAddress &address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
  return millis();
}

bool syncTime() {
  DEBUG_PRINTLN("Syncing time via NTP...");
  Udp.begin(localPort);
  sendNTPpacket(ntpServer);
  delay(1000);
  if (Udp.parsePacket()) {
    Udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = (highWord << 16) | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    rtc.begin();
    unsigned long before = rtc.getEpoch();
    timeDiffMs = ((long)epoch - (long)before) * 1000L;
    rtc.setEpoch(epoch);
    DEBUG_PRINT("Epoch: ");
    DEBUG_PRINTLN(epoch);
    DEBUG_PRINT("RTC diff ms: ");
    DEBUG_PRINTLN(timeDiffMs);
    DEBUG_PRINTLN("Time synced via NTP");
    return true;
  }
  DEBUG_PRINTLN("NTP no response");
  return false;
}

bool initSD() {
  DEBUG_PRINTLN("Initializing SD...");
  if (!SD.begin(PIN_SD_CS)) {
    DEBUG_PRINTLN("SD init failed");
    return false;
  }
  DEBUG_PRINTLN("SD ready");
  return true;
}

String timeStamp() {
  char buf[7];
  sprintf(buf, "%02d%02d%02d", year() % 100, month(), day());
  return String(buf);
}

void appendData(File &f) {
  char buf[96];
  snprintf(buf, sizeof(buf), "%02d%02d%02d;%02d:%02d:%02d;%.2f;%.2f;%.2f;%.2f;%.2f",
           year() % 100, month(), day(), hour(), minute(), second(),
           readNTC(), bme.readTemperature(), bme.readPressure() / 100.0,
           bme.readHumidity(), readBattery());
  f.println(buf);
  DEBUG_PRINTLN(buf);
}

void appendStatusData(File &f) {
  char buf[120];
  snprintf(buf, sizeof(buf), "%02d%02d%02d;%02d:%02d:%02d;%.2f;%.2f;%.2f;%.2f;%.2f;%ld",
           year() % 100, month(), day(), hour(), minute(), second(),
           readNTC(), bme.readTemperature(), bme.readPressure() / 100.0,
           bme.readHumidity(), readBattery(), timeDiffMs);
  f.println(buf);
  DEBUG_PRINTLN(buf);
}

bool ftpUpload(const String &localName, const char *remoteDir) {
  File f = SD.open(localName);
  if (!f) {
    DEBUG_PRINTLN("FTP open failed");
    return false;
  }

  NBClient ftp;
  if (!ftp.connect(FTP_SERVER, FTP_PORT)) {
    DEBUG_PRINTLN("FTP connect failed");
    f.close();
    return false;
  }

  ftp.print("USER " FTP_USER "\r\n");
  delay(200);
  ftp.print("PASS " FTP_PASS "\r\n");
  delay(200);
  ftp.print("TYPE I\r\n");
  delay(200);
  ftp.print("EPSV\r\n");
  delay(500);

  String resp = "";
  while (ftp.available()) resp += (char)ftp.read();
  int dataPort;
  if (sscanf(resp.c_str(), "229 Entering Extended Passive Mode (|||%d|)",
             &dataPort) != 1) {
    ftp.stop();
    f.close();
    DEBUG_PRINTLN("EPSV parse failed");
    return false;
  }
  NBClient data;
  if (!data.connect(FTP_SERVER, dataPort)) {
    ftp.stop();
    f.close();
    DEBUG_PRINTLN("data connect failed");
    return false;
  }

  ftp.print("STOR ");
  ftp.print(remoteDir);
  ftp.print("/");
  ftp.print(localName);
  ftp.print("\r\n");
  delay(200);

  while (f.available()) {
    data.write(f.read());
  }
  data.stop();
  ftp.print("QUIT\r\n");
  ftp.stop();
  f.close();
  DEBUG_PRINTLN("FTP upload done");
  return true;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

#ifdef DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  while (!Serial) {
    ;
  }
  DEBUG_PRINTLN("\nStarting " SENSOR_ID " firmware " SOFTWARE_VERSION);
#endif

  delay(15000);
  if (!initLTE()) {
    blinkError(4);
  }
  DEBUG_PRINTLN("LTE initialized");
  if (!syncTime()) {
    blinkError(4);
  }
  DEBUG_PRINTLN("Time synchronized");
  if (!checkBME()) {
    blinkError(3);
  }
  DEBUG_PRINTLN("BME280 ready");
  if (!initSD()) {
    blinkError(3);
  }
  DEBUG_PRINTLN("SD card ready");
  String statName = String(SENSOR_ID) + "_stat0.txt";
  File stat = SD.open(statName, FILE_WRITE);
  DEBUG_PRINTLN("Writing stat file");
  for (int i = 0; i < 3; ++i) {
    appendStatusData(stat);
  }
  stat.close();
  ftpUpload(statName, FTP_STAT_DIR);

  startDay = millis();
  lastMinute = millis();
  lastHour = millis();
  DEBUG_PRINTLN("Initialization complete");
}

void loop() {
  DEBUG_PRINTLN("Loop start");
  if (!SD.begin(PIN_SD_CS)) {
    digitalWrite(LED_PIN, LOW);
    DEBUG_PRINTLN("SD card not present");
    delay(1000);
    return;
  }

  if (digitalRead(PIN_BUTTON) == LOW) {
    SD.end();
    DEBUG_PRINTLN("SD card unmounted");
    while (digitalRead(PIN_BUTTON) == LOW) {
      delay(100);
    }
    DEBUG_PRINTLN("Button released");
    return;
  }

  if (millis() - lastMinute >= 60000) {
    String filename = String(SENSOR_ID) + timeStamp() + ".csv";
    dataFile = SD.open(filename, FILE_WRITE);
    appendData(dataFile);
    dataFile.close();
    DEBUG_PRINTLN("Data logged");
    lastMinute += 60000;
  }

  if (millis() - lastHour >= 3600000) {
    syncTime();
    DEBUG_PRINTLN("Hourly time sync");
    String statName = String(SENSOR_ID) + "_stat1.txt";
    File stat = SD.open(statName, FILE_WRITE);
    appendStatusData(stat);
    stat.close();
    ftpUpload(statName, FTP_STAT_DIR);
    lastHour += 3600000;
  }

  if (millis() - startDay >= 86400000) {
    // TODO: upload yesterday file via FTP
    DEBUG_PRINTLN("Daily upload trigger");
    startDay += 86400000;
  }

  digitalWrite(LED_PIN, HIGH);
  delay(10);
  digitalWrite(LED_PIN, LOW);
  DEBUG_PRINTLN("Measurement done");
  delay(1000);
}
