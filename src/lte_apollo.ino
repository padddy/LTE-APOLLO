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

bool syncTime() {
  DEBUG_PRINTLN("Syncing time...");
  // In a real implementation this would obtain time via LTE
  rtc.begin();
  rtc.setEpoch(0);
  DEBUG_PRINTLN("Time synced");
  return true;
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
  File stat = SD.open(String(SENSOR_ID) + "_stat0.txt", FILE_WRITE);
  DEBUG_PRINTLN("Writing stat file");
  for (int i = 0; i < 3; ++i) {
    appendData(stat);
  }
  stat.close();
  // TODO: FTP upload of stat file

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
    // TODO: send status via FTP
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
