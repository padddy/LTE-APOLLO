#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_BME280.h>
#include <Arduino_MKRNB.h>
#include "config.h"

Adafruit_BME280 bme;
NB nbAccess;
NBClient client;
GPRS gprs;
NBFileUtils nbfs;

File dataFile;

unsigned long lastMinute = 0;
unsigned long lastHour = 0;
unsigned long startDay = 0;

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
  if (!bme.begin(BME_ADDRESS)) return false;
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0;
  float h = bme.readHumidity();
  return (t > TEMP_MIN && t < TEMP_MAX &&
          p > PRESS_MIN && p < PRESS_MAX &&
          h > HUM_MIN && h < HUM_MAX);
}

bool initLTE() {
  if (nbAccess.begin(SIM_PIN, SIM_APN) != NB_READY) return false;
  return true;
}

bool syncTime() {
  NBProvisioning prov;
  if (!prov.getTime()) return false;
  return true;
}

bool initSD() {
  if (!SD.begin(PIN_SD_CS)) return false;
  return true;
}

String timeStamp() {
  // placeholder for UTC time string
  return String(year()) + String(month()) + String(day());
}

void appendData(File &f) {
  String line = String(year()) + ";" + String(hour()) + ":" + String(minute()) + ":" + String(second());
  line += ";" + String(readNTC(), 2);
  line += ";" + String(bme.readTemperature(), 2);
  line += ";" + String(bme.readPressure() / 100.0, 2);
  line += ";" + String(bme.readHumidity(), 2);
  line += ";" + String(readBattery(), 2);
  f.println(line);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

  delay(15000);
  if (!initLTE()) {
    blinkError(4);
  }
  if (!syncTime()) {
    blinkError(4);
  }
  if (!checkBME()) {
    blinkError(3);
  }
  if (!initSD()) {
    blinkError(3);
  }
  File stat = SD.open(String(SENSOR_ID) + "_stat0.txt", FILE_WRITE);
  for (int i = 0; i < 3; ++i) {
    appendData(stat);
  }
  stat.close();
  // TODO: FTP upload of stat file

  startDay = millis();
  lastMinute = millis();
  lastHour = millis();
}

void loop() {
  if (!SD.begin(PIN_SD_CS)) {
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    return;
  }

  if (digitalRead(PIN_BUTTON) == LOW) {
    SD.end();
    while (digitalRead(PIN_BUTTON) == LOW) {
      delay(100);
    }
    return;
  }

  if (millis() - lastMinute >= 60000) {
    String filename = String(SENSOR_ID) + String(day()) + String(month()) + String(year()) + ".csv";
    dataFile = SD.open(filename, FILE_WRITE);
    appendData(dataFile);
    dataFile.close();
    lastMinute += 60000;
  }

  if (millis() - lastHour >= 3600000) {
    syncTime();
    // TODO: send status via FTP
    lastHour += 3600000;
  }

  if (millis() - startDay >= 86400000) {
    // TODO: upload yesterday file via FTP
    startDay += 86400000;
  }

  digitalWrite(LED_PIN, HIGH);
  delay(10);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
