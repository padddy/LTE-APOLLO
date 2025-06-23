#ifndef CONFIG_H
#define CONFIG_H

// Sensor and hardware configuration
#define SENSOR_ID "APOLLO001"
#define PIN_NTC A0
#define PIN_BAT A1
#define PIN_BUTTON A2
#define PIN_SD_CS 4
#define LED_PIN LED_BUILTIN
#define BME_ADDRESS 0x76
#define NTC_BETA 3976
#define NTC_OFFSET 0.0

// Plausibility ranges
#define TEMP_MIN -40
#define TEMP_MAX 100
#define PRESS_MIN 900
#define PRESS_MAX 1100
#define HUM_MIN 10
#define HUM_MAX 100

#define FTP_SERVER "ftp.example.com"
#define FTP_USER "user"
#define FTP_PASS "pass"
#define FTP_PORT 21
#define FTP_STAT_DIR "/status"
#define FTP_DATA_DIR "/data"

// NTP server for time synchronization via UDP
#define NTP_SERVER_IP IPAddress(129, 6, 15, 28) // time.nist.gov
#define NTP_LOCAL_PORT 2390

#define SIM_PIN "1234"
#define SIM_APN "apn"

// Intervals
#define TIME_SYNC_INTERVAL_MIN 60
#define DAILY_UPLOAD_TIME "23:55:00"


#define SOFTWARE_VERSION "1.0"

// Debugging over serial monitor
#define DEBUG_SERIAL 1
#define DEBUG_BAUD 9600

#endif // CONFIG_H
