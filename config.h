// ================== CONFIG ==================
#pragma once

// --- Mobilfunk ---
#define SIM_PIN   "0000"
#define APN       "internet"
#define APN_USER  ""
#define APN_PASS  ""

// --- Gerätekennung ---
#define DEVICE_ID "AA"

// --- Google Apps Script ---
#define DEPLOY_ID "AKfycbw8QV9mzmfdb3WFB41NUkwUeyN3BWxh1Q8Y0f2e_f_-z7TzH-rott7IVe40KihxJkv0cQ"
#define TOKEN     "e36b58b3007648ebcb56fcda4d2d19432cece33fb821019ff2d1bae90524dc46"
#define APP_HOST  "script.google.com"
#define APP_PORT  443

// --- Upload-Tuning ---
#define SEND_MAX_BYTES        8192   // pro Chunk
#define SEND_PERIOD_MS        4000   // Standard-Intervall
#define SEND_FAST_PERIOD_MS   1000   // wenn Rückstand/Fehler
//#define SEND_BUDGET_MS        300   // Zeitbudget pro Zyklus zum "Leeren"
#define MAX_CHUNKS_PER_CYCLE  0      // Sicherheitsdeckel (≈ 18 KB/Zyklus)
#define SEND_BUDGET_MS_SLOW    150   // normale Entleer-Zeit pro Tick
#define SEND_BUDGET_MS_FAST    600   // Boot-/Rückstau-Entleer-Zeit
#define BOOT_BURST_SECONDS     30    // wie lange nach Start/NTP wir "schnell" leeren

#define ENABLE_UPLOADS 1   // 0 = Messung nur auf SD, 1 = auch HTTP Upload

// --- Puffer ---
#define QSIZE        16384         // 12 KB Queue (8–12 KB ist gut)
#define SD_MAXLINE   256

// --- SD ---
#define PIN_SD_CS    5

// --- NTP/RTC ---
#define USE_NTP   1
#define NTP_HOST "0.pool.ntp.org"
#define NTP_PORT 123
#define NTP_SYNC_INTERVAL_S    (1UL*3600UL)   //(24UL*3600UL)   // alle 24h neu syncen
#define NTP_RETRY_MIN_MS       15000UL         // erster Retry nach 15s
#define NTP_RETRY_MAX_MS       600000UL        // maximale Retry-Pause 10min

// --- Sensoren/Hardware ---
#define SENSOR_ID "AA"
#define PIN_NTC   A0
#define PIN_BAT   A1
#define PIN_BUTTON A2
#define LED_PIN   LED_BUILTIN

#define BME_ADDRESS 0x77
#define BME_TEMP_OFFSET  0.0
#define BME_PRESS_OFFSET 0.0
#define BME_HUM_OFFSET   0.0
#define NTC_BETA   3976
#define NTC_OFFSET 0.0

// --- LED / Status ---
#define LED_ACTIVE              1     // 0 = LED komplett aus
#define LED_PULSE_MS            60    // Mess-Puls-Dauer
#define LED_SLOW_ON_MS          250   // LTE connect -> an
#define LED_SLOW_OFF_MS         750   // LTE connect -> aus
#define LED_FAST_ON_MS          120   // Time-Sync -> an
#define LED_FAST_OFF_MS         180   // Time-Sync -> aus  (≈ 3.3 Hz)
#define LED_UPLOAD_PERIOD_MS    500   // 2 Blitze/Sekunde (alle 500ms)
#define LED_UPLOAD_PULSE_MS     60
#define LED_NOLTE_PERIOD_MS     1000  // Kein LTE: 1x kurz aus/sec
#define LED_NOLTE_OFF_MS        80

// ===== UI =====
#define LED_INVERT       0        // 0: HIGH = an (Default auf MKR), 1: invertieren
#define BUZZER_ACTIVE    0        // 1, wenn Piezo angeschlossen
#define BUZZER_PIN       6        // PWM-Pin für tone(); beliebig anpassen

// Sichtbare Schritt-Pausen
#define BOOT_STEP_GAP_MS 600      // Pause zwischen Schritt-Anzeigen


// Plausibilitäten
#define TEMP_MIN  -40
#define TEMP_MAX  100
#define PRESS_MIN 900
#define PRESS_MAX 1100
#define HUM_MIN   10
#define HUM_MAX   100

// Logging
#define LOG_VERBOSE 0

// Logging
#define ENABLE_SERIAL_LOG 1   // 1 = Serial-Logging an, 0 = komplett aus

