# LTE-APOLLO

Dieses Projekt demonstriert eine einfache Firmware für den Arduino MKR NB 1500.
Der Sensor erfasst Temperatur, Druck und Feuchte mit einem BME280 sowie eine
Temperatur über einen NTC. Die Daten werden auf einer SD-Karte gespeichert und
per LTE auf einen FTP-Server hochgeladen.

## Struktur
- `include/config.h` – Konfiguration und Pins
- `src/lte_apollo.ino` – Hauptprogramm

Die Werte werden im CSV-Format in Tagesdateien gespeichert. Statusmeldungen und
Messdaten können an einen FTP-Server gesendet werden. Das Programm ist als
Grundlage gedacht und muss ggf. an die eigenen Anforderungen angepasst werden.

Die Uhrzeit wird per LTE über NTP synchronisiert. Dazu wird ein UDP-Paket an
`time.nist.gov` gesendet und der empfangene Zeitstempel in die RTC
übernommen. Die Differenz zwischen der alten RTC-Zeit und der neuen Zeit wird
als `T_DIFF` in Millisekunden bei jeder Statusmeldung protokolliert.

### Debug
Durch Aktivieren von `DEBUG_SERIAL` in `config.h` werden Statusmeldungen über die
serielle Schnittstelle ausgegeben (Baudrate siehe `DEBUG_BAUD`).
