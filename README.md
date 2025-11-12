# LTE-APOLLO

Dieses Projekt demonstriert eine Firmware für den Arduino MKR NB 1500.
Ein BME280 ermittelt Temperatur, Druck und Luftfeuchte, ein NTC misst an A0 eine
weitere Temperatur. Die Messwerte werden auf einer SD-Karte abgelegt und
täglich per LTE auf einen FTP-Server hochgeladen.

## Struktur
- `include/config.h` – Konfiguration und Pins
- `src/lte_apollo.ino` – Hauptprogramm

Die Werte werden im CSV-Format in Tagesdateien gespeichert. Statusmeldungen und
Messdaten können an einen FTP-Server gesendet werden. Das Programm ist als
Grundlage gedacht und muss ggf. an die eigenen Anforderungen angepasst werden.

Die Uhrzeit wird per NTP (UDP) mit dem Server `ptbtime1.ptb.de`
(IP `192.53.103.108`) synchronisiert. Die Firmware sendet dabei ein NTP-Paket an
Port `123` und aktualisiert nach Empfang des Zeitstempels die interne RTC. Die
Differenz zwischen der vorherigen RTC-Zeit und der neuen Zeit wird als
`T_DIFF` in Millisekunden bei jeder Statusmeldung protokolliert.

### Debug
Durch Aktivieren von `DEBUG_SERIAL` in `config.h` werden Statusmeldungen über die
serielle Schnittstelle ausgegeben (Baudrate siehe `DEBUG_BAUD`).
