# ESP32 Blink Beispiel

Dieses Projekt enthält eine minimale Firmware, die die interne LED des ESP32 im Sekundentakt blinken lässt.

## Installation

Das Skript `install.sh` installiert das ESP-IDF falls erforderlich, baut die Firmware und flasht sie auf die Platine. Der serielle Port kann über die Umgebungsvariable `BOARD_PORT` angepasst werden (Standard ist `/dev/ttyUSB0`).

```bash
./install.sh
```
