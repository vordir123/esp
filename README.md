
# ESP32 Speed Manipulator

This project demonstrates using an ESP32-WROOM-32 module to
calculate rotational speed from a reed switch, manipulate the speed
using PWM, and expose a web dashboard for calibration and statistics.
It also supports OTA firmware updates.

## Wiring

```
Reed switch -> GPIO18 (with pull‑up)
PWM output  -> GPIO19 (to motor driver)
GPIO0       -> Boot/flash as normal
3V3/GND     -> Power
```

## Build Instructions

1. Install [PlatformIO](https://platformio.org/).
2. Clone this repository.
3. Open the project folder in your IDE and build/upload using the
   `esp32dev` platform and `ESP32-WROOM-32` board.

```
pio run --target upload
```

## Usage

After uploading, connect the ESP32 to your Wi‑Fi network (defined in
`ESP32-wroom-32_SpeedManipulator.ino`). Open a browser to the board's
IP to view the dashboard. Use the calibration dialog to adjust the
`pulsesPerRev` value stored in NVS. The dashboard displays current
speed, a history graph and system stats. OTA updates are available at
`/update`.

=======
# ESP32 Blink Beispiel

Dieses Projekt enthält eine minimale Firmware, die die interne LED des ESP32 im Sekundentakt blinken lässt.

## Installation

Das Skript `install.sh` installiert das ESP-IDF falls erforderlich, baut die Firmware und flasht sie auf die Platine. Der serielle Port kann über die Umgebungsvariable `BOARD_PORT` angepasst werden (Standard ist `/dev/ttyUSB0`).

```bash
./install.sh
```


