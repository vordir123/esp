# ESP Example Project

This repository demonstrates a simple ESP-based controller with a web dashboard.
It is intended for ESP32 boards but can be adapted to other ESP variants.

## Hardware Connections

| Function        | GPIO | Notes                          |
|-----------------|------|--------------------------------|
| Start button    | 0    | Use internal pull-up           |
| PWM output      | 18   | Connect servo signal wire here |
| Analog sensor   | 34   | Requires 10&nbsp;k&Omega; pull-up |

All grounds must be tied together and the sensor should be powered from the
ESP's 3.3&nbsp;V rail. The servo can be powered separately if it draws more current
than the board can supply.

### Pinout Diagram

```
            +-------------------+
            |      ESP32        |
            |                   |
    3.3V ---+ [3V3]          GND +--- GND
   Button ---+ [GPIO0]      GPIO18 +--- Servo signal
 Sensor  ---+ [GPIO34]        VIN +--- 3.3V sensor
            +-------------------+
```

## Building with PlatformIO in VS Code

1. Install [Visual Studio Code](https://code.visualstudio.com/) and the
   [PlatformIO extension](https://platformio.org/install/ide?install=vscode).
2. Open this repository folder in VS Code.
3. When PlatformIO prompts to initialize the project, accept the defaults.
4. Press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>B</kbd> (or use the **Build**
   task) to compile the firmware.
5. Use **Upload** to flash the binary to the board.

Required libraries are listed in `platformio.ini` and can be installed with

```bash
pio lib install ESPAsyncWebServer ArduinoJson
```

## Calibration and Web Dashboard

After flashing, the device starts an access point named `ESP-Setup`. Connect to
this network and open `http://192.168.4.1` in a browser. The dashboard lets you

- set Wi‑Fi credentials,
- monitor sensor readings,
- move the servo using a slider,
- store calibration values (min/max positions).

Calibration saves the current settings to non‑volatile storage so they persist
after reboot. You can revisit the dashboard at any time by browsing to the
module's IP address on your local network.

## Libraries Used

- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)

Refer to their documentation for detailed usage and licensing information.
