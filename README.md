# ESP PlatformIO Project Structure

This repository follows a typical PlatformIO layout:

- **`/src`** – main application code.
- **`/lib`** – custom libraries used by the project.
- **`/data`** – files to upload to SPIFFS (e.g. your HTML dashboard).

To build the firmware locally run:

```bash
platformio run
```

To build and upload to a connected device use:

```bash
platformio upload
```
