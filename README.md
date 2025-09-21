# Balance Board Project

## Important Links

- Waveshare ESP32-S3-Matrix: https://www.waveshare.com/wiki/ESP32-S3-Matrix

## Build, Flash, and Monitor (ESP-IDF idf.py)

Prerequisites:
- ESP-IDF installed and environment exported (run export.sh / export.ps1 from your ESP-IDF install).
- Board connected over USB. Know its serial port (e.g., /dev/ttyACM0 on Linux, COMx on Windows, /dev/cu.usbmodem… on macOS).

Typical workflow from the project root:

1) Select target (example uses ESP32-S3; change if you use a different chip):
   idf.py set-target esp32s3

2) Configure (optional):
   idf.py menuconfig

3) Build the application:
   idf.py build

4) Flash and open serial monitor:
   idf.py -p <PORT> flash monitor

- Exit monitor with Ctrl+].
- If you don’t specify -p, ESP-IDF will try to auto-detect.

Clean builds:
- Remove build artifacts entirely (useful if switching targets/IDF versions):
  idf.py fullclean

## Running Unit Tests (ESP-IDF Unit Test App)

This project uses the ESP-IDF Unit Test App to run component tests. Tests live under each component’s test/ folder. Currently available:
- Component: utilities
  - QueuedNotificationService unit tests

Key points:
- Normal app builds do not include tests.
- To run tests, use the Unit Test App via idf.py -T.

Run only the utilities component tests:
   idf.py -T utilities -p <PORT> flash monitor

Run all discovered component tests in the project:
   idf.py -T all -p <PORT> flash monitor

Notes:
- The Unit Test App will build and flash a special test firmware, then run the Unity tests and print results in the serial monitor.
- After running tests, run a normal build/flash (without -T) to put your regular firmware back on the device.
