# Balance Board Project

## Purpose

The purpose of this project is to use the ESP32-S3-Matrix device, attached to a balance board, as a bluetooth mouse for my computer. The idea is to attach a battery to the ESP32-S3-Matrix and have it send all the click/movement information to the computer, thus keeping my muscles from becoming dystrophied. 

So, this project uses the gyroscope/accelerometer on the device to detect and simulate mouse movement, clicks, and scroll wheel.  It then acts as a standard Bluetooth HID device that can be discovered by the computer and tuned like any other HID device.  

The LED matrix in the device is used to give visual feedback to the user, such as:

1. device is in bluetooth pairing mode (eg. if it has no bluetooth connection), and whether pairing succeeded or failed
2. left click, middle click, right click (fast tilt of the device on the left, forward, or right), and whether a button is being held (drag)
3. scroll wheel up, scroll wheel down (rotate the board about its vertical axis)
4. cursor movement (a small "ball" that follows the board's tilt)

Note: the device does **not** detect double clicks. It only reports individual button down/up events; a
double click is simply two clicks in quick succession, and the host OS decides (using its own double-click
speed setting) whether they form a double click. Likewise, click vs. drag is decided by the host from how
long the button stays down and whether the cursor moves meanwhile.

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

Building with Docker (no local ESP-IDF install needed), from the repository root:

   docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/project -w /project espressif/idf:v5.5.4 idf.py build

Clean builds:
- Remove build artifacts entirely (useful if switching targets/IDF versions):
  idf.py fullclean

## Running Unit Tests (ESP-IDF Unit Test App)

This project uses the ESP-IDF Unit Test App to run component tests. Tests live under each component’s test/ folder. Currently available:
- Component: utilities
  - QueuedNotificationService unit tests

Important: The idf.py -T option is only available in newer ESP-IDF versions. If your idf.py does not recognize -T, either update ESP-IDF or use the fallback method described below.

Option A — Using idf.py -T (newer ESP-IDF):
- Normal app builds do not include tests.
- Run the Unit Test App by selecting tests with -T.

Run only the utilities component tests:
   idf.py -T utilities -p <PORT> flash monitor

Run all discovered component tests in the project:
   idf.py -T all -p <PORT> flash monitor

Option B — Fallback for older ESP-IDF (no -T support):
You can run tests using ESP-IDF’s Unit Test App project and point it to this repository’s components.

1) Open an ESP-IDF environment (run export.sh / export.ps1).
2) Change directory to the Unit Test App provided by ESP-IDF:
   cd "$IDF_PATH/examples/system/unit_test"
3) Configure CMake to discover this project’s components and select which tests to run by passing these options to idf.py
   (BALANCE_BOARD_DIR is the root of this repository; set it before changing directory, e.g. `export BALANCE_BOARD_DIR="$PWD"` from the repo root):
   -D EXTRA_COMPONENT_DIRS="$BALANCE_BOARD_DIR/components" \
   -D TEST_COMPONENTS="utilities"

Example commands to build, flash, and monitor only the utilities tests on the target:
   idf.py -D EXTRA_COMPONENT_DIRS="$BALANCE_BOARD_DIR/components" -D TEST_COMPONENTS="utilities" -p <PORT> flash monitor

To run all tests available to the Unit Test App (including those discovered in this repo via EXTRA_COMPONENT_DIRS):
   idf.py -D EXTRA_COMPONENT_DIRS="$BALANCE_BOARD_DIR/components" -D TEST_COMPONENTS=all -p <PORT> flash monitor

Notes:
- The Unit Test App will build and flash a special test firmware, then run the Unity tests and print results in the serial monitor.
- After running tests, return to your project directory and do a normal build/flash to restore your regular firmware.

## Running Host Unit Tests (no hardware needed)

The hardware-independent logic (gesture detection, calibration, HID report encoding, the LED renderer,
the BOOT-button press classifier and the 50% LED intensity ceiling) lives in portable components
(`components/mouse`, `components/gestures`, `components/led_feedback`) and is unit tested on the host
(x86/Linux) under `host_test/`, using Unity and AddressSanitizer + UndefinedBehaviorSanitizer.

From the repository root:

   host_test/run_host_tests.sh            # builds and runs inside the espressif/idf Docker image
   host_test/run_host_tests.sh --local    # uses a local cmake + C++20 compiler instead

## Device Behavior

### Bluetooth (BLE HID mouse)

- BLE-only, using the NimBLE host and ESP-IDF's `esp_hid` HID-over-GATT device profile
  (`components/ble_hid_mouse`). It shows up as a mouse named **BalanceMouse**.
- Sends standard 4-byte mouse reports: buttons (bit0 left, bit1 right, bit2 middle), X, Y and wheel
  (each a signed 8-bit value). One report goes out for every button down/up, every movement sample and
  every scroll tick, but only while a host is connected and the link is encrypted.
- Pairing is "Just Works" (there is no display or keypad), with bonding. Bonds are stored in NVS. When no
  host is connected the board advertises. That is pairing mode if it has no bond, and it is also how an
  already-bonded computer reconnects by itself after the board reboots.
- A long press of the BOOT button (see below) forgets the bond and goes back to pairing mode, disconnecting
  first if a host is connected.

### LED matrix feedback

All LEDs are capped at **50% intensity**. The cap is enforced inside `LightPixel` itself, so no code path can
exceed it, because the matrix heats up quickly. Coordinates are as seen from the LED side with the USB-C
connector at the top (see `components/led_feedback/include/MatrixLayout.hpp`).

| State | Display |
|---|---|
| Pairing mode / waiting for a host | whole matrix breathes slowly in blue (3 s period) |
| Pairing succeeded | two quick full-matrix green flashes, then the normal mouse display |
| Pairing failed | three quick full-matrix red flashes, then back to the blue breathing |
| Connected, idle or moving | a 2x2 white "ball" that follows the board's tilt (centred at rest) |
| Scrolling | a column of purple chevrons scrolling continuously in the scroll direction |
| Left button | far-left column green: solid for a click, blinking once held > 0.5 s (drag) |
| Right button | far-right column amber: solid for a click, blinking once held > 0.5 s (drag) |
| Middle button | top row cyan: solid for a click, blinking once held > 0.5 s (drag) |

### Calibration

Two independent baselines are subtracted from the raw readings: one for tilt (accelerometer X/Y, which
drives the cursor) and one for yaw (which drives the scroll wheel).

- At power-on, the stance the board starts in counts as "zero". The baseline is not taken from the very first
  sample: right after the IMU is reset its data registers can still read all-zero, which would make the board's
  real resting tilt look like a permanent lean (phantom cursor drift). The first 10 samples (~200 ms) are
  discarded, then the baseline is seeded from the first sample whose total acceleration is a plausible resting
  reading (0.5 g to 1.5 g). Nothing is sent to the host until then.
- After that, each baseline slowly follows the resting position (exponential moving average, 5 s time
  constant), but **only while completely idle**: no button held, not moving, not scrolling, and both signals
  below their start thresholds. A held drag or a long scroll is therefore never absorbed into the baseline.
- A small, steady offset in how you naturally stand (below the start thresholds) is absorbed within about
  15 s. That keeps the movement and scroll ranges symmetric.
- A larger permanent change of stance after power-on looks like deliberate movement or scrolling. Press the
  BOOT button briefly to re-zero.

### BOOT button (GPIO0)

- **Short press** (released before 3 s): immediately re-zeroes both calibration baselines to the current position.
- **Long press** (held 3 s): fires at the 3 s mark without waiting for release. It forgets the BLE bond
  and restarts pairing mode, disconnecting the current host first if one is connected.
- The button is debounced (30 ms) and handled by an any-edge interrupt plus a small task that polls while the button is held.
