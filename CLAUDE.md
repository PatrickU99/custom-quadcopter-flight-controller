# Quadcopter PID Flight Controller Project

## Goal
Custom quadcopter build with a **self-written PID flight controller** — explicitly NOT using Betaflight or other off-the-shelf firmware. Point of the project: write the control loop, motor mixing, and sensor fusion from scratch. Resume-grade project demonstrating embedded systems / control theory skill.

**Current stage:** Hardware bring-up essentially complete (motors spin on command, IMU reading live data). Next step: writing the actual PID controller.

**Important:** I want to write the PID/control logic myself. Don't provide PID, sensor fusion, or motor mixing code unless I specifically ask for it — I'm using it as a learning/portfolio project and want to implement it myself. Conceptual help, debugging help on code I've written, and explanations are welcome at any time.

---

## Hardware

### Frame
QAV250-class build (250mm), 5" props.

### Flight controller / MCU
- **ESP32-S3-DevKitC-1 (N16R8 — 16MB flash, 8MB PSRAM)**, Type-C, Arduino-compatible.
- Switched from an STM32F411CEU6 Black Pill (WeAct/SHILLEHTEK) after that board broke. STM32-specific work (DFU flow, USB CDC config, `Servo` on TIM4/TIM1) is legacy/reference only.
- PlatformIO (VS Code + PlatformIO IDE extension), Arduino framework.
- Working `platformio.ini`:
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_speed = 460800

build_flags = 
    -DARDUINO_USB_CDC_ON_BOOT=1
```
- **Gotcha:** `ARDUINO_USB_CDC_ON_BOOT=1` is REQUIRED or `Serial` never enumerates as USB.
- **Only call `Serial.begin()` once**, in `main.cpp`'s `setup()`. Calling it again elsewhere (e.g. `mpuSetup()`) resets USB CDC and silently kills Serial output for the rest of the boot. Already cost significant debug time — don't repeat.

### ESC
4x single 30A ESCs (Amazon ASIN B0CZNM4H7H), individual units (not 4-in-1).

Originally started with an AERO SELFIE 45A 4-in-1 ESC (8-pin JST-SH) — legacy/reference only. Switched to 4x singles due to soldering difficulty and preference for pre-terminated bullet connectors.

**Firmware decision:** compared SimonK (PWM-only) vs AM32 (DShot/telemetry/adjustable timing). Conclusion: for this project, advanced features aren't required — a basic PWM-only ESC gives sufficiently consistent, proportional throttle response for the PID loop. Telemetry/RPM-filtering would only matter for later noise-filtering refinement, not initial control loop.


### Motors
Readytosky MT2204 2300KV (2x CW + 2x CCW), correctly sized for QAV250 w/ 5-6" props, 2-3S, ~440g max thrust. Pre-soldered gold-plated 3.5mm bullet connectors.

### IMU
MPU6050 via I2C:
- SDA → GPIO8, SCL → GPIO9, VCC → 3.3V, GND → GND
- Using **Adafruit_MPU6050** library (deliberate choice — "from scratch" priority applies to PID/control logic, not every peripheral driver).
```ini
lib_deps = 
    adafruit/Adafruit MPU6050@^2.2.6
    adafruit/Adafruit Unified Sensor@^1.1.14
    adafruit/Adafruit BusIO@^1.14.5
```
- Must call `Wire.begin(8, 9)` BEFORE `mpu.begin(0x68, &Wire)`, or the library defaults to wrong I2C pins.
- `mpu.getEvent(&a, &g, &temp)` called every `loop()`, not `setup()`. Currently global so other functions can access latest reading.
- **Units: accel in m/s² (not g's), gyro in rad/s (not deg/s).** Conversion: `degrees = radians * (180/PI)`.
- Filter bandwidth: `MPU6050_BAND_21_HZ` (accel ±2G, gyro ±250°/s) — starting point, may need retuning once vibration is real.
- **Gyro axis mapping (gx/gy/gz → roll/pitch/yaw) NOT yet empirically verified** against physical frame orientation. Standard convention (X=roll, Y=pitch, Z=yaw) assumed but sign/direction depends on physical chip mounting — must test by hand before trusting in the PID loop.

### Battery
3S LiPo, ~12.6V full charge, confirmed compatible.

### Propellers
Labeled L/R (CCW/CW). Rotation direction per motor position not yet finally confirmed against frame's CW/CCW convention.

---

## Firmware structure (PlatformIO `src/`)

- `main.cpp` — setup()/loop() only, calls into other modules
- `mpu.h` / `mpu.cpp` — MPU6050 setup + read functions
- Motor control currently inline in `main.cpp` — should be split into `motor.h`/`motor.cpp` before PID code adds complexity

**Rule:** only `main.cpp` contains `setup()`/`loop()`. Every other file exposes plain functions (`mpuSetup()`, `mpuRead()`, etc.) called from `main.cpp` — avoids "multiple definition of loop()" linker error already hit once.

### Motor control (current working implementation)
- 4 motors on GPIO4, 5, 6, 7 (confirmed working). GPIO45 avoided — unreliable strapping pin. Avoid strapping pins 0, 3, 45, 46 and native-USB pins 19, 20 for motor signals.
- Uses ESP32 LEDC peripheral via older API: `ledcSetup(channel, freq, resolution)` + `ledcAttachPin(pin, channel)` + `ledcWrite(channel, duty)` — NOT `ledcAttach()` one-liner (needs newer core than installed, caused build failures).
- **Each motor needs its own unique LEDC channel (0-3)** — reusing a channel across pins silently steals/breaks the first pin's signal. Already hit and fixed.
- **Critical working parameter: PWM_FREQ = 50Hz, PWM_RES = 12-bit.** 16-bit at 50Hz silently failed (pin read ~3.7V garbage instead of clean toggling) — the frequency/resolution combo exceeded what the LEDC clock divider could achieve; `ledcSetup()` doesn't error clearly on this. **Don't change without re-testing.**
- ARM_US = 1000 (also idle value), SPIN_US = 1300 (arbitrary low test throttle) — placeholders, not tuned.
- Arm sequence: hold ARM_US for 4 seconds before changing throttle.
- Serial manual control: 's' to spin at SPIN_US, 'p' to stop (returns to ARM_US).

**SAFETY:** 'p' stop-via-serial is convenience only, NOT a real safety mechanism — only works if ESP32 is still running and receiving the keypress. **A hardware-level failsafe (physical disarm button on a GPIO, or a watchdog) is still needed before real flight testing** — flagged multiple times, not yet implemented. Always keep the battery connector within immediate physical reach during any powered motor test.

---

## Personal constraints relevant to advice
- Real difficulty with fine soldering (solder not wetting to ESC pads — likely heat transfer / big-pad heat-sinking / flux / tip condition issues). Drove decisions toward pre-terminated connectors and minimizing solder joints.
- **The actual value of this project is the PID controller and control theory work** — not soldering skill, and not necessarily hand-rolling every low-level peripheral driver (hence using the Adafruit MPU6050 library).
- Comfortable with C/C++ fundamentals but still building fluency on: variable scope (global/local), `extern`, `const` vs mutable, header/implementation separation, include guards. Explanations at this level are welcome, not a sign of a bigger issue.
- Prefers direct, concrete, step-by-step troubleshooting. Has already worked through a lot of hardware/toolchain debugging (DFU flow, USB CDC, LEDC resolution bug, linker errors, double Serial.begin()).

---

## Open items (as of last update)
1. **PID controller — not started.** Next major piece of work. I want to write this myself.
2. Gyro axis-to-roll/pitch/yaw mapping not empirically verified against physical frame mounting.
3. Motor-position-to-GPIO mapping (M1-M4 vs GPIO4/5/6/7) not finalized against frame's CW/CCW convention — needs to match whatever mixing logic is chosen.
4. Prop rotation direction per motor position not confirmed/matched.
5. No hardware failsafe/watchdog for motor cutoff yet.

7. Soldering motor phase wires to ESC — need to confirm ESC-side bullet connectors match gender/size (3.5mm) of motor's pre-installed bullets.
8. `motor.h`/`motor.cpp` split not yet done.

## Reference material
Used Ryan Boland's "Embedded Programming for Quadcopters" talk (Tanooki Labs, ~2015) as inspiration for PID controller structure — covers gyro/accel complementary filter, rate-mode PI loop (P+I only, no D — deemed too noise-susceptible), motor mixing equations, and stabilize mode (angle PID feeding rate PID). His code repo: github.com/bolandrm/rmb_multicopter — used for conceptual reference only, not for copying code.

## Repository
GitHub: github.com/PatrickU99, separate repo from earlier ESP32-Voice-Assistant project. Standard PlatformIO `.gitignore` (excludes `.pio/` and VSCode cache files).