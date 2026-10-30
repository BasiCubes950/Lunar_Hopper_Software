# D5 · IMU Module: What Changed

Notion task: **D5 · IMU Orientation and Event Detection: Code Structure**

Goal: turn the BNO055 example into an IMU module that drops straight into the main program and answers four questions: how far is the hopper leaning, is it in the air, did it just hit the ground, and has it stopped moving.

The file layout is unchanged. The only new files are `IMUModule.ino` and this document; nothing was moved or renamed. `LegActuators.ino` (D1) is untouched.

## Summary

| File | Change |
|---|---|
| `IMUModule.ino` | **New.** The D5 module |
| `HopperLegs.ino` | Calls the IMU module; stops the legs on an IMU fault |
| `SelfRighting.ino` | Gets its IMU data from the module (self-righting logic itself unchanged) |
| `IMU.ino` | Compile guard added at top and bottom only; original code untouched |
| `three_l16_actuators.ino` | Compile guard added at top and bottom only; original code untouched |
| `LegActuators.ino` | No change |

---

## 1. `IMUModule.ino` (new)

This is the only file that talks to the BNO055, and it owns the `bno` object.

### Notion requirements, and how they are met

| Notion requirement | Implementation |
|---|---|
| Remove `delay()` | Each call checks `millis()`; the sensor is read every 10 ms, otherwise the call returns immediately |
| Tilt from the gravity direction | `tilt = acos(g.z / |g|)`, not the X/Y/Z Euler angles, so it stays correct on its side or upside down |
| Replace `while(1);` | If the sensor isn't found, or reads all zeros 20 times in a row, the module enters a fault state and the program keeps running |
| One function each for tilt, falling, impact, still | `imuTiltDeg()`, `imuIsFalling()`, `imuImpactDetected()`, `imuIsStill()` |
| Limits as named settings at the top | See the settings table below |
| Same Serial speed as the leg code | The main program sets 115200 for everything |
| Own `begin()` and `update()` | `imuBegin()` and `imuUpdate()` |

The sensor runs in `OPERATION_MODE_IMUPLUS` (accelerometer + gyro fusion, no magnetometer, since motors and magnets would disturb it). This is the same mode D2 already used.

### Settings (top of the file; values are placeholders to tune in person)

| Setting | Value | Meaning |
|---|---|---|
| `IMU_SIMULATE` | on | Pretend values, no sensor needed. **Comment out once the BNO055 is wired** |
| `IMU_DEBUG` | on | Print the IMU status to Serial every 250 ms |
| `IMU_READ_INTERVAL_MS` | 10 | Read interval (100 Hz), fast enough to catch a landing spike |
| `FALL_ACCEL_MS2` / `FALL_TIME_MS` | 3.0 / 60 | Total acceleration below 3 m/s² for 60 ms means falling (in free fall the accelerometer reads close to 0) |
| `IMPACT_ACCEL_MS2` | 30 | Total acceleration above this means impact. Fusion mode fixes the accelerometer range at ±4 g (about 39 m/s²), so this must stay below 39 |
| `IMPACT_LOCKOUT_MS` | 300 | Bounces within this time after an impact are not counted |
| `IMU_STILL_ACCEL_MS2` / `IMU_STILL_GYRO_DPS` / `IMU_STILL_TIME_MS` | 0.3 / 3.0 / 500 | Linear acceleration and rotation both small for 500 ms means still (acceleration and time are D2's previous values) |
| `IMU_MAX_BAD_READS` | 20 | Consecutive all-zero reads before the sensor is treated as lost |

### Functions

```cpp
bool  imuBegin();             // once in setup(); returns false if the sensor isn't found (no freeze)
void  imuUpdate();            // every loop(); returns immediately if it isn't time to read yet
float imuTiltDeg();           // tilt: 0 upright, 90 on its side, 180 upside down
bool  imuIsFalling();         // in the air
bool  imuImpactDetected();    // just hit the ground; true ONCE per impact, cleared when read
bool  imuIsStill();           // stopped moving
bool  imuHasFault();          // sensor missing or lost
unsigned long imuStillDurationMs();           // how long it has been still (ms); 0 while moving
void  imuGetUpVector(float&, float&, float&); // unit "up" vector in the hopper's frame; (0,0,1) when upright
void  imuSimCommand(char c);                  // switch pretend-data scenarios from Serial keys
```

### Pretend-data mode (`IMU_SIMULATE`)

Type a capital letter in the Serial Monitor (115200). Capitals are used so they don't clash with the leg keys r / f / h / 0.

| Key | Scenario |
|---|---|
| `U` | Upright, still |
| `T` | Tilted 30°, still (starts self-righting automatically) |
| `S` | On its side (90°), still (starts self-righting automatically) |
| `D` | Upside down (180°), still |
| `M` | Moving / shaking |
| `F` | Falling (free fall) |
| `L` | Landing impact (a spike of about 20 ms) |

Example Serial output:

```
IMU tilt=30.0 falling=0 impacts=1 still=1 fault=0  |acc|=9.81 |lin|=0.05 |gyro|=0.3
```

### IMU mounting (to be decided)

The code assumes +z points up when the hopper is upright. If the IMU is mounted differently, remap the axes in one place: `imuReadSensor()` (the real-sensor version). Use a proper rotation, not just flipping the sign of one axis. Everything else follows automatically.

---

## 2. `HopperLegs.ino` (main program)

- Added at the top:
  - `#define HOPPER_MAIN_SKETCH`, which makes the two standalone test files switch themselves off when compiled together with the main program (see section 4).
  - `extern const uint8_t NUM_LEGS;` and `extern const float LAUNCH_POSE_MM;`. Both are defined in `LegActuators.ino`, but Arduino compiles the main file first, which used to give "not declared in this scope". Declaring them up front removes the dependence on compile order.
- `setup()`: `selfRightingBegin()` is replaced by `imuBegin()`. If the sensor is missing at power-up, the legs only finish the homing move that `beginLegs()` already does, then accept no further moves. The legs are not "stopped in place" at boot because their position isn't known reliably yet.
- `loop()`:
  - Calls `imuUpdate()` at the start of every loop.
  - If the IMU faults while running, the three legs are held where they are (once), `IMU FAULT - legs stopped` is printed, and self-righting and Serial commands are skipped.
  - Every character read from Serial is also passed to `imuSimCommand(c)` (the pretend-data keys).
- The existing r / f / h / 0 keys and the state printout are unchanged.

## 3. `SelfRighting.ino` (D2)

The self-righting state machine and the leg-length calculation (`computeLegTargets`) are unchanged. Only the sensor reading was replaced with calls to the IMU module.

- **Removed:**
  - The three `#include` lines
  - The `bno` object
  - `selfRightingBegin()` and `readUpVector()`
  - The constants `STILL_ACCEL_MS2`, `STILL_TIME_MS`, `RAD2DEG_D2`
  - The variables `monStill`, `monStillStart`, `d2Still`, `d2StillStart`
- **Waiting to be still** (`D2_WAIT_STILL`): previously D2 read linear acceleration and timed it itself. It now requires `IMU_STILL_TIME_MS` of continuous stillness counted from when the wait began, which is the same behaviour as before:
  ```cpp
  if (min(imuStillDurationMs(), now - d2WaitStart) >= IMU_STILL_TIME_MS) d2State = D2_DECIDE;
  ```
- **Deciding** (`D2_DECIDE`): tilt and the up vector now come from `imuTiltDeg()` and `imuGetUpVector()`.
- **Automatic start** (`checkAutoSelfRighting`):
  ```cpp
  if (imuIsStill() && imuTiltDeg() > AUTO_TRIGGER_TILT_DEG) startSelfRighting();
  ```
- Added `d2EnterWaitStill(now)`, which records the start time when entering the wait state.
- Because D2 now reads from the IMU module, pretend-data mode also drives self-righting, so the whole flow can be tested without hardware.

## 4. `IMU.ino` and `three_l16_actuators.ino`

Not a single line of the original code changed. One line was added at the top and one at the bottom:

```cpp
#ifndef HOPPER_MAIN_SKETCH
...original code...
#endif  // HOPPER_MAIN_SKETCH
```

- Opened on its own: it works exactly as before. `IMU.ino` stays as the wiring test, as Notion asks.
- Compiled with the main program: its contents are skipped, so there are no more duplicate `setup()` / `loop()` / `bno` / `STROKE_MM` errors.

---

## How to test

1. Install the libraries **Adafruit BNO055** and **Adafruit Unified Sensor**.
2. Arduino IDE needs the main file to have the same name as its folder, and names can't contain spaces, so the `Arduino Code` folder can't be opened directly. Instead, copy all the `.ino` files from here into a folder named `HopperLegs` and open `HopperLegs.ino`.
3. Set the Serial Monitor to **115200**.
4. With `IMU_SIMULATE` on, type `T`, `U`, `F`, `L`, `S` in turn. Watch `tilt` / `falling` / `impacts` / `still` and the `self-right state` line (0 idle, 1 waiting for still, 2 deciding, 3 moving, 4 finishing, 5 done, 6 failed).
5. With the real BNO055 connected, comment out `#define IMU_SIMULATE`, then:
   - Check calibration with `bno.getCalibration()`.
   - Measure and tune the thresholds at the top of the file.
   - Confirm how the IMU is mounted.

Tested so far (simulated environment, not real hardware):

- The whole folder compiled together the way Arduino does it, with `HopperLegs.ino` first: passes.
- Each of the two standalone test files compiled on its own: passes.
- Tilted or on its side starting self-righting, falling, landing, still, sensor lost while running, and no sensor at boot: all behave correctly.

Not yet compiled or run in the real Arduino IDE or on a board.

## Notion "Done when" checklist

- [x] No `delay()` and no `while(1);` left in the IMU code
- [x] Tilt comes from the gravity direction
- [x] Each of the four outputs works on the Serial Monitor with pretend values
- [x] The IMU code has its own begin / update functions (`imuBegin()` / `imuUpdate()`)
