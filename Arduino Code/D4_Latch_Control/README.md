# D4 — simulated latch control

**SIMULATION ONLY.** All operations change memory and print to Serial. There are
no pin assignments, hardware writes, or physical release outputs.

Open `D4_Latch_Control.ino` as a standalone Arduino sketch. Use Serial Monitor at
**115200 baud**. Spaces and line endings are ignored.

| Command | Action |
| --- | --- |
| `c` | Close the latch |
| `j` | Request simulated release |
| `s` | Emergency stop; latch a software lockout |
| `r` | Validate current inputs and reset a fault or stop |

The sketch supplies healthy pretend sensor readings at startup. Try `c`, wait for
CLOSED, then `j`. Use `s` to cancel an operation and `r` to acknowledge the stop.
After reset, close again before requesting release. Serial prints the current
state, request acceptance/refusal, and readable failure reasons. All other
commands, including the former sensor/fault injection commands, are rejected.

## Architecture and safety

The short sketch handles commands and reporting. `LatchModule.h` owns all safety
checks and timing, using one namespace, plain variables, enums, and functions.
Include it in one translation unit. It does not depend on other team modules.

Normal states are `IDLE -> CLOSING -> CLOSED -> RELEASING -> RELEASED`. Closing
and release each take 800 ms. `update()` checks inputs and advances operations
without blocking. A 3000 ms timeout takes priority over completion, including
after a long polling gap. Unsigned 32-bit subtraction handles millis rollover.

Release requires confirmed closure, stationary and upright pose, healthy battery
and IMU, and no stop or external fault. Pending release rechecks every condition
before completion. Any unsafe pending release is cancelled immediately when
readings are supplied or `update()` detects the failure. Restoring readings does
not clear the fault. Stops and external faults block both closing and release.
Closing alone does not require healthy pose/battery/IMU readings.

The first fault and its readable reason remain latched until `resetLockout()`
succeeds. Reset requires FAULT, every safety input safe, and no stuck injection.
Reset clears closure confirmation and returns to IDLE. Repeated `begin()` calls
cannot clear a lockout. A software stop (`s`) remains latched until reset; an
active external stop input must also be cleared at its source before reset.

| Module interface | Purpose |
| --- | --- |
| `setSafetyConditions(still, vertical, batteryOK, imuOK, stopActive, externalFault)` | Supply readings and immediately check them; last two arguments default to false |
| `update()` | Check safety and non-blocking timers on every loop |
| `close()` / `requestRelease()` | Return whether the operation was accepted |
| `stop()` / `inhibitRelease()` | Cancel operation and latch emergency stop |
| `resetLockout()` | Explicit validated fault acknowledgement |
| `status()` / `statusName()` / `fault()` | Read state and first fault code |
| `isClosed()` / `isComplete()` | Read simulated closure and operation completion |
| `message` / `faultReason` | Last refusal reason / retained first fault reason |
| `simulateStuck(bool)` / `simulateLostClosure()` | Automated timeout and feedback-failure injection |

Initial sensor values are false, so missing readings prevent release and reset.
Supply fresh readings before commands and every update in any future integration.
The setter checks safety immediately, so a later healthy reading cannot hide a
failure. Prefer the setter over assigning input variables directly.

## Tests

From the repository root:

```sh
"Arduino Code/D4_Latch_Control/tests/run_tests.sh"
```

The full suite compiles the sketch with a host C++11 compiler, warnings treated
as errors, and deterministic simulated time. It tests each safety input before
and during release and during reset, lost closure, successful repeated cycles,
stop lockouts, first-fault retention, stuck closing/release, exact timeout and
completion boundaries, long polling gaps, millis rollover, Serial reporting,
and rejection of every removed command. Sensor and fault injection remain in
automated tests; they do not need Serial commands. Temporary binaries are removed.
The host Arduino shim is not a board or hardware validation.

## Integration limits

D6 has its own incompatible `Latch` namespace; do not include both definitions.
Future integration must replace that simulated implementation, supply actual
battery/IMU/stop/fault readings, handle rejected requests and faults, and retain
D6 supervisory checks. No other team files are changed here.

This simulation has no sensor freshness timeout, physical closure feedback,
debouncing, persistent lockout across power loss, or hardware validation. Healthy
readings remain healthy until replaced. Real hardware requires qualified sensor
feedback, data validity checks, and independent hardware interlocks and stops.
