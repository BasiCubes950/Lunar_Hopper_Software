# STAC D6 simulation

An Arduino simulation of the lunar hopper's D6 controller. It cycles through eleven operational states and latches emergency stops and faults until a manual restart. All subsystem operations are simulated in memory; there are no motor or latch-release outputs.

This standalone sketch stays separate from other team sketches so their `setup()` and `loop()` functions are not compiled together. The folder and primary sketch must both be named `D6_Main_Controller`. Real D1–D7 integration is not complete. Do not connect motors, actuators, or latch-release hardware.

## Run in Wokwi

Create an Arduino Uno project in Wokwi and replace its default sketch with the contents of `D6_Main_Controller.ino` (Wokwi may call its editor tab `sketch.ino`). Add the header and replace the diagram using `D6_Main_Controller.ino`, `SimulatedSubsystems.h`, and the supplied `diagram.json`. Put the header alongside the sketch in the Wokwi project. Open the serial monitor at 115200 baud. The stop button connects pin 2 to GND using the Arduino's input pull-up.

Each simulated operation normally takes 800 ms. Every operational state has a 3000 ms timeout. No operation uses `delay()`.

| Serial command | Effect |
| --- | --- |
| `s` | Emergency stop |
| `r` | Restart from SETTLE, only from STOP or FAULT with button released and healthy battery/IMU |
| `t` / `n` | Hold / resume simulated operation completion |
| `b` / `g` | Fail / restore simulated battery |
| `i` / `k` | Fail / restore simulated IMU |
| `u` / `o` | Tilt / restore simulated upright pose |

Restart also resets the simulated pose, opens the simulated latch, and clears the operation hold. Restoring a sensor alone does not clear STOP or FAULT.

## How the code works

`D6_Main_Controller.ino` owns the state machine and safety decisions. `changeState()` calls `startStateOperation()` once on entry. Each pass through `loop()` checks safety and timeouts, calls subsystem `update()` functions, then polls their completion status.

`SimulatedSubsystems.h` contains Legs, Winch, Latch, IMU, and Battery modules. Each has `begin()` and `update()`, plus commands and status functions. A small shared timer type lets each operation finish without blocking. The modules update their own simulated sensor confirmations; D6 does not force completion by changing sensor flags.

| State | Entry operation | Completion polled |
| --- | --- | --- |
| SETTLE | IMU settle | IMU complete |
| WIND | Winch wind | Winch complete |
| LATCH | Latch close | Latch complete and closed |
| BACK_OFF | Winch back off | Winch complete |
| FOLD_LEGS | Legs fold | Legs complete |
| READY | IMU readiness observation | IMU complete, then release interlocks |
| RELEASE | Latch simulated release | Latch complete; interlocks checked before every update |
| FLYING | IMU simulated flight | IMU complete |
| LANDED | IMU simulated landing | IMU complete and stationary |
| STAND_UP | Legs stand | Legs complete and upright confirmation |
| GET_READY_AGAIN | Latch reset | Latch complete, then return to SETTLE |

Release requires a healthy battery and IMU, stationary and upright pose, and a closed latch. Stop/fault cancels all active simulated operations and inhibits release. Timeouts take priority over completion. These are simulation rules, not a real hardware safety system.

## Tests

With a C++ compiler installed, run from `Arduino Code/D6_Main_Controller/`:

```sh
c++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/state_machine_test.cpp -o /tmp/d6-state-machine-tests
/tmp/d6-state-machine-tests
```

The tests compile the actual sketch with a tiny simulated Arduino interface. They cover all eleven states, frequent polling without restarting operations, repeated cycles, serial/button stops in every state, timeouts in every state, release interlock failures, restart restrictions, sensor commands, and clock rollover. They advance a fake clock rather than waiting in real time. Host tests do not replace a Wokwi/Arduino build check.

## Arduino compile check

With Arduino CLI and the Arduino AVR core installed, run from the team repository root:

```sh
arduino-cli compile --fqbn arduino:avr:uno "Arduino Code/D6_Main_Controller"
```

Host tests do not verify Arduino preprocessing or AVR compilation. An interactive Wokwi smoke test is still required.
