# Rover Controller: C Module Exercises

This project turns the autonomous controller into small C modules. The aim is
to follow how data moves through a program and how decisions are made. The
serial and terminal code is provided as a boundary to the operating system;
students do not need to study every hardware detail to understand the rover's
logic.

The original single-file C version remains in the parent directory as
`rover_controlA.c`. The earlier C++ reference is `rover_controlA.cpp`.

## Requirements

- A C compiler such as `clang` or `gcc`
- `make`
- macOS or Linux for the POSIX serial and terminal modules
- A rover and serial device only when running the finished program on hardware

The logic tests and full build do not need a connected rover. Build commands
run from this directory:

```sh
make test
make
```

The finished executable is `build/rover_controlA`. The default serial device is
`/dev/ttyUSB0`; pass another device path as the first argument when needed.
For example, macOS serial devices commonly use `/dev/cu.*` names:

```sh
./build/rover_controlA /dev/cu.usbserial-0001
```

## Module Map

- `rover.h`: shared data types and named values
- `rover_logic.c`: keyboard mapping, PWM math, scan sections, and clearance choice
- `navigation.c`: parse and validate the camera's navigation data
- `serial.c`: send commands and PWM values to the rover
- `terminal.c`: read one key at a time and restore terminal settings
- `time_util.c`: small clock and delay helpers
- `main.c`: connect the modules and run the manual/autonomous state machine
- `test_rover.c`: tests that run without rover hardware

Each `.c` file has a matching header for its public functions. The header tells
other modules what they can call; the `.c` file contains how that work is done.

## Suggested Homework Sequence

### HW 1: Read the shared data model

Open `rover.h`. Find the navigation measurements, the two operating modes, and
the autonomous states. Draw arrows showing which state can follow each other
in `main.c`.

Tasks:

1. Write down what information one `NavData` value carries.
2. Find every place `enum AutoState` is changed in `main.c`.
3. Build the complete project with `make`.

Checkpoint: the project builds without warnings. At this stage, focus on how
the files are compiled together, not on serial hardware.

### HW 2: Practice pure decision logic

Read `rover_logic.h`, `rover_logic.c`, and the matching tests in
`test_rover.c`. These functions take values and return decisions; they do not
open files or contact hardware.

Tasks:

1. Trace how `rover_manual_command` converts a key into an ESP32 command.
2. Work out the expected left and right PWM values for a few throttle/steer
   pairs before running the tests.
3. Add a test for an input key that should not move the rover.
4. Add a test for a scan clearance that is lower than the current best.

Checkpoint: `make test` prints `All rover logic and navigation tests passed.`

### HW 3: Parse camera navigation data

The camera process writes one whitespace-separated line containing a timestamp,
three distances, throttle, steering, and a status word. `navigation.c` converts
that line into a `NavData` structure and checks whether its timestamp is recent.

Tasks:

1. Match each conversion in `navigation_parse_line` to a field in `NavData`.
2. Add a test for a line with one missing value.
3. Add a test for a valid line with status `STOP` and check
   `rover_navigation_blocked`.
4. Explain why stale camera data should not be treated as a clear path.

Checkpoint: `make test` passes with the new parser tests.

### HW 4: Understand the hardware boundary

Read the function declarations in `serial.h` and `terminal.h` before opening
their implementations. Treat these modules as tools the control logic can use:
open a serial device, send a movement/speed value, or read a keyboard key.

Tasks:

1. Follow `serial_set_motor_speed` and note how an integer becomes a line of
   text for the ESP32.
2. Find where `terminal_start` is paired with `terminal_restore`.
3. Build and run the tests without a rover attached.
4. Do not run the controller against hardware until the movement mappings and
   emergency stop behavior have been reviewed with an instructor.

Checkpoint: `make clean && make test && make` completes successfully offline.

### HW 5: Follow the controller state machine

Read `main.c` from `main` downward. The top-level loop reads a key, applies the
manual dead-man stop, and then updates autonomous behavior when that mode is
active.

Tasks:

1. Trace the normal autonomous path: `AUTO_DRIVE` to `AUTO_STOPPED`, then
   `AUTO_SCAN`, then either `AUTO_REVERSE` or `AUTO_TURN_TO_BEST`.
2. For each state, list the data it reads, the condition it checks, and the
   next state it selects.
3. Find the places that send `S` (stop) when data is missing or a timed action
   finishes.
4. Add one harmless status message for a state transition and verify the
   program still builds.

Checkpoint: describe the state transitions without needing to explain the
POSIX serial configuration.

## Control Flow Summary

1. In manual mode, a key maps to a movement command and refreshes the dead-man
   timer. If key input stops, the rover receives a stop command after the
   timeout.
2. In autonomous drive, fresh navigation data supplies throttle and steering.
   Missing or stale data causes a stop; blocked data begins the recovery flow.
3. During a scan, the controller remembers the direction with the best center
   clearance. It turns toward that direction or briefly reverses if no
   clearance meets the threshold.

## Useful Commands

```sh
make test       # run offline logic/parser tests
make            # compile the full controller
make clean      # remove generated build files
```

The executable controls real motors. Build and test freely, but only run it
with hardware after the rover is secured and an instructor has checked the
serial device and stop behavior.
