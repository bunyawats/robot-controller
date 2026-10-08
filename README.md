# robot-controller

ESP32-S3 firmware (Rust, `esp-idf-svc` std stack) that drives a 16-servo biped through a
PCA9685 servo driver. Moves are hardcoded: a joint self-test, arm up/down, and a 10-step
walk. There is no networking and no remote control.

## Hardware

Photos: [the robot](16%20DOF%20Robot.jpg) (3 servos per arm, 5 per leg) and
[the ESP32-S3 on its extension board](ESP32-Extension%20Board.jpg).

The board is an ESP32-S3-N16R8 module (16 MB flash, 8 MB octal PSRAM) on a DevKitC-style
board with two USB-C ports, plugged into an "ESP32 Extension Board" (DC 6.5-9 V barrel jack,
5V and 3V3 headers, and an S / 3.3V / GND pin row per GPIO). Flash and monitor through the
USB-C port wired to the onboard WCH USB-serial chip; on this Mac it shows up as
`/dev/cu.usbmodem5CF70246821`.

| PCA9685 | Extension board |
|---|---|
| VCC | 3.3V pin (middle red pin of any GPIO row) |
| GND | GND pin of any GPIO row |
| SDA | S pin of the IO8 row (GPIO8) |
| SCL | S pin of the IO9 row (GPIO9) |
| V+ | separate 5 V servo supply |

- Use **3V3** for `VCC`, not 5 V: the PCA9685 board's I2C pull-ups follow `VCC`, and 5 V
  on the ESP32-S3's GPIOs can damage it.
- Power the servos from their own 5 V supply rated **3 A or more** (16 SG90/MG90S servos
  moving together peak around 3-4 A). Never from the ESP32-S3 or its USB port, and not from
  the extension board's 5V header either, even when it is fed from the DC jack: its small
  onboard regulator is unlikely to handle that peak, and the dips will reset the ESP32-S3.
- Join the grounds of the servo supply, the PCA9685 and the ESP32-S3.
- Add a 1000 uF capacitor across the servo supply near the PCA9685 to prevent brownout resets.
- Plug each servo's 3-pin lead into a PCA9685 channel header (signal, V+, GND order as
  printed on the board).
- Don't use GPIO35, 36 or 37 for anything you add later: on N16R8 modules the octal PSRAM
  uses them internally, even though the extension board breaks them out to headers.

## First run (do this before anything else)

The joint names and channel numbers in `src/servo.rs` are a **guess**. Fix them first:

1. Hang the robot or lay it on its back so no joint can drive into the table.
2. Flash with `DEMO = Demo::SelfTest` (the default in `src/main.rs`).
3. After a 5 second countdown every joint snaps to neutral, then each one moves +15 degrees
   and back, one at a time. The log prints the joint name before each move.
4. Write down which physical servo moved for each name, and which way it went.
5. Edit `CALIBRATION` (and the `Joint` enum order or names if needed) in `src/servo.rs`:
   - `channel`: the PCA9685 channel that servo is really plugged into
   - `invert`: set `true` if the joint moved the wrong way (positive angles are documented
     on the `Joint` enum)
   - `trim_deg`: nudge so neutral (90) means the joint is truly centered
   - `min_deg` / `max_deg`: widen only after checking the joint cannot bind
6. Repeat until neutral looks like standing straight with arms down.
7. Then set `DEMO` to `Demo::Arms`, `Demo::Walk` or `Demo::All`.

## Build and flash

One-time host setup (same as `esp32-mqtt-blink`):

```bash
cargo install espup
espup install
espup install --toolchain-version 1.98.1.0 --name esp-1.98 --targets esp32,esp32s3
. $HOME/export-esp.sh
cargo install espflash ldproxy
```

Then:

```bash
. $HOME/export-esp.sh
cargo build --release
espflash flash --monitor -p /dev/cu.usbmodem5CF70246821 target/xtensa-esp32s3-espidf/release/robot-controller
```

Press reset to run the demo again. After the demo the servos keep holding the final pose;
set `RELEASE_AT_END` in `src/main.rs` to cut servo power instead (the robot then goes limp).

## Code layout

| File | What it does | Hardware needed to test |
|---|---|---|
| `src/servo.rs` | Joint list, calibration table, angle to PWM tick conversion | none |
| `src/pca9685.rs` | Minimal PCA9685 driver over a small `Transport` trait | none |
| `src/motion.rs` | Poses, smooth interpolation between them | none |
| `src/moves.rs` | Self-test, arm up/down, walk | none |
| `src/main.rs` | ESP32-S3 wiring (I2C, boot countdown, demo selection) | ESP32-S3 |

The first four modules have no ESP dependencies and carry unit tests. To run them on a PC,
include them from a small test crate with `#[path = "..."] mod servo;` and so on, then
`rustc --test`. They are not run by `cargo test` here because the project builds only for
the ESP32-S3 target.

## Tuning the walk

`DEFAULT_GAIT` in `src/moves.rs`:

- `lean_deg`: sideways lean over the support foot while the other foot is lifted
- `stride_deg`: total hip swing front to back
- `knee_bend_deg`: how high the swinging foot is lifted
- `phase_ms`: time for each of the five phases of a step

Start small, raise one value at a time, and keep a hand near the robot. The walk is a
simple open-loop gait with no balance feedback, so it will need tuning on the real robot
before it walks reliably; it is a starting point, not a finished gait.

## Notes

- This project has not been run on hardware yet.
- Every pose is clamped to the joint limits in `CALIBRATION`, and the unit tests check that
  the default walk stays inside them.
