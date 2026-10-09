# robot-controller

ESP32-S3 firmware (Rust, `esp-idf-svc` std stack) that drives a 16-servo biped through a
PCA9685 servo driver. Moves are hardcoded: a joint self-test, arm up/down, and a 10-step
walk. There is no networking and no remote control.

## Hardware

Photos: [the robot](16%20DOF%20Robot.jpg) (4 servos per arm, 4 per leg),
[the kit manual's servo numbering](Robot%20Schema.png),
[the ESP32-S3 on its extension board](ESP32-Extension%20Board.jpg) and
[the PCA9685 board](PCA9685.jpg).

The board is an ESP32-S3-N16R8 module (16 MB flash, 8 MB octal PSRAM) on a DevKitC-style
board with two USB-C ports, plugged into an "ESP32 Extension Board" (DC 6.5-9 V barrel jack,
5V and 3V3 headers, and an S / 3.3V / GND pin row per GPIO). Flash and monitor through the
USB-C port wired to the onboard WCH USB-serial chip; on this Mac it shows up as
`/dev/cu.usbmodem5CF70246821`.

[WIRING.md](WIRING.md) has the full ESP32-S3 to PCA9685 wiring guide with a diagram and a
pre-power-on checklist.

The PCA9685 is the common 16-channel servo board. Its address pads A0-A5 are all open
(factory default), which gives address 0x40, the one the firmware uses.

| PCA9685 | Connect to |
|---|---|
| Left header `VCC` | 3.3V pin on the extension board (middle red pin of any GPIO row) |
| Left header `GND` | GND pin of any GPIO row |
| Left header `SDA` | S pin of the IO8 row (GPIO8) |
| Left header `SCL` | S pin of the IO9 row (GPIO9) |
| Left header `OE` | nothing (pulled low on the board, so the outputs stay on) |
| Left header `V+` | nothing |
| Green screw terminal `+` / `GND` | separate 5 V servo supply |

- Feed servo power through the green screw terminal, not the `V+` header pin: on this board
  design the terminal usually goes through a reverse-polarity protection part, and the
  header pin bypasses it. Check the `+` / `GND` markings before tightening.
- The unpopulated pins on the right edge duplicate the left header for chaining a second
  board. Leave them empty.

- Use **3V3** for `VCC`, not 5 V: the PCA9685 board's I2C pull-ups follow `VCC`, and 5 V
  on the ESP32-S3's GPIOs can damage it.
- Power the servos from their own 5 V supply rated **3 A or more** (16 SG90/MG90S servos
  moving together peak around 3-4 A). Never from the ESP32-S3 or its USB port, and not from
  the extension board's 5V header either, even when it is fed from the DC jack: its small
  onboard regulator is unlikely to handle that peak, and the dips will reset the ESP32-S3.
- Join the grounds of the servo supply, the PCA9685 and the ESP32-S3.
- The PCA9685 board already has a large capacitor on the servo supply. If it is under
  1000 uF, add a 1000 uF one across the screw terminal to prevent brownout resets.
- Plug the servos in by the manual's numbers (see [the schematic](Robot%20Schema.png)):
  S1-S8 on channels 0-7 and S25-S32 on channels 8-15.

  | Channel | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
  |---|---|---|---|---|---|---|---|---|
  | Servo | S1 | S2 | S3 | S4 | S5 | S6 | S7 | S8 |
  | Joint (right) | ankle roll | knee lower | knee upper | hip roll | shoulder pitch | shoulder roll | elbow | gripper |

  | Channel | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 |
  |---|---|---|---|---|---|---|---|---|
  | Servo | S25 | S26 | S27 | S28 | S29 | S30 | S31 | S32 |
  | Joint (left) | gripper | elbow | shoulder roll | shoulder pitch | hip roll | knee upper | knee lower | ankle roll |
- Each channel column: yellow = signal, red = V+, black = GND. With
  orange / red / brown servo leads, orange goes on yellow and brown on black. Channels run
  0-15 from left to right, in four groups of four.
- Don't use GPIO35, 36 or 37 for anything you add later: on N16R8 modules the octal PSRAM
  uses them internally, even though the extension board breaks them out to headers.

## First run (do this before anything else)

The joint names and channels in `src/servo.rs` come from the kit manual, but which side is
"right" and which way each servo turns are a **guess**. Check them first:

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
- `stride_deg`: total knee-pair swing front to back. The two knee servos turn by equal and
  opposite amounts so the foot stays flat, which moves the foot forward or back.
- `lift_deg`: extra outward hip roll on the swinging leg so its foot clears the ground
- `phase_ms`: time for each of the five phases of a step

The legs have no hip or ankle pitch, so the walk is a sideways-lean shuffle rather than a
real stride. Start small, raise one value at a time, and keep a hand near the robot. The walk is a
simple open-loop gait with no balance feedback, so it will need tuning on the real robot
before it walks reliably; it is a starting point, not a finished gait.

## Notes

- This project has not been run on hardware yet.
- Every pose is clamped to the joint limits in `CALIBRATION`, and the unit tests check that
  the default walk stays inside them.
