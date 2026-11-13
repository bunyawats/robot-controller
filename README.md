# robot-controller

ESP32-S3 firmware (Rust, `esp-idf-svc` std stack) that drives a 16-servo biped through a
PCA9685 servo driver, controlled over MQTT: single servos, several servos at once, the RGB
status LED, and named actions (sway, walk, stand up, dance, stand by, rainbow, ...). See
[MQTT control](#mqtt-control).

## Hardware

Photos: [the robot](16%20DOF%20Robot.jpg) (4 servos per arm, 4 per leg),
[the kit manual's servo numbering](Robot%20Schema.png),
[the ESP32-S3 board on its own](ESP32-Board.jpg),
[the ESP32-S3 on its extension board](ESP32-Extension%20Board.jpg) and
[the PCA9685 board](PCA9685.jpg).

The board is an ESP32-S3-N16R8 module (16 MB flash, 8 MB octal PSRAM) on a DevKitC-style
board with two USB-C ports, plugged into an "ESP32 Extension Board" (DC 6.5-9 V barrel jack,
5V and 3V3 headers, and an S / 3.3V / GND pin row per GPIO). Flash and monitor through the
USB-C port wired to the onboard WCH USB-serial chip; on this Mac it shows up as
`/dev/cu.usbmodem5CF70246821`. The other USB-C port is the S3's native USB
(`/dev/cu.usbmodem31xxx`, the number changes): it reconnects on every reset, and resets
through it often leave the chip waiting for a download, so reflash instead of resetting
if you use that port.

The square white part on the ESP32-S3 board is a WS2812 RGB LED on GPIO48. The firmware
uses it as a status light:

| Colour | Meaning |
|---|---|
| Blue | Boot countdown (hands off the robot), or not connected to the MQTT broker |
| Green | Idle, ready for a command |
| Yellow | Busy running a robot command (other robot commands are rejected) |
| Dim blue | Stand-by (after `stand_by`) |
| Red | PCA9685 not answering at boot, or a servo write failed (stays red until reset) |

If the PCA9685 doesn't answer at boot, the firmware still connects to MQTT: the state is
`pca9685_not_found`, robot commands are rejected with `rejected_no_servos`, and LED commands
still work (so the LED and MQTT can be tested without the servo board). Fix the wiring and
reset to use the servos.

A colour set over MQTT (`<prefix>/led`) replaces the status colours until `auto` is sent;
red for a fault always wins. If the LED stays dark, look for an `RGB` solder pad that needs
bridging; some boards of this kind put the LED on GPIO38 instead.

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
| Left header `V+` | `+` of a separate 5 V servo supply |
| Green screw terminal `GND` (or left header `GND`) | `-` of the servo supply |

- Feed servo power into the `V+` header pin, not the green screw terminal: on this board the
  terminal's reverse-polarity protection transistor does not conduct, so no power reaches
  the servos from it. The header path has no reverse-polarity protection, so check the
  polarity before switching on. See [WIRING.md](WIRING.md#servo-power) for details.
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
  1000 uF, add a 1000 uF one across `V+` and `GND` to prevent brownout resets.
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

The joint names and channels in `src/robot_16_dof/servo.rs` come from the kit manual, but which side is
"right" and which way each servo turns are a **guess**. Check them first:

1. Hang the robot or lay it on its back so no joint can drive into the table.
2. Flash, wait for the 5 second countdown (every joint then snaps to neutral) and for the
   LED to turn green.
3. Send `self_test` to `robot/cmd`: each joint moves +15 degrees and back, one at a time.
   The log prints the joint name before each move.
4. Write down which physical servo moved for each name, and which way it went.
   To check servos one at a time, send
   `{"cmd":"joint_test","joints":["s4"]}` (+30 degrees, its positive direction, and back
   three times; add `"deg":-30` for the other way) or
   `{"cmd":"full_range","joints":["s4"],"times":1}`.
5. Edit `CALIBRATION` (and the `Joint` enum order or names if needed) in `src/robot_16_dof/servo.rs`:
   - `channel`: the PCA9685 channel that servo is really plugged into
   - `invert`: set `true` if the joint moved the wrong way (positive angles are documented
     on the `Joint` enum)
   - `trim_deg`: nudge so neutral (90) means the joint is truly centered
   - `min_deg` / `max_deg`: widen only after checking the joint cannot bind
6. Repeat until neutral looks like standing straight with arms down.
7. Then try `arms`, `sway`, `dance` and, holding the robot, `walk`.

## MQTT control

Set the WiFi and broker in `cfg.toml` (see [Build and flash](#build-and-flash)). Topics are
under `topic_prefix` (default `robot`):

| Topic | Payload | Effect |
|---|---|---|
| `robot/servo/<joint>` | `120` or `{"angle":120,"ms":500}` | Moves one servo |
| `robot/servos` | `{"r_hip_roll":100,"l_hip_roll":80,"ms":600}` | Moves several servos, two at a time in the given order |
| `robot/led` | `off`, `auto`, `rainbow`, `red`/`green`/`blue`/`yellow`/`cyan`/`magenta`/`orange`/`white`, `#rrggbb` or `{"r":0,"g":40,"b":0}` | Sets the LED (applied at once, even while busy) |
| `robot/cmd` | an action word, or `{"cmd":"walk","steps":6}` | Runs an action (below) |
| `robot/status` | published by the robot | `{"state":"busy","action":"walk","reason":"started","error":null}` |
| `robot/pose` | published by the robot | `{"r_ankle_roll":90.0,"r_knee_lower":75.5,...}`: every joint's angle |

- **Joints** are named `r_ankle_roll`, `r_knee_lower`, `r_knee_upper`, `r_hip_roll`,
  `r_shoulder_pitch`, `r_shoulder_roll`, `r_elbow`, `r_gripper` and the same with `l_`, or
  by the kit's servo number (`s1`-`s8`, `s25`-`s32`, or just `4`).
- **Angles** are logical degrees: 90 is neutral, and each joint's sign convention is on the
  `Joint` enum in `src/robot_16_dof/servo.rs`. Angles beyond a joint's calibrated limits are
  clamped. `ms` (100-10000, default 500) is the time for each move; `robot/servos` moves
  two servos at a time (the servo supply can't start more at once), so the whole move
  takes `ms` for every pair.
- **Named colours** use brightness 64 of 255; `#rrggbb` and `{"r","g","b"}` are sent as
  given (the LED is very bright at 255).

Actions on `robot/cmd` (words are case-insensitive; `stand_by`, `standby` and `stand-by`
are the same):

| Action | Parameters | What it does |
|---|---|---|
| `sway` | `times` (3) | Hips and ankles lean left and right together |
| `sway_grip` | `times` (3) | Hips sway, claws open on the left and close on the right |
| `sway_arms` | `times` (3) | Like `sway_grip`, raising and lowering the arms too |
| `walk` | `steps` (4, max 20) | Lean-and-shuffle walk (knees not verified yet: hold the robot) |
| `stand_up` | | Every joint back to neutral, two at a time, legs first |
| `dance` | `times` (3) | Lean, raise an arm, snap the claws, each side, then arms out |
| `stand_by` | | Same neutral stance reached slowly; LED dim blue; servos keep holding |
| `arms` | `arm` (`both`/`left`/`right`), `times` (3) | Raise and lower the arms |
| `release` | | Cut the servo pulses: the robot goes limp and may fall |
| `rainbow` | | LED rainbow fade (same as `rainbow` on `robot/led`) |
| `stop` | | Stop the running action where it is (within 20 ms) |
| `self_test` | | Each joint +15 degrees and back, one at a time |
| `joint_test` | `joints` (required), `deg` (30, max +-60), `times` (3) | Swing the listed joints together |
| `full_range` | `joints` (required), `times` (3) | Each listed joint through its calibrated range |

`times` is 1-10. Only one robot command runs at a time: while one is running (LED yellow),
other robot commands are rejected with `"reason":"rejected_busy"`; LED commands and `stop`
still work. Invalid messages get `"reason":"rejected_invalid"` with the problem in `error`.
Every action publishes `started` and then `done` or `stopped`, and a `heartbeat` goes out
every 30 s.

`robot/pose` carries all 16 joints (logical degrees to 0.1, clamped to the calibrated
limits, in `Joint` order). It is sent at most once (QoS 0) every 100 ms while the robot
moves, then once with the final pose when each command ends, and once when the broker
connects. Nothing is sent while the robot stands still, and nothing without the PCA9685.

After `release`, the next move re-energizes every servo at once from the last commanded
pose; send `stand_up` first.

Examples:

```bash
B=mqtt-broker.local   # your broker host
mosquitto_sub -h $B -t 'robot/status' -v &
mosquitto_pub -h $B -t robot/servo/s5 -m 150
mosquitto_pub -h $B -t robot/servos -m '{"s5":150,"s28":150,"ms":800}'
mosquitto_pub -h $B -t robot/led -m '#200020'
mosquitto_pub -h $B -t robot/cmd -m dance
mosquitto_pub -h $B -t robot/cmd -m '{"cmd":"walk","steps":2}'
mosquitto_pub -h $B -t robot/cmd -m stop
```

## Build and flash

One-time host setup (same as `esp32-mqtt-blink`):

```bash
cargo install espup
espup install
espup install --toolchain-version 1.98.1.0 --name esp-1.98 --targets esp32,esp32s3
. $HOME/export-esp.sh
cargo install espflash ldproxy
```

Copy the config template and fill in your WiFi and broker. `cfg.toml` is gitignored and
compiled into the firmware, so any change needs a rebuild and reflash. Use the broker
host's `.local` mDNS hostname rather than its IP, so a DHCP change on the broker host
doesn't break the connection:

```bash
cp cfg.toml.example cfg.toml
```

Then:

```bash
. $HOME/export-esp.sh
cargo build --release
espflash flash --monitor -p /dev/cu.usbmodem5CF70246821 target/xtensa-esp32s3-espidf/release/robot-controller
```

After boot the servos hold neutral until a command arrives. Commands sent while the robot is
offline are lost.

## Code layout

| File | What it does | Hardware needed to test |
|---|---|---|
| `src/robot_16_dof/servo.rs` | Joint list, calibration table, angle to PWM tick conversion | none |
| `src/robot_16_dof/pca9685.rs` | Minimal PCA9685 driver over a small `Transport` trait | none |
| `src/robot_16_dof/motion.rs` | Poses, smooth interpolation between them, stopping a move | none |
| `src/robot_16_dof/moves.rs` | Self-test, joint test, servo moves, sway, stand up/by, dance, arms, walk | none |
| `src/robot_16_dof/esp32.rs` | PCA9685 over the ESP32-S3's I2C, the real servo `Hardware` | ESP32-S3 |
| `src/rgb_led/color.rs` | RGB colours and the rainbow fade | none |
| `src/rgb_led/indicator.rs` | Which colour the status LED shows | none |
| `src/rgb_led/ws2812.rs` | WS2812 LED driver over the ESP32-S3's RMT peripheral | ESP32-S3 |
| `src/remote/json.rs` | Small JSON reader for payloads | none |
| `src/remote/command.rs` | MQTT topic and payload to `Command` | none |
| `src/remote/status.rs` | Status JSON | none |
| `src/remote/pose.rs` | Pose JSON and its 10 Hz throttle | none |
| `src/remote/mqtt.rs` | WiFi and MQTT threads | ESP32-S3 |
| `src/main.rs` | Boot, command loop, busy/stop handling, LED thread | ESP32-S3 |

The modules marked "none" have no ESP dependencies and carry unit tests. To run them on a PC,
include `src/robot_16_dof/mod.rs`, `src/rgb_led/mod.rs` and `src/remote/mod.rs` from a small
test crate with `#[path = "..."] mod robot_16_dof;`, `mod rgb_led;` and `mod remote;`, then
`rustc --test` (the ESP-only
files are skipped off-target). They are not run by `cargo test` here because the project
builds only for the ESP32-S3 target.

## Tuning the walk

`DEFAULT_GAIT` in `src/robot_16_dof/moves.rs`:

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

- Only some joints are verified on hardware so far (see CLAUDE.md).
- Every pose is clamped to the joint limits in `CALIBRATION`, and the unit tests check that
  the default walk stays inside them.
