# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ESP32-S3 firmware (Rust, `esp-idf-svc` std stack) for a 16-servo biped driven through a PCA9685 over I2C (SDA GPIO8, SCL GPIO9, addr 0x40). Controlled over MQTT (single servo, several servos, the RGB LED, named actions); the topic contract is in README.md ("MQTT control") and `src/remote/command.rs`. The WiFi/MQTT threading is copied from the sibling `esp32-mqtt-relay` project (`~/Projects/Rust/esp32_mqtt_4_relay`). Hardware bring-up is in progress. See README.md for wiring, power, and the first-run calibration procedure.

The phone controller (Bevy/wasm canvas UI + Axum BFF bridging WebSocket to this firmware's MQTT topics) lives in a separate repo, `~/Projects/Rust/robot-console` (GitHub `bunyawats/robot-console`), together with its design spec and implementation plan (`docs/`). The BFF depends on this firmware's MQTT contract: when adding or changing topics, actions or status fields (e.g. `robot/pose`, or the planned walk backward, `turn`, `spread_arms` and an `led` field in status), update robot-console's `bff/src/mapping.rs` and its docs too.

## Build and flash

The toolchain is pinned to `esp-1.98` in `rust-toolchain.toml` (the 1.99.0.0 esp release fails to build std for espidf with "`AT_FDCWD` not found in `libc`"). Don't bump it casually.

`cfg.toml` (gitignored, copied from `cfg.toml.example`) holds WiFi and broker settings and is read at compile time by `toml-cfg`; its section name `[robot-controller]` must match the crate name. Without it the build still succeeds, but the firmware stops at boot with `wifi_ssid is empty`.

```bash
. $HOME/export-esp.sh
cargo build --release
espflash flash --monitor -p /dev/cu.usbmodem5CF70246821 target/xtensa-esp32s3-espidf/release/robot-controller
```

## Tests

`cargo test` does **not** work: the crate only builds for the Xtensa ESP32-S3 target (set in `.cargo/config.toml`). The hardware-independent modules (`robot_16_dof::{servo, pca9685, motion, moves}`, `rgb_led::{color, indicator}` and `remote::{json, command, status, pose}`) carry `#[cfg(test)]` unit tests that run on the host by compiling them into a throwaway test crate with the stable toolchain. Their ESP-only siblings (`robot_16_dof::esp32`, `rgb_led::ws2812`, `remote::mqtt`) are behind `#[cfg(target_os = "espidf")]`, so the three `mod.rs` files can be included whole. Because the test crate is plain `rustc` with no Cargo, these modules can't use external crates; that's why `remote::json` is hand-written instead of serde_json. Keep that crate outside the repo (e.g. a temp dir):

```bash
P=$PWD/src
cat > host_tests.rs <<EOF
#[path = "$P/robot_16_dof/mod.rs"] mod robot_16_dof;
#[path = "$P/rgb_led/mod.rs"] mod rgb_led;
#[path = "$P/remote/mod.rs"] mod remote;
EOF
rustc +stable --edition 2021 --test host_tests.rs -o host_tests && ./host_tests
./host_tests walk_stays_inside   # run a single test by name filter
```

All three modules must be declared at the test crate root under these names, because the modules reference each other via `crate::robot_16_dof::servo` etc.

## Architecture

Three top-level modules plus `main.rs`. In each module, everything except the one ESP-only file is pure logic with no ESP dependencies.

**`robot_16_dof/`** — the 16-servo biped:

- **`servo.rs`** — the only module that knows about physical servos. Defines the `Joint` enum (order must match `CALIBRATION`), the per-joint `Calib` table (PCA channel, min/max limits, trim, invert), and `angle_to_ticks` / `pose_to_ticks`. Everywhere else works in **logical degrees** (90 = neutral, with sign conventions documented on `Joint`); limit clamping, inversion and trim are applied only here. `pose_to_ticks` remaps from joint index to PCA channel index.
- **`pca9685.rs`** — minimal driver generic over a tiny `Transport` trait (`write` + `delay_ms`) rather than an I2C crate, so it can be tested with a mock.
- **`motion.rs`** — `Pose` (array of logical angles indexed by `Joint::index()`; `with` sets an offset from neutral, `add` adds a delta) and `Robot<H: Hardware>`, which tracks the current pose and interpolates to targets with smoothstep at `STEP_MS` intervals. `Hardware` trait = `write_pose` + `delay_ms` + `should_abort` (default false). Once `should_abort` is true the robot holds its last frame and every later `move_to`/`pause` returns at once until `clear_abort`, so any multi-move routine ends within one step without needing its own checks. `test_support::MockHw` records poses/elapsed time and can request an abort after N writes.
- **`moves.rs`** — self-test, joint test (`swing_deg` sets the direction), `full_range_test` (min to max calibrated limit), `move_joints` (absolute angles, clamped, `MAX_SERVOS_AT_ONCE` = 2 servos per move), `stand_up` / `stand_by` (to neutral in `STAND_ORDER` pairs; stand_by is slower), `dance`, `sway` / `sway_and_grip` / `sway_arms_and_grip` (hips left-right; the latter two move one servo pair at a time), arm up/down, and an open-loop 5-phase walk parameterized by `Gait` (`DEFAULT_GAIT`). Each leg is hip roll, two stacked knee pitch servos and ankle roll (no hip or ankle pitch), so the walk is a lean-and-shuffle gait. `frame()` keeps the feet flat by turning the knee pair by equal and opposite amounts (`lower = -upper`) and setting ankle roll to `-hip roll`. A test asserts the default walk stays within every joint's calibrated limits — keep it passing when tuning the gait or calibration.
- **`esp32.rs`** (ESP only) — `I2cBus` (`Transport` over the ESP-IDF I2C driver, 100 kHz, one retry per write), `connect_pca9685` (opens I2C, logs a bus scan, inits the PCA9685 at 50 Hz), and `Servos<F: FaultSignal, P: PoseSink>` (`Hardware` via `servo::pose_to_ticks` with `CALIBRATION`; passes every frame to `P::frame`; calls `F::fault` on the first failed write; `should_abort` reads a shared `AtomicBool` set by the MQTT `stop` command). The robot module doesn't know about the LED; `FaultSignal` is how `main.rs` connects the two.

**`rgb_led/`** — the board's 3-colour LED:

- **`color.rs`** — `Rgb` tuple, `OFF`, and `rainbow(hue, max)`.
- **`indicator.rs`** — `Indicator`, the LED state machine fed by `LedMsg`s (activity, broker connected, MQTT `LedMode`, fault). Priority: fault red > MQTT-set colour (until `auto`) > status colour (yellow busy, blue booting/disconnected, dim blue standby, green idle).
- **`ws2812.rs`** (ESP only) — `Ws2812`, which drives the board's WS2812 on GPIO48 through the RMT `BytesEncoder` (GRB byte order, MSB first). `set` only logs on failure.

**`remote/`** — MQTT control:

- **`json.rs`** — minimal dependency-free JSON reader (`parse` → `Value`) and `quote`.
- **`command.rs`** — `parse(prefix, topic, payload)` → `Command::{Robot(RobotCommand), Led(LedMode), Stop}` with all validation and range limits; also the subscribe/status subtopic names.
- **`status.rs`** — `Snapshot` (state + action) and the status JSON.
- **`pose.rs`** — the `<prefix>/pose` JSON (all 16 joints, clamped, 0.1 degree) and `Throttle` (one frame per `POSE_INTERVAL_MS` = 100 ms).
- **`mqtt.rs`** (ESP only) — WiFi + MQTT threads from the relay project: WiFi watchdog, connection drainer (calls `on_message`; must never publish/subscribe — esp-idf-svc#441 deadlock), subscriber (retries the 4 subscriptions while a flag is false; don't gate on a Connected event), and the only publisher thread (`Outgoing::Status` at QoS 1 / `Outgoing::Pose` at QoS 0 from one channel).

**`main.rs`** — the app: `Config` (toml-cfg), boot (LED thread, PCA9685, countdown, snap to neutral), MQTT start, heartbeat, then `run_commands`. The main thread owns the `Robot` and runs one `RobotCommand` at a time. `on_message` runs on the drainer thread and never blocks: it rejects robot commands while `busy` (set with `swap` there, cleared by the main thread after the command), applies LED commands directly, and turns `stop` into the shared abort flag. The LED thread owns the `Ws2812`; `LedLink` (a channel sender) is the robot's `FaultSignal`; `PoseLink` is its `PoseSink` (stores the latest frame in `Shared::pose`, publishes throttled frames), and `main` also publishes the final pose after each command and the current one on every broker connect. `sway` uses `SWAY_JOINTS` (hips + ankles). If the PCA9685 doesn't answer at boot, `robot` is `None`: no countdown, state `pca9685_not_found`, `Activity::NoServos` (red, but MQTT LED colours still show, unlike `Fault`), robot commands rejected, and `main` idles instead of returning (returning ends the main task and releases the LED pin).

The joint layout follows the kit manual's servo numbering (`Robot Schema.png`: S1–S8 → channels 0–7, S25–S32 → 8–15, and the `Joint` enum is in channel order). Turning directions are verified only for `RShoulderPitch` and `LShoulderPitch` (the left one is inverted); the shoulder rolls are inverted per a 2026-10-12 check; all trims and the other joints are still unchecked. For checking servos one by one use the `joint_test` / `full_range` / `self_test` commands. At boot the firmware logs an I2C scan (expect `found 0x40`). The servo 5 V currently comes from the extension board and can start only about two servos at the same moment (four at once → only two move); stagger moves until a separate 5 V 3 A+ supply is fitted (`MAX_SERVOS_AT_ONCE`; the `sway` command still moves its 4 servos together, which worked on 2026-11-11). Board photo: `ESP32-Board.jpg`; flash through the WCH USB-serial port (`/dev/cu.usbmodem5CF70246821`), not the native-USB port, whose resets often land in download mode.
