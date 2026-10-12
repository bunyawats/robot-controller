# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ESP32-S3 firmware (Rust, `esp-idf-svc` std stack) for a 16-servo biped driven through a PCA9685 over I2C (SDA GPIO8, SCL GPIO9, addr 0x40). Moves are hardcoded and run once at boot; no networking. Hardware bring-up is in progress: both shoulder pitch joints are verified and `Demo::Arms` runs. See README.md for wiring, power, and the first-run calibration procedure.

## Build and flash

The toolchain is pinned to `esp-1.98` in `rust-toolchain.toml` (the 1.99.0.0 esp release fails to build std for espidf with "`AT_FDCWD` not found in `libc`"). Don't bump it casually.

```bash
. $HOME/export-esp.sh
cargo build --release
espflash flash --monitor -p /dev/cu.usbmodem5CF70246821 target/xtensa-esp32s3-espidf/release/robot-controller
```

## Tests

`cargo test` does **not** work: the crate only builds for the Xtensa ESP32-S3 target (set in `.cargo/config.toml`). The hardware-independent modules (`robot_16_dof::{servo, pca9685, motion, moves}` and `rgb_led::color`) carry `#[cfg(test)]` unit tests that run on the host by compiling them into a throwaway test crate with the stable toolchain. Their ESP-only siblings (`robot_16_dof::esp32`, `rgb_led::ws2812`) are behind `#[cfg(target_os = "espidf")]`, so the two `mod.rs` files can be included whole. Keep that crate outside the repo (e.g. a temp dir):

```bash
P=$PWD/src
cat > host_tests.rs <<EOF
#[path = "$P/robot_16_dof/mod.rs"] mod robot_16_dof;
#[path = "$P/rgb_led/mod.rs"] mod rgb_led;
EOF
rustc +stable --edition 2021 --test host_tests.rs -o host_tests && ./host_tests
./host_tests walk_stays_inside   # run a single test by name filter
```

Both modules must be declared at the test crate root under these names, because the robot modules reference each other via `crate::robot_16_dof::servo` etc.

## Architecture

Two top-level modules plus `main.rs`. In each module, everything except the one ESP-only file is pure logic with no ESP dependencies.

**`robot_16_dof/`** — the 16-servo biped:

- **`servo.rs`** — the only module that knows about physical servos. Defines the `Joint` enum (order must match `CALIBRATION`), the per-joint `Calib` table (PCA channel, min/max limits, trim, invert), and `angle_to_ticks` / `pose_to_ticks`. Everywhere else works in **logical degrees** (90 = neutral, with sign conventions documented on `Joint`); limit clamping, inversion and trim are applied only here. `pose_to_ticks` remaps from joint index to PCA channel index.
- **`pca9685.rs`** — minimal driver generic over a tiny `Transport` trait (`write` + `delay_ms`) rather than an I2C crate, so it can be tested with a mock.
- **`motion.rs`** — `Pose` (array of logical angles indexed by `Joint::index()`; `with` sets an offset from neutral, `add` adds a delta) and `Robot<H: Hardware>`, which tracks the current pose and interpolates to targets with smoothstep at `STEP_MS` intervals. `Hardware` trait = `write_pose` + `delay_ms`. `test_support::MockHw` records poses/elapsed time for tests.
- **`moves.rs`** — self-test, joint test, arm up/down, and an open-loop 5-phase walk parameterized by `Gait` (`DEFAULT_GAIT`). Each leg is hip roll, two stacked knee pitch servos and ankle roll (no hip or ankle pitch), so the walk is a lean-and-shuffle gait. `frame()` keeps the feet flat by turning the knee pair by equal and opposite amounts (`lower = -upper`) and setting ankle roll to `-hip roll`. A test asserts the default walk stays within every joint's calibrated limits — keep it passing when tuning the gait or calibration.
- **`esp32.rs`** (ESP only) — `I2cBus` (`Transport` over the ESP-IDF I2C driver, 100 kHz, one retry per write), `connect_pca9685` (opens I2C, logs a bus scan, inits the PCA9685 at 50 Hz), and `Servos<F: FaultSignal>` (`Hardware` via `servo::pose_to_ticks` with `CALIBRATION`; calls `F::fault` on the first failed write). The robot module doesn't know about the LED; `FaultSignal` is how `main.rs` connects the two.

**`rgb_led/`** — the board's 3-colour LED:

- **`color.rs`** — `Rgb` tuple, `OFF`, and `rainbow(hue, max)`.
- **`ws2812.rs`** (ESP only) — `Ws2812`, which drives the board's WS2812 on GPIO48 through the RMT `BytesEncoder` (GRB byte order, MSB first). `set` only logs on failure.

**`main.rs`** — the app: the `Demo` enum and `DEMO` const, status colours (blue countdown, green running, red on any PCA9685 failure via `impl FaultSignal for Ws2812`, off when done), boot countdown, then runs the chosen demo. `RELEASE_AT_END` controls whether servos go limp afterwards.

The joint layout follows the kit manual's servo numbering (`Robot Schema.png`: S1–S8 → channels 0–7, S25–S32 → 8–15, and the `Joint` enum is in channel order). Turning directions are verified only for `RShoulderPitch` and `LShoulderPitch` (the left one is inverted); all trims and the other joints are still unchecked, so `DEMO` defaults to `Demo::SelfTest`. `Demo::Joints(&[...])` swings each listed joint +30 degrees and back three times, one after another, for checking servos one by one. At boot the firmware logs an I2C scan (expect `found 0x40`). `Demo::LedCycle` only fades the status LED through the rainbow and never touches I2C, so it runs with the PCA9685 disconnected. Board photo: `ESP32-Board.jpg`; flash through the WCH USB-serial port (`/dev/cu.usbmodem5CF70246821`), not the native-USB port, whose resets often land in download mode.
