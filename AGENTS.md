# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

ESP32-S3 firmware (Rust, `esp-idf-svc` std stack) for a 16-servo biped driven through a PCA9685 over I2C (SDA GPIO8, SCL GPIO9, addr 0x40). Moves are hardcoded and run once at boot; no networking. Not yet run on real hardware. See README.md for wiring, power, and the first-run calibration procedure.

## Build and flash

The toolchain is pinned to `esp-1.98` in `rust-toolchain.toml` (the 1.99.0.0 esp release fails to build std for espidf with "`AT_FDCWD` not found in `libc`"). Don't bump it casually.

```bash
. $HOME/export-esp.sh
cargo build --release
espflash flash --monitor -p /dev/cu.usbmodem5CF70246821 target/xtensa-esp32s3-espidf/release/robot-controller
```

## Tests

`cargo test` does **not** work: the crate only builds for the Xtensa ESP32-S3 target (set in `.cargo/config.toml`). The four hardware-independent modules (`servo`, `pca9685`, `motion`, `moves`) carry `#[cfg(test)]` unit tests that run on the host by compiling them into a throwaway test crate with the stable toolchain. Keep that crate outside the repo (e.g. a temp dir):

```bash
P=$PWD/src
cat > host_tests.rs <<EOF
#[path = "$P/servo.rs"] mod servo;
#[path = "$P/pca9685.rs"] mod pca9685;
#[path = "$P/motion.rs"] mod motion;
#[path = "$P/moves.rs"] mod moves;
EOF
rustc +stable --edition 2021 --test host_tests.rs -o host_tests && ./host_tests
./host_tests walk_stays_inside   # run a single test by name filter
```

All four modules must be declared at the test crate root because they reference each other via `crate::servo`, `crate::motion`, etc.

## Architecture

Layered so everything except `main.rs` is pure logic with no ESP dependencies:

- **`servo.rs`** — the only module that knows about physical servos. Defines the `Joint` enum (order must match `CALIBRATION`), the per-joint `Calib` table (PCA channel, min/max limits, trim, invert), and `angle_to_ticks` / `pose_to_ticks`. Everywhere else works in **logical degrees** (90 = neutral, with sign conventions documented on `Joint`); limit clamping, inversion and trim are applied only here. `pose_to_ticks` remaps from joint index to PCA channel index.
- **`pca9685.rs`** — minimal driver generic over a tiny `Transport` trait (`write` + `delay_ms`) rather than an I2C crate, so it can be tested with a mock.
- **`motion.rs`** — `Pose` (array of logical angles indexed by `Joint::index()`; `with` sets an offset from neutral, `add` adds a delta) and `Robot<H: Hardware>`, which tracks the current pose and interpolates to targets with smoothstep at `STEP_MS` intervals. `Hardware` trait = `write_pose` + `delay_ms`. `test_support::MockHw` records poses/elapsed time for tests.
- **`moves.rs`** — self-test, arm up/down, and an open-loop 5-phase walk parameterized by `Gait` (`DEFAULT_GAIT`). Each leg is hip roll, two stacked knee pitch servos and ankle roll (no hip or ankle pitch), so the walk is a lean-and-shuffle gait. `frame()` keeps the feet flat by turning the knee pair by equal and opposite amounts (`lower = -upper`) and setting ankle roll to `-hip roll`. A test asserts the default walk stays within every joint's calibrated limits — keep it passing when tuning the gait or calibration.
- **`main.rs`** — ESP glue only: implements `Transport` (`I2cBus`) and `Hardware` (`Servos`, which calls `servo::pose_to_ticks` with `CALIBRATION`), boot countdown, then runs the demo chosen by the `DEMO` const. `RELEASE_AT_END` controls whether servos go limp afterwards.

The joint layout follows the kit manual's servo numbering (`Robot Schema.png`: S1–S8 → channels 0–7, S25–S32 → 8–15, and the `Joint` enum is in channel order). Which side is right, the turning directions and trims are still unverified pending the hardware self-test; `DEMO` defaults to `Demo::SelfTest` for that reason.
