# Robot Controller UI — Design Specification

Exported 2026-11-14 from the master copy in Claude Docs:
<https://claude.ai/artifact/M3DMid6nr3PMxeVKKqaTHs>. Edit there and re-export.

## Overview

A phone-first web app, styled as a game-console controller, drives the 16-servo biped through a Rust BFF that speaks WebSocket to the browser and MQTT to the robot. The UI is Rust compiled to WebAssembly; the BFF is the only component that talks to the MQTT broker.

**Goals**

- One-thumb control of eight basic functions: walk forward, walk backward, turn left, turn right, stand up, sway body, spread arms, rainbow.
- A live LED mirror on screen that shows the same colour as the robot's on-board RGB LED.
- X / O button combos that feel like a console controller (hold, press, chord).
- A STOP that always works, even while the robot is busy.

**Non-goals (v1)**

- Per-servo sliders or calibration (the existing `joint_test` / `full_range` MQTT commands stay the tool for that).
- Multiple robots, user accounts, or access from outside the home network.
- Video streaming.

## Architecture

```
┌────────────────┐  WebSocket /ws  ┌────────────────┐      MQTT      ┌────────────────┐      MQTT      ┌──────────────────┐
│ Phone UI       │◀──────────────▶│ BFF            │◀─────────────▶│ MQTT broker    │◀─────────────▶│ ESP32-S3 robot   │
│ Bevy → wasm    │                 │ Axum + rumqttc │                │ Mosquitto      │                │ 16 servos + LED  │
│ PWA gamepad    │                 │ intent → topic │                │ robot/#        │                │ status every 30 s│
└────────────────┘                 └────────────────┘                └────────────────┘                └──────────────────┘
```

Only the BFF talks MQTT; the phone speaks WebSocket. The phone sends intents ("walk forward") over one WebSocket; the BFF turns them into the firmware's existing `robot/cmd` and `robot/led` messages and pushes every `robot/status` back to all connected phones. The BFF also serves the static UI files, so the phone needs one URL.

## Technology stack

The UI is a Bevy app rendering to a single full-screen `<canvas>` (WebGL2), built with Trunk; the BFF is Axum on Tokio with rumqttc. Both share one `protocol` crate so the WebSocket messages are type-checked on both ends. Canvas is a hard requirement: it is why the UI is Rust/wasm, and it leaves room for later features such as an animated view of the robot's current pose.

| Layer | Choice | Why |
| --- | --- | --- |
| UI engine | Bevy (2D now, 3D-capable) compiled to `wasm32-unknown-unknown`, WebGL2 backend | Everything drawn on one canvas; the same app can later add a 2D or 3D robot model animated from live joint angles |
| UI build | Trunk (`trunk serve`; release with `--cargo-profile wasm-release` + `data-wasm-opt="s"`) | Standard Rust-wasm loop; size profile: `opt-level = "s"`, fat LTO, `codegen-units = 1`, trimmed Bevy features |
| Mobile packaging | PWA: manifest, service worker, `display: fullscreen`, landscape lock | Installs to the home screen like an app, no store |
| Touch input | Bevy `Touches` resource (multi-touch), hit-testing against button shapes in the app; `navigator.vibrate` via `web-sys` for haptics | Chords and holds handled in a Bevy input system |
| WebSocket client | `web-sys` WebSocket (or `ewebsock`) feeding a Bevy event channel | Non-blocking in the browser's event loop |
| BFF framework | Axum on Tokio | Built-in WebSocket upgrade, Tower middleware, serves the static UI too |
| MQTT client | rumqttc (async) | Tokio-native, auto-reconnect via its event loop |
| Shared types | `protocol` crate with serde | Same enums in wasm and server |

Alternatives considered: a DOM framework such as Leptos or Yew (smaller bundle, but no canvas, which rules it out); Leptos with a hand-drawn canvas via `web-sys` (light, but every later feature, such as the 3D pose view, would be built from scratch); Actix-web or Poem for the BFF (Axum has the simplest WebSocket + static-file story on Tokio).

**Canvas only.** Every control, the LED mirror and the status bar are drawn by Bevy on the canvas; the HTML page holds only the `<canvas>` and a loading message. Costs to accept: a larger download (a trimmed Bevy build measured 7.5 MB gzipped, so the PWA caches it after the first load), screen-reader labels must be added separately (an ARIA live region mirroring the status text), and testing needs real taps, because synthetic JavaScript events don't reach winit.

**Future: live pose view.** A panel showing the robot's 16 joints animated from their current logical angles. It needs the firmware to publish the pose (for example `robot/pose` with 16 angles at about 10 Hz while moving) and the BFF to forward it as a `pose` WebSocket message.

## Controller UI

The screen is a landscape gamepad: D-pad on the left for movement, X / O / triangle / square on the right for actions, the LED mirror in the centre, and a red STOP pill under it.

```
 ┌──────────────────────────────────────────────────────────────┐
 │  ● connected · robot idle · walk done              [⚙]      │
 │                                                              │
 │      ▲                 ╭───────╮                △            │
 │   ◀  ●  ▶              │  LED  │            □       ○        │
 │      ▼                 ╰───────╯                ✕            │
 │                        [ STOP ]                              │
 │   L                                                   R      │
 └──────────────────────────────────────────────────────────────┘
```

**LED mirror.** A round 96 px glowing disc whose colour is the BFF's best estimate of the on-board LED (see BFF section): green idle, yellow busy, dim blue standby, blue disconnected or booting, red fault or no servo board, animated hue cycle for rainbow, or the MQTT-set colour. A text label under it states the same state, so colour is never the only signal.

**Status bar.** Two dots: UI↔BFF WebSocket and BFF↔robot (last status within 45 s, since heartbeats arrive every 30 s). Then `state · action · reason`, and the `error` text in red when a command is rejected.

**Button mapping**

| Function | Input | Robot action |
| --- | --- | --- |
| Walk forward | D-pad ▲ (tap = 2 steps, hold = repeat) | `walk` forward |
| Walk backward | D-pad ▼ | `walk` backward (firmware gap) |
| Turn left | D-pad ◀ | `turn` left (firmware gap) |
| Turn right | D-pad ▶ | `turn` right (firmware gap) |
| Stand up | X | `stand_up` |
| Sway body | O | `sway` |
| Spread arms | X + O pressed together (within 150 ms) | `arms` out (firmware gap: `spread_arms`) |
| Rainbow | Hold O for 1 s | LED `rainbow`; hold again for `auto` |
| Stop | STOP pill, or any two of the four D-pad keys at once | `stop` |
| Stand by | Hold X for 1 s | `stand_by` |

**Combo rules.** Chords are detected in the UI: the first button press opens a 150 ms window; a second press inside it forms a chord and cancels the single. Holds fire at 1 s with a ring progress animation. Every accepted input gives a 20 ms vibration; a rejected one (`rejected_busy`) gives a double buzz and shakes the LED disc.

**Busy handling.** While `state = busy`, motion buttons dim but stay tappable (the BFF rejects them, giving feedback); STOP and rainbow stay full brightness because the firmware accepts them while busy.

**Layout.** Landscape, safe-area insets respected, buttons at least 64 px, no text selection or long-press menu (`touch-action: none`, `user-select: none`). Portrait shows a "rotate your phone" card.

## WebSocket protocol (UI ↔ BFF)

One WebSocket at `/ws`, JSON text frames tagged by `type`, defined once as serde enums in the `protocol` crate. The UI sends intents, never MQTT topics; the BFF owns the mapping.

**UI → BFF**

```json
{"type":"action","id":17,"action":"walk_forward"}
{"type":"action","id":18,"action":"turn_left"}
{"type":"stop","id":19}
{"type":"led","id":20,"mode":"rainbow"}
{"type":"ping","t":1731571200000}
```

`action` is one of `walk_forward`, `walk_backward`, `turn_left`, `turn_right`, `stand_up`, `stand_by`, `sway`, `spread_arms`. `mode` is `rainbow`, `auto`, `off` or a named colour.

**BFF → UI**

```json
{"type":"ack","id":17,"ok":true}
{"type":"ack","id":18,"ok":false,"error":"robot busy (walk)"}
{"type":"status","robot_online":true,"state":"busy","action":"walk","reason":"started","error":null,"led":{"kind":"solid","rgb":[64,48,0]}}
{"type":"pong","t":1731571200000}
```

- On connect the BFF sends the latest `status` at once, so a new phone shows the LED without waiting for the next heartbeat.
- `ack` means "published to MQTT" (or refused by the BFF); the real outcome arrives as `status` with `started`, `done`, `stopped` or `rejected_*`.
- `led.kind` is `solid` (with `rgb`), `rainbow` (UI animates the hue locally) or `off`.
- The UI pings every 10 s and reconnects with backoff 0.5 s → 8 s; the controls grey out while the socket is down.

## BFF ↔ robot (MQTT)

The BFF publishes to the existing firmware topics under `robot/` and subscribes to `robot/status`; four of the eight functions need new firmware actions before they work.

| UI action | MQTT topic | Payload | Firmware today |
| --- | --- | --- | --- |
| `walk_forward` | `robot/cmd` | `{"cmd":"walk","steps":2}` | Exists (knees unverified) |
| `walk_backward` | `robot/cmd` | `{"cmd":"walk","steps":2,"dir":"back"}` | Gap: add `dir` to `walk` (reverse phase order) |
| `turn_left` / `turn_right` | `robot/cmd` | `{"cmd":"turn","dir":"left","steps":2}` | Gap: new `turn` action (asymmetric shuffle) |
| `stand_up` | `robot/cmd` | `stand_up` | Exists |
| `stand_by` | `robot/cmd` | `stand_by` | Exists |
| `sway` | `robot/cmd` | `{"cmd":"sway","times":2}` | Exists |
| `spread_arms` | `robot/cmd` | `spread_arms` | Gap: new pose, shoulder rolls out (only `arms` up/down exists) |
| rainbow / auto | `robot/led` | `rainbow` / `auto` | Exists, works while busy |
| stop | `robot/cmd` | `stop` | Exists, within 20 ms |

**State the BFF keeps.** Last `robot/status` JSON and its arrival time; the LED mode last set over MQTT (`auto` by default). From these it derives `led` for the UI using the same priority as `rgb_led::indicator`: fault red > MQTT-set colour > status colour. Ideally the firmware adds an `"led":[r,g,b]` field (or `"led":"rainbow"`) to its status JSON so the mirror is exact rather than inferred; until then the BFF re-implements the priority.

**Busy gate.** The firmware already rejects robot commands while busy. The BFF also refuses locally when the last status is `busy`, so the phone gets an `ack ok:false` without a broker round-trip; `stop` and LED commands always pass.

**Robustness.** rumqttc reconnects with keep-alive 10 s, QoS 1 for commands, QoS 0 subscription to status. Fan-out to all WebSocket clients goes through a `tokio::sync::broadcast` channel; the latest status lives in a `watch` channel. Robot is "offline" when no status arrived for 45 s.

**Security.** LAN only, bound to `0.0.0.0:8080`; a required shared token as a `?token=` query on `/ws` checked by the BFF. The broker credentials stay in the BFF's config, never in the browser.

## Project layout, build and milestones

A separate Cargo workspace next to the firmware (the firmware crate only builds for Xtensa, so it can't share a workspace with host and wasm crates).

```
robot-console/
  Cargo.toml          # workspace + [profile.wasm-release]
  protocol/           # serde enums: ClientMsg, ServerMsg, Action, LedView
  bff/                # axum + rumqttc; serves ui/dist at /, WebSocket at /ws
  ui/                 # bevy app; index.html (canvas only), Trunk.toml, manifest, sw.js
```

- Dev: `trunk serve --port 8081 --proxy-backend=ws://localhost:8080/ws` in `ui/`, `cargo run -p bff` in parallel, Mosquitto on the Mac.
- Release: `trunk build --release --cargo-profile wasm-release`, then the BFF serves `ui/dist`. Measure wasm size gzipped and trim Bevy features until the first load is acceptable on mobile data (budget: under 8 MB gzipped).
- Tests: unit tests for `protocol` round-trips, the action → MQTT mapping and the LED-priority function; an integration test runs the BFF against a local Mosquitto and a fake robot that echoes status. Clippy the UI for `wasm32-unknown-unknown`.
- Manual check on a real phone: chords, hold timing, haptics, reconnect after Wi-Fi drop.

**Milestones**

1. BFF bridge: WebSocket ↔ MQTT for the four existing actions, status fan-out.
2. Controller UI: layout, LED mirror, X / O / chords, STOP.
3. Firmware: `walk` direction, `turn`, `spread_arms`, `led` field in status.
4. PWA polish: install, fullscreen, landscape, offline shell. Later: live pose view on the canvas (needs `robot/pose` from the firmware).

**Decisions (made 2026-11-14)**

- **Turning:** asymmetric shuffle. Without hip yaw, `turn` repeats a shuffle where one leg travels further than the other, turning a few degrees per step. Step lengths are tuned in a hardware trial.
- **Holding the D-pad:** repeat 2-step walks. The BFF sends `walk` with `steps: 2`, then another after each `done`, until the button is released; the walk in progress finishes its 2 steps. STOP still cuts it at once.
- **BFF host:** the Mac, next to Mosquitto. No new hardware; the BFF connects to the broker on `localhost`.
- **Security:** a shared token is required. It lives in the BFF config, the phone enters it once (kept in local storage) and sends it as `?token=` on `/ws`; a wrong token closes the socket.
