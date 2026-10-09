# ESP32-S3 to PCA9685 wiring

The ESP32-S3 drives the PCA9685 over I2C (SDA = GPIO8, SCL = GPIO9, address 0x40). The
PCA9685 drives the 16 servos, which run from their own 5 V supply.

![Wiring diagram](Wiring%20Diagram.png)

Only the PCA9685's logic runs from the ESP32-S3's 3.3 V. Servo current never passes through
the ESP32-S3 or the extension board.

## Pin mapping

Four wires run from the PCA9685's left header to the extension board. Each GPIO row on the
extension board has three pins: **S** (signal), **3.3V** (middle, red) and **GND**.

| PCA9685 pin (left header) | Extension board pin | Purpose |
|---|---|---|
| `GND` | GND pin of any GPIO row | Common ground |
| `VCC` | 3.3V pin of any GPIO row | PCA9685 logic power and I2C pull-ups |
| `SDA` | S pin of the IO8 row (GPIO8) | I2C data |
| `SCL` | S pin of the IO9 row (GPIO9) | I2C clock |
| `OE` | Not connected | Pulled low on the board, so outputs stay enabled |
| `V+` | Not connected | Servo power goes through the screw terminal instead |

## Servo power

The servos get their own 5 V supply rated 3 A or more, connected to the PCA9685's green
screw terminal.

- Use the screw terminal (`+` / `GND`), not the `V+` header pin. On this board design the
  terminal usually goes through a reverse-polarity protection part, and the header pin
  bypasses it. Check the markings before tightening.
- 16 SG90/MG90S servos moving together draw about 3-4 A at peak. Never power them from the
  ESP32-S3, its USB port, or the extension board's 5V header: the dips will reset the
  ESP32-S3.
- Connect the grounds of the servo supply, the PCA9685 and the ESP32-S3. The GND wire in the
  pin mapping already joins the PCA9685 and the ESP32-S3.
- If the PCA9685's onboard capacitor is under 1000 uF, add a 1000 uF one across the screw
  terminal to prevent brownout resets.

## Servo channels

Servos plug in by the kit manual's numbers (see [the schematic](Robot%20Schema.png)): S1-S8
on channels 0-7 and S25-S32 on channels 8-15. Which side is right is still a guess until the
self-test confirms it.

| Channel | Servo | Joint |
|---|---|---|
| 0 | S1 | Right ankle roll |
| 1 | S2 | Right knee lower |
| 2 | S3 | Right knee upper |
| 3 | S4 | Right hip roll |
| 4 | S5 | Right shoulder pitch |
| 5 | S6 | Right shoulder roll |
| 6 | S7 | Right elbow |
| 7 | S8 | Right gripper |
| 8 | S25 | Left gripper |
| 9 | S26 | Left elbow |
| 10 | S27 | Left shoulder roll |
| 11 | S28 | Left shoulder pitch |
| 12 | S29 | Left hip roll |
| 13 | S30 | Left knee upper |
| 14 | S31 | Left knee lower |
| 15 | S32 | Left ankle roll |

On each channel column, yellow = signal, red = V+ and black = GND. With orange / red / brown
servo leads, orange goes on yellow and brown on black.

## Checks before power-on

- [ ] `VCC` goes to **3.3V, not 5V**. The PCA9685 board's I2C pull-ups run from `VCC`, so
  5 V would put 5 V on GPIO8/9 and could damage the ESP32-S3.
- [ ] Address pads A0-A5 are all open (address 0x40).
- [ ] The pins on the PCA9685's right edge are empty. They only chain a second board.
- [ ] Servo supply polarity matches the `+` / `GND` marks on the screw terminal.
- [ ] The PCA9685's POWER LED is lit.

Don't use GPIO35, 36 or 37 for anything added later: the N16R8 module's octal PSRAM uses
them, even though the extension board breaks them out.

If the boot log still shows `PCA9685 not responding at 0x40: ESP_FAIL`, recheck SDA/SCL
(they are easy to swap) and the shared ground.
