# ESP32-S3 to PCA9685 wiring

The ESP32-S3 drives the PCA9685 over I2C (SDA = GPIO8, SCL = GPIO9, address 0x40). The
PCA9685 drives the 16 servos, which run from their own 5 V supply.

![Wiring diagram](Wiring%20Diagram.png)

Only the PCA9685's logic runs from the ESP32-S3's 3.3 V. Servo current never passes through
the ESP32-S3 or the extension board. Servo power enters through the PCA9685's `V+` header
pin, because the screw terminal on this board does not pass power through (see Servo power).

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
| `V+` | Not connected to the ESP32-S3 | Servo 5 V input (see Servo power) |

## Servo power

The servos get their own 5 V supply rated 3 A or more, connected to the **left-header `V+`
and `GND` pins**, not the green screw terminal.

On this board the terminal's `+` goes through a reverse-polarity protection transistor before
it reaches the servo rail, and that transistor does not conduct: the terminal reads 5 V with
correct polarity while the channel red/black pins read under 1 V. The header `V+` pin joins
the rail after the transistor, so it works.

```
terminal +  --> [protection transistor, not conducting] --> V+ rail --> red pin of every channel
                                                              ^
left-header V+ pin -------------------------------------------+
```

| Supply wire | PCA9685 pin |
|---|---|
| `+` 5 V | Left-header `V+` (bottom pin, below `VCC`) |
| `-` GND | Terminal `GND` screw (right screw) or left-header `GND` |

- **No reverse-polarity protection on this path.** Reversed power goes straight to the
  servos. Use red for `V+` and black for `GND`, ideally in a keyed connector, and check the
  polarity before switching on.
- **Don't put 5 V on `VCC`.** It is the pin right above `V+` and must stay on 3.3 V.
- The terminal `GND` screw, the header `GND` pin and every channel's black pin are the same
  ground; only the `+` side is broken. Keep the header `GND` wire to the extension board
  either way, so I2C and the servo supply share a ground.
- 16 SG90/MG90S servos moving together draw about 3-4 A at peak. That is too much for a
  single thin Dupont jumper on `V+`: use a thicker wire soldered to the pin, or bridge the
  protection transistor so the terminal works again (that path is unprotected too).
- Never power more than one or two servos from the ESP32-S3, its USB port, or the extension
  board's 5V header: the dips will reset the ESP32-S3. One servo on the extension board's
  5V pin is fine for bench tests.
- If the PCA9685's onboard capacitor is under 1000 uF, add a 1000 uF one across `V+` and
  `GND` to prevent brownout resets.

## Servo channels

Servos plug in by the kit manual's numbers (see [the schematic](Robot%20Schema.png)): S1-S8
on channels 0-7 and S25-S32 on channels 8-15. Which side is right is still a guess until the
self-test confirms it.

| Channel | Servo | Joint | ข้อต่อ |
|---|---|---|---|
| 0 | S1 | Right ankle roll | ข้อเท้าขวา (เอียงข้าง) |
| 1 | S2 | Right knee lower | เข่าขวา ตัวล่าง |
| 2 | S3 | Right knee upper | เข่าขวา ตัวบน |
| 3 | S4 | Right hip roll | สะโพกขวา (เอียงข้าง) |
| 4 | S5 | Right shoulder pitch | ไหล่ขวา (แกว่งหน้า-หลัง) |
| 5 | S6 | Right shoulder roll | ไหล่ขวา (กาง-หุบแขน) |
| 6 | S7 | Right elbow | ข้อศอกขวา |
| 7 | S8 | Right gripper | มือจับขวา |
| 8 | S25 | Left gripper | มือจับซ้าย |
| 9 | S26 | Left elbow | ข้อศอกซ้าย |
| 10 | S27 | Left shoulder roll | ไหล่ซ้าย (กาง-หุบแขน) |
| 11 | S28 | Left shoulder pitch | ไหล่ซ้าย (แกว่งหน้า-หลัง) |
| 12 | S29 | Left hip roll | สะโพกซ้าย (เอียงข้าง) |
| 13 | S30 | Left knee upper | เข่าซ้าย ตัวบน |
| 14 | S31 | Left knee lower | เข่าซ้าย ตัวล่าง |
| 15 | S32 | Left ankle roll | ข้อเท้าซ้าย (เอียงข้าง) |

On each channel column, yellow = signal, red = V+ and black = GND. With orange / red / brown
servo leads, orange goes on yellow and brown on black.

## Checks before power-on

- [ ] `VCC` goes to **3.3V, not 5V**. The PCA9685 board's I2C pull-ups run from `VCC`, so
  5 V would put 5 V on GPIO8/9 and could damage the ESP32-S3.
- [ ] Address pads A0-A5 are all open (address 0x40).
- [ ] The pins on the PCA9685's right edge are empty. They only chain a second board.
- [ ] Servo supply `+` goes to header `V+`, not `VCC`, and its `-` to `GND`.
- [ ] Channel 0's red/black pins read about 5 V with the supply on.
- [ ] The PCA9685's POWER LED is lit.

Don't use GPIO35, 36 or 37 for anything added later: the N16R8 module's octal PSRAM uses
them, even though the extension board breaks them out.

If the boot log still shows `PCA9685 not responding at 0x40: ESP_FAIL`, recheck SDA/SCL
(they are easy to swap) and the shared ground.
