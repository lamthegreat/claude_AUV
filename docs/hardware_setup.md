# Hardware Setup

## BOM (Bill of Materials)

| Item | Qty | Notes |
|------|-----|-------|
| Raspberry Pi 5 (4GB) | 1 | Main computer |
| Teensy 4.1 | 1 | Sensor hub MCU |
| Teensy 4.0 | 1 | Motor controller MCU |
| BNO085 breakout | 1 | IMU (SparkFun or Adafruit breakout) |
| DI 2205-2207 brushless motor | 6 | Need waterproofing |
| Bidirectional ESC | 6 | Standard 30A hobby ESC, or BlueRobotics Basic ESC |
| USB cable (USB-A to Micro-B) | 2 | Pi 5 → each Teensy |
| Depth sensor | 1 | Recommended: BlueRobotics Bar30 (future) |
| Waterproof enclosure | 1 | For electronics |
| LiPo battery | 1 | 4S 14.8V, capacity TBD based on runtime needs |

## Wiring Diagram

### Teensy 4.1 (Sensor Hub)

```
BNO085 (SPI)
  VIN  → 3.3V
  GND  → GND
  SCK  → Teensy pin 13 (SPI0 SCK)
  MISO → Teensy pin 12 (SPI0 MISO)
  MOSI → Teensy pin 11 (SPI0 MOSI)
  CS   → Teensy pin 10 (BNO085_CS_PIN)
  INT  → Teensy pin 8  (BNO085_INT_PIN)
  RST  → Teensy pin 7  (BNO085_RST_PIN)
  WAKE → Teensy pin 9  (BNO085_WAKE_PIN)

Future Depth Sensor (I2C1)
  SDA  → Teensy pin 17
  SCL  → Teensy pin 16

USB → Raspberry Pi 5 USB port (becomes /dev/auv_sensor_hub)
```

### Teensy 4.0 (Motor Controller)

```
ESC Signal Wires (PWM 50Hz)
  ESC 0 (front_left)   → Teensy pin 2
  ESC 1 (front_right)  → Teensy pin 3
  ESC 2 (rear_left)    → Teensy pin 4
  ESC 3 (rear_right)   → Teensy pin 5
  ESC 4 (vert_front)   → Teensy pin 6
  ESC 5 (vert_rear)    → Teensy pin 7

ESC Ground → Common ground with Teensy GND
ESC Power  → Battery / power distribution board

USB → Raspberry Pi 5 USB port (becomes /dev/auv_motor_ctrl)
```

## Power Architecture

```
LiPo (4S ~14.8V)
    │
    ├── Power Distribution Board
    │       ├── ESC 1-6 (direct 14.8V input)
    │       └── BEC (5V/3A) → Teensy 4.0
    │
    └── Voltage Regulator (5V/5A) → Raspberry Pi 5 (via USB-C)
                                  → Teensy 4.1
```

## USB Device Identification

After flashing firmware to each Teensy, identify its USB serial number:

```bash
# Plug in one Teensy at a time
udevadm info -a -n /dev/ttyACM0 | grep -i serial

# Example output:
#   ATTRS{serial}=="7770980"
```

Update `config/udev/99-auv-teensy.rules` with these values, then deploy the rules.
