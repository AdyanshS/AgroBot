
# AgroBot

A ros2 based four wheel omni wheel drive (X Drive)

## Firmware

Using Esp32 with PlatformIO and micro ros

### Pinouts

For encoder these pins have been tested and working.

|  Motor No | Encoder | ESP32 |
|-----------|---------|-------|
|    1      | A       |  14   |
|           | B       |  27   |
|    2      | A       |  26   |
|           | B       |  25   |
|    3      | A       |  13   |
|           | B       |  12   |
|    4      | A       |  33   |
|           | B       |  32   |

Power to encoders was provided using 5V from ESP32

### Joystick

Check batter percentage

```bash
 upower -i /org/freedesktop/UPower/devices/gaming_input_ps_controller_battery_98ob6oe9oacoe2
```

### IMPORTANT NOTES

#### Servo Number Connected and Their Angles (0-300)

- **Servo 1**: Servo Claw
  - Ground facing pose: 180
  - Full back pose: 115
  - Front facing pose: 260

- **Servo 2**: Servo Grip
  - Opened Pose: 180
  - Closed Pose: 150

- **Servo 3**: Servo Arm
  - Extended Pose: 300
  - Retracted Pose: 180
  