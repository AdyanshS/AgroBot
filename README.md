
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
