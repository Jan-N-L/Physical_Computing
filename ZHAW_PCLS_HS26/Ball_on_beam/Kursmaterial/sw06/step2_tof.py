from machine import Pin, I2C
from lcd1602 import LCD1602        # Interface to the display
from vl53l0x import VL53L0X        # Interface to time-of-flight sensor
from utime import sleep

# Connect the distance sensor to port IC20
i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))
tof = VL53L0X(i2c0)

# Connect the display to port IC21
i2c1 = I2C(id=1, scl=Pin(7), sda=Pin(6))
display = LCD1602(i2c1, 2, 16)

while True:
    try:
        x = tof.ping()                        # Measure the distance
        display.setCursor(0,0)                # Place the cursor at (0,0)
        display.print("Distance: %4dmm" % x)  # Print the distance
        sleep(0.1)
    except RuntimeError:
        print("Retrying!")
