from machine import Pin, I2C
from vl53l0x import VL53L0X

# This script requires an extra file:
# - vl53l0x.py: Contains the class to read the distance sensor
# Make sure to upload this file to your Raspberry Pi Pico!

# Connect the distance sensor to one of the I2C ports (IC20 or IC21)
i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))    # I2C port 0
i2c1 = I2C(id=1, scl=Pin(7), sda=Pin(6))    # I2C port 1

# Create a sensor object using the selected I2C port
tof = VL53L0X(i2c0)

# Main loop to read distance measurements
while True:
    try:
        x = tof.ping()                 # Measure distance in millimeters
        print(x, "mm")                 # Print the result
    except RuntimeError:
        print("Retrying!")             # If reading fails, try again
