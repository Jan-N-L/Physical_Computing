###############################################################################
# This script reads temperature and humidity from a DHT sensor and displays it
# on an LCD screen. It also activates a buzzer if values exceed a threshold.
#
# Note: This code works with DHT sensor v1.1 / v1.2, which has a BLUE plastic 
# cover. The sensor communicates via a digital pin (e.g., D16, D18, or D20).
###############################################################################

# This script requires an extra file:
# - lcd1602.py: Contains the class to control the LCD display
# Make sure to upload this file to your Raspberry Pi Pico!

# Import necessary modules and classes
from lcd1602 import LCD1602        # LCD display class
from dht import DHT11 as DHT       # DHT sensor class (version 11)
from machine import I2C, Pin, PWM  # Hardware/GPIO interfaces
from utime import sleep            # Used to pause the program

# Set up I2C communication for the LCD display
i2c0 = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)
#i2c1 = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)

# Let's create the display instance
d = LCD1602(i2c0, 2, 16)

# Set up the DHT sensor on digital pin D18
# (Make sure you're using the BLUE DHT sensor!)
sensor = DHT(Pin(18))
sleep(0.5)

# Set up a passive buzzer on digital pin D16
buzzer = PWM(Pin(16))
# Set buzzer frequency to 500 Hz
buzzer.freq(500)      


# Main loop: runs forever
while True:
    sensor.measure()                    # Measure temperature and humidity
    sleep(1)                            # Wait a bit before reading the data 
    temp = sensor.temperature()         # Get temperature value
    hum = sensor.humidity()             # Get humidity value
    
    # Display temperature on the first row of the LCD
    d.setCursor(0,0)                    # Move cursor to top-left
    d.print("Temp:  %3.1f" % temp)      # Show temperature with 1 decimal
    
    # Display humidity on the second row of the LCD
    d.setCursor(0,1)                    # Move cursor to start of second row
    d.print("Humid: %3.1f" % hum)       # Show humidity with 1 decimal
    sleep(1)                            # Pause before next update

    # Turn buzzer on if temperature or humidity exceed some threshold
    if temp > 25 or hum > 80:
        buzzer.duty_u16(200)            # Set buzzer volume (PWM duty cycle)
    else:
        buzzer.duty_u16(0)              # Turn buzzer off

    