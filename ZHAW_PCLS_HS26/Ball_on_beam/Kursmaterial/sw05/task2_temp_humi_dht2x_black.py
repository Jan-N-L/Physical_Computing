###############################################################################
# This script reads temperature and humidity from a DHT sensor and displays it
# on an LCD screen. It also activates a buzzer if values exceed a threshold.
#
# Note: This code works with DHT sensor v2.0 / v2.1, which has a BLACK plastic 
# cover. The sensor communicates via the I2C protocol (via I2C0, I2C1 plugs).
###############################################################################

# This script requires twi extra files:
# - lcd1602.py: Contains the class to control the LCD display
# - dht20.py: Contains the class for the DHT sensor
# Make sure to upload these files to your Raspberry Pi Pico!

# Import necessary modules and classes
from lcd1602 import LCD1602        # LCD display class
from dht20 import DHT20 as DHT     # DHT sensor class (version 2)
from machine import I2C, Pin, PWM  # Hardware/GPIO interfaces
from utime import sleep            # Used to pause the program

# Set up I2C communication for the LCD display
i2c0 = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)
d = LCD1602(i2c0, 2, 16)

# Set up I2C communication for the DHT sensor
i2c1 = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)
sensor = DHT(i2c1)
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

    # TASK: Add a condition to activate the buzzer when the
    # temperature exceeds 28°C. Note: The buzzer is controlled
    # using PWM. You can set its volume using buzzer.duty_u16().
    # A higher duty cycle means a louder sound. Don't overdo it!
    # A value of 200 is already quite loud.
