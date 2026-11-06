from machine import Pin
from utime import sleep

# Create the LED object
led = Pin("LED", mode=Pin.OUT)

# Initialize GPIO pin 18 as an input for the button
button = Pin(18, Pin.IN)

# Infinite loop to toggle the LED when the button is pressed
while True:
    if button.value():
        led.toggle()
        sleep(0.3)
