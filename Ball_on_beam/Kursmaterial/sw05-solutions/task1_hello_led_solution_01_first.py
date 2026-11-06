from machine import Pin
from utime import sleep

# Initialize the onboard LED as an output pin
led = Pin("LED", mode=Pin.OUT)

# Infinite loop to toggle the LED
while True:
   led.toggle()
   sleep(0.5)

# Note the KeyboardInterrupt error at the end.