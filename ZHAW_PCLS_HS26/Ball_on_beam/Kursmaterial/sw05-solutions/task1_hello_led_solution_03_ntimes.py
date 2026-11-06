from machine import Pin
from utime import sleep

# Initialize the onboard LED as an output pin
led = Pin("LED", mode=Pin.OUT)

# How many times to blink?
n = 5

# Start blinking
for i in range(5):
    led.on()
    sleep(1)
    led.off()
    sleep(1)
   
print("Done!")
