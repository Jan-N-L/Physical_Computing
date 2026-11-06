from machine import Pin
from utime import sleep

# Initialize the onboard LED as an output pin
led = Pin("LED", mode=Pin.OUT)

# How many times to blink?
n = 5

# Durations in s?
long_duration = 0.5
short_duration = 0.1

# Start blinking (long duration)
for i in range(5):
    led.on()
    sleep(long_duration)
    led.off()
    sleep(long_duration)

# Start blinking (short duration)
for i in range(5):
    led.on()
    sleep(short_duration)
    led.off()
    sleep(short_duration)
   
print("Done!")
