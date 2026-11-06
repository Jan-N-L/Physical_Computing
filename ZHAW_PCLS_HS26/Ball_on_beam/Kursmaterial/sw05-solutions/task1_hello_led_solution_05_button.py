from machine import Pin
from utime import sleep

# Initialize the onboard LED as an output pin
led = Pin("LED", mode=Pin.OUT)

# Initialize GPIO pin 18 as an input for the button
button = Pin(18, Pin.IN)

# An infinit loop
while True:
    if button.value():
        led.on()
    else:
        led.off()
    sleep(0.01)
   
