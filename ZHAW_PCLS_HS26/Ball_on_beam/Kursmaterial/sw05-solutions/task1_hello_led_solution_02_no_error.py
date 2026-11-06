from machine import Pin
from utime import sleep

# Initialize the onboard LED as an output pin
led = Pin("LED", mode=Pin.OUT)

# Infinite loop to toggle the LED
while True:
   try:
      led.toggle()
      sleep(0.5)
   except KeyboardInterrupt:
      # When stopping code execution, Visual Studio Code sends 
      # a KeyboardInterrupt error. To prevent an error message 
      # from appearing, we can catch the exception and exit the 
      # loop gracefully.
      break

# Ensure the LED is turned off when exiting.
led.off()
