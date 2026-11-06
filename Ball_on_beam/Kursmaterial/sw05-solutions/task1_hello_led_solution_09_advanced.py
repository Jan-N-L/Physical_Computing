from machine import Pin, Timer
from utime import sleep

# This is a more advanced solution that uses a timer object to 
# create the same blinking effect as in previous examples.

# Initialize the onboard LED as an output pin
led = Pin("LED", Pin.OUT)

# Create a timer object that can run code repeatedly at set intervals
timer = Timer()

# Define a function that will be called by the timer
# Each time it's called, it switches the LED on or off
def tick(timer):
    led.toggle()

# Start the timer:
# - freq=10:             runs the timer 10 times per second
# - mode=Timer.PERIODIC: repeats the action continuously at the set frequency
# - callback=tick:       calls the tick() function each time the timer triggers
timer.init(freq=10, 
           mode=Timer.PERIODIC, 
           callback=tick)

# Let the LED blink for 10 seconds
print("Blinking for 10s...")
sleep(10)
print("Done!")
