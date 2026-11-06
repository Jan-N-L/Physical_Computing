import time
from machine import Pin, PWM
from math import sin, pi

# This demo uses PWM to smoothly fade an LED on and off.
# Unfortunately, this won't work with the onboard LED of the Pico W.
# Use the LED socket connected to one of the digital pins (e.g., D16).

# Set up the pin as an output and create a PWM object for it
pin = Pin(16, Pin.OUT)
pwm = PWM(pin)

# Set the PWM frequency to 1000 Hz (how fast the signal switches)
pwm.freq(1000)

# Define the brightness range and frequency for the LED
DUTY_MAX = 2**16-1   # Maximum brightness (PWM duty cycle)
DUTY_MIN = 0         # LED off (minimum duty cycle)
blink_freq = 5       # Blinking frequency (in Hz)

# Use sinus function to compute the blinking
start = time.ticks_us()
while True:
    t = time.ticks_diff(time.ticks_us(), start) / 1e6
    # duty = A/2·(sin(1/(2πf)·t) + 1)
    duty = DUTY_MAX/2*(sin(t*2*pi*blink_freq) + 1)
    pwm.duty_u16(int(duty))
    time.sleep(0.001)
