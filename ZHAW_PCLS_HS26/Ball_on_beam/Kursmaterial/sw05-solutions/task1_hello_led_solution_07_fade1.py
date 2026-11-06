from machine import Pin, PWM
from utime import sleep

# This demo uses PWM to smoothly fade an LED on and off.
# Unfortunately, this won't work with the onboard LED of the Pico W.
# Use the LED socket connected to one of the digital pins (e.g., D16).

# Set up the pin as an output and create a PWM object for it
pin = Pin(16, Pin.OUT)
pwm = PWM(pin)

# Set the PWM frequency to 1000 Hz (how fast the signal switches)
pwm.freq(1000)

# Define the brightness range for the LED
DUTY_MAX = 2**16-1   # Maximum brightness (PWM duty cycle)
DUTY_MIN = 0         # LED off (minimum duty cycle)

# Initialize variables for fading logic
duty = 0             # Current brightness level
direction = 1        # Direction of change: 1 = brighter, -1 = dimmer
step = 256           # Amount to change brightness each step

# Loop to gradually increase and decrease LED brightness
while True:
    # Update brightness
    duty += direction*step
    if duty > DUTY_MAX:
        # If too bright, reverse direction
        duty = DUTY_MAX
        direction = -1
    elif duty < DUTY_MIN:
        # If too dim, reverse direction
        duty = DUTY_MIN
        direction = 1
    pwm.duty_u16(duty)
    sleep(0.001)
