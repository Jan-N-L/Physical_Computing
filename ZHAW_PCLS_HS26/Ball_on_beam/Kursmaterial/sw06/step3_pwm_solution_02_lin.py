###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################

# There are many ways how to realize a smooth blinking.
# Alternative 2: Incrementally increase the duty (linear version).

# This demo uses PWM (pulse width modulation) to fade an LED on and off.
# Unfortunately this demo does not work for the onboard LED of the Pico W.
# But you can use the LED socket connected to one of the digital ports.

from machine import Pin, PWM
from utime import sleep

# Parameters
step = 128              # Affects how quickly the LED flashes
period = 0.001          # Also affects how quickly the LED flashes

# Constants
DUTY_MAX = 2**16-1      # Maximal PWM duty (LED brightest)
DUTY_MIN = 0            # Minimal PWM duty (LED off)

# Instantiate the LED and the PWM wrapper
led = Pin(18, Pin.OUT)  # LED at digital output D18
pwm = PWM(led)          # Use PWM for that LED
pwm.freq(1000)          # PWM frequency in Hz 

# Action!
while True:
    for i in range(DUTY_MIN, DUTY_MAX+1, step):
        print(i)
        pwm.duty_u16(i)
        sleep(sleep)
    for i in range(DUTY_MAX, DUTY_MIN, -step):
        print(i)
        pwm.duty_u16(i)
        sleep(period)

