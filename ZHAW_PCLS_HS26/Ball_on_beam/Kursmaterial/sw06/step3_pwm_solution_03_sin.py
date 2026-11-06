###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################

# There are many ways how to realize a smooth blinking.
# Alternative 3: Use a sin-wave as a function.

# This demo uses PWM (pulse width modulation) to fade an LED on and off.
# Unfortunately this demo does not work for the onboard LED of the Pico W.
# But you can use the LED socket connected to one of the digital ports.

from machine import Pin, PWM
import utime
import math

# Parameters
freq = 1.0              # Blink frequency, in Hz
period = 0.001          # Also affects how quickly the LED flashes

# Constants
DUTY_MAX = 2**16-1      # Maximal PWM duty (LED brightest)
DUTY_MIN = 0            # Minimal PWM duty (LED off)

# Instantiate the LED and the PWM wrapper
led = Pin(18, Pin.OUT)  # LED at digital output D18
pwm = PWM(led)          # Use PWM for that LED
pwm.freq(1000)          # PWM frequency in Hz 

# Action!
t0 = utime.ticks_us()
while True:
    t = utime.ticks_us()
    dt = utime.ticks_diff(t, t0) / 1e6

    duty = math.sin(dt*freq*2*math.pi)  # Sine with values between -1 and +1
    duty = 0.5*(duty+1)                 # Sine with values between 0 and +1
    # duty = duty**4                    # Optional: modify the shape of the sine
    duty = DUTY_MAX*duty                # Sine with values between 0 and 2^16-1
    pwm.duty_u16(int(duty))             # Set the duty as 16-bit unsigned integer
    utime.sleep(period)

    print(int(duty))
