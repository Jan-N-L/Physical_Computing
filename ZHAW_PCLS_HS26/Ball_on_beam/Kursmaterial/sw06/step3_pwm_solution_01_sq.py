###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################

# There are many ways how to realize a smooth blinking.
# Alternative 1: Incrementally increase the duty (squared version x^2).

# This demo uses PWM (pulse width modulation) to fade an LED on and off.
# Unfortunately this demo does not work for the onboard LED of the Pico W.
# But you can use the LED socket connected to one of the digital ports.

from machine import Pin, PWM
from utime import sleep

# Construct PWM object, with LED socket kit attached on D16
pin = Pin(16, Pin.OUT)
pwm = PWM(pin)
pwm.freq(1000)

# Initializations
duty = 0
direction = 1

# Action!
while True:
    duty = duty + direction
    if duty > 255:
        duty = 255
        direction = -1
    elif duty < 0:
        duty = 0
        direction = 1
    pwm.duty_u16(duty * duty)
    sleep(0.001)
