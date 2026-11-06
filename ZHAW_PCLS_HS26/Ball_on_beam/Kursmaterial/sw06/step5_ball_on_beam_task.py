###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################

###############################################################################
# Ball on beam: Basic control loop, without signal smoothing
###############################################################################
# 
# Setup:
#   - I2C0: TOF sensor
#   - D16:  Servo motor
###############################################################################

from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import sleep

def setAngle(angle, pwm):
    """
    Function to convert an angle (in degree) into a PWM duty.
    For our servo, a duty of 1000 to 9000 corresponds to -90 to 90 degrees.
    """
    assert pwm.freq() == 50  # This code makes sense only for 50 Hz PWM
    position = int((angle + 90)/180*8000 + 1000)
    pwm.duty_u16(position)


###############################################################################

# Set up the distance sensor
...

# Set up the servo motor
...

# Specify the target position (in mm)
# That's where the ball should go.
x_set = 200

while True:

    # Measure data
    ...

    # Compare measured data with set point (x_set)
    # This is equivalent to: State feedback!
    ...

    # Compute the new beam angle
    ...

    # Apply the new beam angle
    ...

    # Wait a bit
    sleep(0.01)
