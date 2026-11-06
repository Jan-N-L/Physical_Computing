###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################


###############################################################################
# Basic control loop, without signal smoothing
###############################################################################
# 
# Setup:
#   - I2C0: TOF sensor
#   - D16:  Servo motor


#################################################
# SETTINGS:
#################################################

# Target position (in mm)
x_set = 200

# PID controller parameters.
# You need to tune these!
k_p = 0.1
k_i = 0.1
k_d = 0.1

# Print values to console.
# Disable for best performance.
debug = True
#################################################


##############################################################################
# Imports
##############################################################################
from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
import utime

##############################################################################
# Utilities
##############################################################################
def set_angle(angle, pwm, angle_max=30):
    """
    This is the function we use to convert an angle (in degrees) into the
    corresponding PWM duty.

    For our servo, a duty of 1000 to 9000 corresponds to -90 to 90 degrees.
    Here, we converts an angle (in degrees) into PWM duty and set the servo

    You may have to tune the numbers below to calibrate to your system.
    However, it should roughly match specific system.

    Parameters
    ----------
    angle : Desired angle of the servomotor in degrees
    pwm : The object we use to address the motor's PWM pin
    angle_max : Can be used to prescribe maximal action
    """
    if angle_max is not None:
        # -angle_max <= angle <= angle_max
        angle = max(angle, -angle_max)
        angle = min(angle, angle_max)
    # The formula below is valid only for PWM frequency 50.
    # With an assertion, we can ensure that this is the case.
    # (It will throw an error if the condition is not met.)
    assert pwm.freq() == 50
    position = int((angle + 90)/180*8000 + 1000)
    pwm.duty_u16(position)
    
##############################################################################

# Setup connections
i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))
i2c1 = I2C(id=1, scl=Pin(7), sda=Pin(6))

tof = VL53L0X(i2c0)

pwm = PWM(Pin(16))  # Servo at output D16
pwm.freq(50)        # 50 PWM frequency (don't change!)

# Intialization
t_last = None
e_last = None
integral = 0

while True:
    # Measure new sensor data
    x_new = tof.ping()

    # Measure current timestamp (in µs)
    t_new = utime.ticks_us()

    # Compute current error
    e_new = float(x_set - x_new)

    # Controller update
    if e_last is not None:
        # We come here only after the first loop.

        # Important: Use ticks_diff() to compute differences of tick counts!
        # Details: https://docs.micropython.org/en/latest/library/time.html
        # Divide the time difference by a million to convert into seconds.
        dt = utime.ticks_diff(t_new, t_last) / 1e6

        # Update the integral of the error
        integral += e_new * dt

        # Compute the rate at which the error is changing
        derivative = (e_new - e_last) / dt

        # Given e(t), compute the control signal u(t): the angle of the
        # servomotor. This implements our PID-controller!
        u_new = k_p * e_new + k_i * integral + k_d * derivative
        set_angle(angle=u_new, pwm=pwm)

        # Useful for debugging
        if debug:
            print("e(t)=%7.1f, Ie(t)=%7.1f, de(t)=%7.1f, u(t)=%6.1f" % 
                  (e_new, integral, derivative, u_new))

    # New becomes old
    t_last = t_new
    e_last = e_new
    utime.sleep(0.01)
    
