###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################


###############################################################################
# Basic control loop, with signal smoothing
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

# Parameter for exponential
# smoothing
alpha = 0.1

# Width of filter window
# Must be larger than 2
width = 10

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
def exp_smoothing(x_new, x_s, alpha=0.5):
    """
    This implements exponential smoothing. The smoothed value x_s is computed
    from the current measurement x_new by mixing it with the last value x_s

    Parameters
    ----------
    x_new :     New measurement
    x_s :       Smoothed measurement
    alpha :     Blending value

    Returns
    -------
    The new smoothed value x_s
    """
    if x_s is None:
        # We come here for the first iteration
        return x_new
    x_s_new = x_new * alpha + (1-alpha) * x_s
    return x_s_new


def moving_average(x_array):
    """
    This computes the mean of the values contained in the array:
    x_s = mean(x_array).

    Parameters
    ----------
    x_new : A list of sensor measurements of length n with measurements

    Returns
    -------
    The new smoothed value x_s

    Note
    ----
    If the array contains any None values, the function returns the most
    recent value of the input x_array[0], which is guaranteed to be available.
    This occurs for the first n iterations.
    """
    try:
        return sum(x_array)/len(x_array)
    except TypeError:
        # See note.
        return x_array[0]


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

# Initialize data arrays with None values
# Creates [ None, None, None, None, None ]
t_array = [ None ] * width
x_array = [ None ] * width
x_smooth = [ None ] * width

# Exponential smoothing initialization
x_exp = None

# Moving average initialization
x_ma = None

# Other initializations
t_last = None
integral = 0
step = 0

# Setup connections
i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))
i2c1 = I2C(id=1, scl=Pin(7), sda=Pin(6))

tof = VL53L0X(i2c0)
pwm = PWM(Pin(16))  # Servo at output D16
pwm.freq(50)        # 50 PWM frequency (don't change!)

# Control loop
while True:
    step += 1

    # Measure new data
    x_new = tof.ping()
    t_new = utime.ticks_us()

    # Update data arrays
    t_array[1:] = t_array[:-1]      # Move all entries by one
    x_array[1:] = x_array[:-1]      # step to the right...
    x_smooth[1:] = x_smooth[:-1]    # [ 6, 5, 4, 3, 2, 1 ] becomes
                                    # [ 6, 6, 5, 4, 3, 2 ]

    t_array[0] = t_new              # The NEWEST value is always on the
    x_array[0] = x_new              # LEFT of the array (at position 0)

    # Choose here which SMOOTHING METHOD to use
    x_exp = exp_smoothing(x_array[0], x_exp, alpha=alpha)
    x_ma = moving_average(x_array)

    #x_smooth[0] = x_ma
    x_smooth[0] = x_exp

    if step >= width:

        # Important: Use ticks_diff() to compute differences of tick counts!
        # Details: https://docs.micropython.org/en/latest/library/time.html
        # Divide the time difference by a million to convert into seconds.
        dt = utime.ticks_diff(t_array[0], t_array[1]) / 1e6

        # Compute current error
        e_new = x_set - x_smooth[0]

        # Update the integral of the error
        integral += e_new * dt
        # Compute the rate at which the error is changing
        # Note: e_new - e_last = x_smooth[0] - x_smooth[1]
        derivative = (x_smooth[0] - x_smooth[1]) / dt

        # Given e(t), compute the control signal u(t): the angle of the
        # servomotor. This implements our PID-controller!
        u_new = k_p * e_new + k_i * integral + k_d * derivative
        set_angle(angle=u_new, pwm=pwm)

        # Useful for debugging
        if debug:
            print("e(t)=%7.1f, Ie(t)=%7.1f, de(t)=%7.1f, u(t)=%6.1f, x(t)=%6.1f, xs(t)=%6.1f," %
                  (e_new, integral, derivative, u_new, x_array[0], x_smooth[0]))

    utime.sleep(0.01)

