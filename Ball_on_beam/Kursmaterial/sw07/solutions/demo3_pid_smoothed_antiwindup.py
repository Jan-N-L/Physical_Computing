###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################


###############################################################################
# Control loop, WITH smoothing and ANTI WINDUP and ADJUSTABLE SETPOINT
###############################################################################
# Here, we will use a so-called potentiometer to adjust the target position.
# Also, we display the current position on the LCD display. If the ball remains
# within a certain range for a certain duration, the controller will 
# automatically switch off.
# 
# Setup: 
#   - I2C0: TOF sensor 
#   - I2C1: LCD display
#   - D16:  Servo motor
#   - A0:   Potentiometer (Rotary angle sensor)


#################################################
# SETTINGS:
#################################################

# Target position (in mm). In this demo, we will 
# set the exact value with the potentiometer. The 
# parameters here define the range for the target 
# value (in mm).
x_set_range = [100, 300]

# PID controller parameters.
# You need to tune these!
k_p = 0.1
k_i = 0.1
k_d = 0.1

# Smoothing method
# - "ma": moving average
# - "exp": exponential smoothing
# - None: no smoothing
smoothing = "exp"

# Parameter for exponential smoothing
alpha = 0.9

# Width of filter window
# Must be larger than 2
width = 20

# Print values to console.
# Disable for best performance.
debug = True

# For reasons not entirely clear, the TOF sensor 
# sometimes and suddenly returns unreasonably 
# large values, and keeps doing so for a while 
# before returning to normal. Obviously, this
# will destabilize the controller. By clamping
# the sensor value, we can easily limit the 
# effect of these outliers.
x_value_range = [0, 400]  # in mm

# Maximal integral (anti windup parameter)
# (set False to disable)
integral_max = 200

# PWM frequency for the servo motor.
# Reasonable range: [50 - 400]
pwm_freq = 10

# Auto-switch off if ball remains within this
# range (in mm) for this duration (in seconds).
auto_off = True
auto_tol = 12
auto_dur = 3
#################################################


##############################################################################
# Imports
##############################################################################
from machine import Pin, I2C, PWM, ADC
from vl53l0x import VL53L0X  # TOF sensor
from lcd1602 import LCD1602  # LCD display
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
    

def get_setpoint(adc, val_range):
    """
    This maps the 16-bit value read from the ADC pin, an integer in the range 
    of [0, 2^16-1], to the specified value range.
    """
    x = adc.read_u16()
    x_min, x_max = 0, 2**16-1
    y_min, y_max = val_range
    return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min


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
    offset = 2
    angle += offset
    if angle_max is not None:
        # -angle_max <= angle <= angle_max
        angle = max(angle, -angle_max)
        angle = min(angle, angle_max)
    # The formula below is valid only for PWM frequency 50.
    # We can adjust the formula for different PWM frequencies.
    factor = pwm.freq() / 50.
    position = factor*((angle + 90.)/180.*8000. + 1000.)
    position = int(position)
    pwm.duty_u16(position)
    
##############################################################################

# Initialize data arrays with None values
# Creates [ None, None, None, None, None ]
t_array = [ None ] * width
x_array = [ None ] * width
x_smooth = [ None ] * width

# Exponential smoothing initialization
x_exp = None

# Auto-switch off initialization
t_auto = None
u_auto = 0.
n_auto = 0.

# Other initializations
t_last = None
integral = 0
step = 0

# Setup connections
i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))
i2c1 = I2C(id=1, scl=Pin(7), sda=Pin(6))

pwm = PWM(Pin(16))          # Servo at output D16
pwm.freq(pwm_freq)          # PWM frequency (adjustable)
poti = ADC(0)               # Potentiometer at input A0
tof = VL53L0X(i2c1)         # TOF sensor
lcd = LCD1602(i2c0, 2, 16)  # LCD display

# We can adjust the timings of the TOF sensor to optimize performance.
# For hackers: 
tof.set_measurement_timing_budget(80000)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 8)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 8)

# Control loop
while True:
    step += 1

    # Measure new data
    x_new = tof.ping()
    x_new = min(max(x_new, x_value_range[0]), x_value_range[1])
    t_new = utime.ticks_us()
    x_set = get_setpoint(poti, val_range=x_set_range)

    # Update data arrays
    t_array[1:] = t_array[:-1]      # Move all entries by one
    x_array[1:] = x_array[:-1]      # step to the right...
    x_smooth[1:] = x_smooth[:-1]    # [ 6, 5, 4, 3, 2, 1 ] becomes
                                    # [ 6, 6, 5, 4, 3, 2 ]

    t_array[0] = t_new              # The NEWEST value is always on the
    x_array[0] = x_new              # LEFT of the array (at position 0)

    # Smoothing
    if smoothing == "exp":
        x_exp = exp_smoothing(x_array[0], x_exp, alpha=alpha)
        x_s = x_exp
    elif smoothing == "ma":
        x_s = moving_average(x_array)
    elif smoothing is None:
        x_s = x_array[0]
    else:
        msg = "This smoothing method does not exist: %s"
        raise ValueError(msg % smoothing)

    x_smooth[0] = x_s

    if step >= width:

        # Important: Use ticks_diff() to compute differences of tick counts!
        # Details: https://docs.micropython.org/en/latest/library/time.html
        # Divide the time difference by a million to convert into seconds.
        dt = utime.ticks_diff(t_array[0], t_array[1]) / 1e6

        # Compute current error
        e_new = x_set - x_smooth[0]

        # Update the integral of the error
        integral += e_new * dt
        # Anti windup
        if integral_max:
            integral = max(integral, -integral_max)
            integral = min(integral, integral_max)
        # Compute the rate at which the error is changing
        # Note: e_new - e_last = x_smooth[0] - x_smooth[1]
        derivative = (x_smooth[0] - x_smooth[1]) / dt

        # Given e(t), compute the control signal u(t): the angle of the
        # servomotor. This implements our PID-controller!
        u_new = k_p * e_new + k_i * integral + k_d * derivative

        # Auto-switch off
        if auto_off:
            if abs(e_new) < auto_tol:
                # Ball is within tolerance, start measuring time.
                if t_last is None:
                    t_last = t_new
                else:
                    if utime.ticks_diff(t_new, t_last) / 1e6 > auto_dur:
                        # Ball has been within tolerance for long enough.
                        # Set the average control signal.
                        u_new = u_auto / n_auto
                        integral = 0
                    else:
                        n_auto += 1
                        u_auto += u_new
            else:
                n_auto = 0
                u_auto = 0
                t_last = None

        u_set = u_new
        set_angle(angle=u_set, pwm=pwm)

        # Useful for debugging.
        if debug:
            lcd.setCursor(0,0)                    
            lcd.print("Setpoint: %3.1fmm" % x_set)      
            lcd.setCursor(0,1)                    
            lcd.print("Position: %3.1fmm" % x_smooth[0])

            freq = 1 / dt

            print("e(t)=%7.1f, Ie(t)=%7.1f, de(t)=%7.1f, u(t)=%6.1f, "
                  "x(t)=%6.1f, xs(t)=%6.1f, freq=%6.1f" %
                  (e_new, integral, derivative, u_set, 
                   x_array[0], x_smooth[0], freq))

    utime.sleep(0.01)

