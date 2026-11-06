###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################

###############################################################################
# Ball on beam: PID control loop, without signal smoothing
###############################################################################
#
# Setup:
#   - I2C0: TOF sensor
#   - D16:  Servo motor
###############################################################################

from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import sleep, ticks_us, ticks_diff

def setAngle(angle, pwm, angle_max=30):
    """
    Function to convert an angle (in degree) into a PWM duty.
    For our servo, a duty of 1000 to 9000 corresponds to -90 to 90 degrees.
    The angle is limited to +-angle_max to protect the beam.
    """
    assert pwm.freq() == 50  # This code makes sense only for 50 Hz PWM
    angle = max(-angle_max, min(angle_max, angle))
    position = int((angle + 90)/180*8000 + 1000)
    pwm.duty_u16(position)


###############################################################################

# Set up connections
i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))
i2c1 = I2C(id=1, scl=Pin(7), sda=Pin(6))

# Set up distance sensor
tof = VL53L0X(i2c0)

# Set up the servo motor
servo = PWM(Pin(16))  # Servo at output D16
servo.freq(50)        # 50 PWM frequency (don't change!)

# Specify the target position (in mm)
# That's where the ball should go.
x_set = 200

# PID controller parameters (tune these: start with ki = kd = 0)
kp = 0.1  # Proportional gain
ki = 0.01 # Integral gain
kd = 0.05 # Derivative gain

# Initialize PID controller variables.
# First measurement before the loop, so the D-term has no spike in loop 1.
integral = 0.0
e_prev = float(x_set - tof.ping())
t_prev = ticks_us()

print("e_new , u_new")

while True:

    # Measure new sensor data
    x_new = tof.ping()

    # Real loop time in seconds (tof.ping() alone takes ~30 ms,
    # so a fixed dt would be far off). Use ticks_diff() for tick counts!
    t_new = ticks_us()
    dt = ticks_diff(t_new, t_prev) / 1e6

    # Compare measured data with set point (x_set)
    e_new = float(x_set - x_new)

    # Update integral and derivative terms
    integral += e_new*dt
    derivative = (e_new - e_prev)/dt

    # Compute the new control signal (angle)
    u_new = kp*e_new + ki*integral + kd*derivative

    # Apply the new beam angle
    setAngle(u_new, servo)

    print(f"{e_new},{u_new}")

    # Update previous values
    e_prev = e_new
    t_prev = t_new

    # Wait a bit
    sleep(0.01)
