from machine import Pin, PWM
from utime import sleep

# Test the servo motor

pwm = PWM(Pin(16))      # Servo at output D16
pwm.freq(50)            # Set the PWM frequency

def set_angle(angle):
    # For our servo, a duty of 1000 to 9000 corresponds to -90 to 90 degrees
    # Here, we convert an angle (in degrees) into a PWM duty and set the servo
    # Note: the formula is valid only for PWM frequency of 50.
    position = int((-angle + 90)/180*8000 + 1000)
    pwm.duty_u16(position)

# Let's run the servo from -30 to +30 degrees
for angle in range(-30,30):
    set_angle(angle)
    sleep(0.1)

set_angle(0)
    

