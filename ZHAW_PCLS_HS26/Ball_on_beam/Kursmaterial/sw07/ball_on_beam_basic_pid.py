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
#
# Idee: PID-Regler = P + I + D
#   P: reagiert auf aktuelle Abweichung       (wo ist der Ball?)
#   I: reagiert auf aufsummierte Abweichung   (wie lange liegt er schon daneben?)
#   D: reagiert auf Änderung der Abweichung   (wie schnell rollt er?)
###############################################################################

# Pin: GPIO-Pins ansprechen, I2C: Bus für den Sensor,
# PWM: Pulsweitenmodulation für den Servo
from machine import Pin, I2C, PWM
# Treiber für den Time-of-Flight-Distanzsensor (Datei vl53l0x.py muss auf dem Pico liegen)
from vl53l0x import VL53L0X
# ticks_us(): Zeitstempel in Mikrosekunden, ticks_diff(): Differenz zweier Zeitstempel
from utime import ticks_us, ticks_diff

def setAngle(angle, pwm, angle_max=30):
    """
    Function to convert an angle (in degree) into a PWM duty.
    For our servo, a duty of 1000 to 9000 corresponds to -90 to 90 degrees.

    Umrechnung Winkel -> PWM-Duty:
      duty_u16 geht von 0 (immer aus) bis 65535 (immer an).
      Bei 50 Hz dauert eine Periode 20 ms. Der Servo liest die Länge
      des Pulses: 1000 ≈ 0.3 ms (-90°), 5000 ≈ 1.5 ms (0°), 9000 ≈ 2.7 ms (+90°).
    """
    assert pwm.freq() == 50  # Formel gilt nur bei 50 Hz, sonst Programmabbruch
    # Winkel auf ±angle_max begrenzen, schützt die Mechanik
    angle = max(-angle_max, min(angle_max, angle))
    # Lineare Abbildung: -90° -> 1000, 0° -> 5000, +90° -> 9000
    position = int((angle + 90)/180*8000 + 1000)
    pwm.duty_u16(position)   # Puls an den Servo schicken


###############################################################################

# Set up connections
# I2C-Bus 0 an Pins 8 (SDA = Daten) und 9 (SCL = Takt): hier hängt der TOF-Sensor
i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))
# I2C-Bus 1: wird hier nicht benutzt (z.B. für ein LCD gedacht)
i2c1 = I2C(id=1, scl=Pin(7), sda=Pin(6))

# Set up distance sensor
# Sensor-Objekt erstellen; misst Distanz Sensor -> Ball in mm
tof = VL53L0X(i2c0)

# Set up the servo motor
servo = PWM(Pin(16))  # Servo at output D16
servo.freq(50)        # 50 PWM frequency (don't change!) -> Standard für Servos


# Sollwert: Ball soll 200 mm vom Sensor entfernt stehen
x_set = 200

# PID-Parameter (Startwerte aus PID-Regelung.md, müssen getunt werden)
# Tuning: erst ki = kd = 0, kp einstellen, dann kd, zuletzt ki
# Rollt Ball weg statt hin -> Vorzeichen aller drei umdrehen
kp = 0.1    # Proportional: vorher fix 0.1 im P-Regler
ki = 0.005  # Integral
kd = 0.15   # Differential

# Startwerte, die von Durchlauf zu Durchlauf gemerkt werden
integral = 0.0                      # Summe der Fehler über die Zeit
e_prev = float(x_set - tof.ping())  # erster Fehler schon vor der Schleife messen,
                                    # sonst gibt D im 1. Durchlauf einen Riesensprung
t_prev = ticks_us()                 # Zeitpunkt dieser ersten Messung


# Regelschleife: läuft endlos, jeder Durchlauf = messen, vergleichen, stellen
while True:

    # Measure new sensor data
    # Istwert: aktuelle Distanz zum Ball in mm (Messung dauert ca. 30 ms)
    x_new = tof.ping()

    # Zeit seit letzter Messung in Sekunden (µs / 1e6)
    # Echt gemessen statt fix angenommen, weil ping() unterschiedlich lange dauert
    t_new = ticks_us()
    dt = ticks_diff(t_new, t_prev) / 1e6

    # Compare measured data with set point (x_set)
    # Regelabweichung e = Soll - Ist
    #   e > 0: Ball näher am Sensor als Ziel
    #   e < 0: Ball weiter weg als Ziel
    #   e = 0: Ball am Ziel
    e_new = float(x_set - x_new)

    # I-Anteil: Fehler * Zeit aufsummieren (Fläche unter der Fehlerkurve)
    # Bleibt Ball dauerhaft knapp neben Ziel, wächst integral -> schiebt nach
    integral += e_new*dt

    # D-Anteil: Änderung des Fehlers pro Sekunde = Geschwindigkeit des Balls
    # Rollt Ball schnell aufs Ziel zu -> bremst schon vorher (dämpft Pendeln)
    derivative = (e_new - e_prev)/dt

    # PID-Regler: Stellgrösse u = P + I + D
    u_new = kp*e_new + ki*integral + kd*derivative

    # Schiene kippen (setAngle begrenzt auf ±30°)
    setAngle(u_new, servo)

    # Werte ausgeben: Komma-getrennt -> Thonny-Plotter zeichnet beide Kurven
    print(f"{e_new},{u_new}")

    # Aktuelle Werte werden im nächsten Durchlauf zu den "vorherigen"
    e_prev = e_new
    t_prev = t_new
