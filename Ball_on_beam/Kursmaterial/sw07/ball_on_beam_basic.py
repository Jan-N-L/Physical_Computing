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
#
# Idee: Ein P-Regler (nur Proportional-Anteil).
#   Sensor misst Ballposition -> Abweichung vom Ziel berechnen
#   -> Schiene proportional zur Abweichung kippen -> Ball rollt -> neu messen.
###############################################################################

# Pin: GPIO-Pins ansprechen, I2C: Bus für den Sensor,
# PWM: Pulsweitenmodulation für den Servo
from machine import Pin, I2C, PWM
# Treiber für den Time-of-Flight-Distanzsensor (Datei vl53l0x.py muss auf dem Pico liegen)
from vl53l0x import VL53L0X
# sleep(s): Programm für s Sekunden anhalten
from utime import sleep

def setAngle(angle, pwm):
    """
    Function to convert an angle (in degree) into a PWM duty.
    For our servo, a duty of 1000 to 9000 corresponds to -90 to 90 degrees.

    Umrechnung Winkel -> PWM-Duty:
      duty_u16 geht von 0 (immer aus) bis 65535 (immer an).
      Bei 50 Hz dauert eine Periode 20 ms. Der Servo liest die Länge
      des Pulses: 1000 ≈ 0.3 ms (-90°), 5000 ≈ 1.5 ms (0°), 9000 ≈ 2.7 ms (+90°).
    """
    assert pwm.freq() == 50  # Formel gilt nur bei 50 Hz, sonst Programmabbruch
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


# Regelschleife: läuft endlos, jeder Durchlauf = messen, vergleichen, stellen
while True:

    # Measure new sensor data
    # Istwert: aktuelle Distanz zum Ball in mm (Messung dauert ca. 30 ms)
    x_new = tof.ping()

    # Compare measured data with set point (x_set)
    # Regelabweichung e = Soll - Ist
    #   e > 0: Ball näher am Sensor als Ziel
    #   e < 0: Ball weiter weg als Ziel
    #   e = 0: Ball am Ziel
    e_new = float(x_set - x_new)

       # P-Regler: Stellgrösse u = Kp * e mit Kp = 0.1
    # z.B. Ball 50 mm zu nah -> e = 50 -> Schiene 5° kippen
    # Vorzeichen bestimmt Kipprichtung: rollt Ball weg statt hin -> 0.1 durch -0.1 ersetzen
    u_new = 0.1*e_new

    # Achtung: keine Begrenzung, bei grossem e kippt Servo bis ±90°
    setAngle(u_new, servo)

    # Werte ausgeben: Komma-getrennt -> Thonny-Plotter zeichnet beide Kurven
    print(f"{e_new},{u_new}")

    # Wait a bit
    # 10 ms Pause; zusammen mit Messung -> ca. 40 ms pro Durchlauf
    sleep(0.01)
