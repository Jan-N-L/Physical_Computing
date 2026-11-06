###############################################################################
# PCLS / Physical Computing in Life Sciences
# Norman Juchler
###############################################################################

###############################################################################
# Ball on beam: optimierter PID-Regler
###############################################################################
#
# Setup:
#   - I2C0: TOF sensor
#   - D16:  Servo motor
#
# Neu gegenüber ball_on_beam_basic_pid.py:
#   1. Filter:       Sensorrauschen glätten, sonst zittert der D-Anteil
#   2. Ausreisser:   Messwerte ausserhalb 0..400 mm abschneiden
#   3. Grob/Fein:    kp wächst mit der Abweichung (weit weg = heftig, nah = fein)
#   4. Sollbereich:  Ball ruhig innerhalb ±7.5 mm -> Schiene halten, Servo steht still
#   5. Störung:      Rollt Ball im Sollbereich los (z.B. Ebene gekippt),
#                    greift der Regler sofort wieder ein
#   6. Anti-Windup:  integral nur nahe am Ziel und begrenzt
###############################################################################

# Pin: GPIO-Pins ansprechen, I2C: Bus für den Sensor,
# PWM: Pulsweitenmodulation für den Servo
from machine import Pin, I2C, PWM
# Treiber für den Time-of-Flight-Distanzsensor (Datei vl53l0x.py muss auf dem Pico liegen)
from vl53l0x import VL53L0X
# ticks_us(): Zeitstempel in Mikrosekunden, ticks_diff(): Differenz zweier Zeitstempel
from utime import ticks_us, ticks_diff

# Servo-Winkel, bei dem die Schiene waagrecht ist (Kalibrierung).
# Ball bei angle=0 hinlegen: rollt er weg -> Wert anpassen, bis er liegen bleibt.
# Spart dem I-Anteil das Lernen beim Start.
ANGLE_OFFSET = 0

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
    angle = max(-angle_max, min(angle_max, angle + ANGLE_OFFSET))
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

# Sollbereich ±0.75 cm: Ball ruhig darin -> Regler hält still
TOL = 7.5       # mm
# Darunter gilt der Ball als "ruhig". Muss knapp über dem Rauschen liegen:
# Ball ruhig hinlegen, 2. Wert im Plotter (Geschwindigkeit) anschauen,
# V_TOL etwas grösser als dieses Zappeln wählen.
V_TOL = 15      # mm/s

# P-Anteil grob/fein: bei e = 0 gilt KP_FINE, ab |e| >= E_BIG gilt KP_COARSE,
# dazwischen linear überblendet (kein Sprung beim Umschalten)
KP_FINE = 0.05    # °/mm  nahe am Ziel: sanft
KP_COARSE = 0.15  # °/mm  weit weg: heftig
E_BIG = 50        # mm    ab hier gilt "weit weg"

ki = 0.05   # °/(mm*s)  Integral: lernt Schräglage (z.B. gekippte Ebene)
kd = 0.06   # °/(mm/s)  Differential: bremst den Ball. Zu gross -> Servo zittert
INTEGRAL_MAX = 200  # mm*s  Anti-Windup: I-Anteil höchstens ki*200 = 10°

# Filter (1 = kein Filter, kleiner = glatter aber träger)
ALPHA = 0.4  # für die Position (geht in P, I und D)
BETA = 0.2   # für die Geschwindigkeit, nur für die Ruhe-Erkennung

# Startwerte, die von Durchlauf zu Durchlauf gemerkt werden
x_f = tof.ping()             # gefilterte Position, Start = erste Messung
e_prev = float(x_set - x_f)  # vorheriger Fehler (für D)
t_prev = ticks_us()          # Zeitpunkt der vorherigen Messung
integral = 0.0               # Summe der Fehler über die Zeit
v_f = 0.0                    # stark geglättete Geschwindigkeit
hold = False                 # True = Ball ruhig im Sollbereich, Servo hält


# Regelschleife: läuft endlos, jeder Durchlauf = messen, vergleichen, stellen
while True:

    # Messen; Ausreisser abschneiden (ohne Ziel liefert der Sensor z.B. 8190)
    x_new = min(max(tof.ping(), 0), 400)

    # Zeit seit letzter Messung in Sekunden
    t_new = ticks_us()
    dt = ticks_diff(t_new, t_prev) / 1e6

    # Exponentieller Filter: neue Messung zählt ALPHA, bisheriger Wert 1-ALPHA
    x_f = ALPHA*x_new + (1 - ALPHA)*x_f

    # Regelabweichung e = Soll - Ist (gefiltert)
    e_new = float(x_set - x_f)

    # Änderung des Fehlers pro Sekunde = Geschwindigkeit des Balls (mm/s)
    derivative = (e_new - e_prev)/dt
    # Nochmals geglättet: zum Entscheiden "ruhig oder nicht" reicht träge,
    # dafür springt hold nicht wegen Rauschen hin und her
    v_f = BETA*derivative + (1 - BETA)*v_f

    # Ball ruhig im Sollbereich?
    # Hysterese: zum Verlassen doppelte Geschwindigkeit nötig wie zum Eintreten
    v_limit = 2*V_TOL if hold else V_TOL
    hold = abs(e_new) < TOL and abs(v_f) < v_limit

    if hold:
        # Nur I-Anteil = gelernte "waagrechte" Stellung.
        # integral wird nicht verändert -> u bleibt gleich -> Servo steht still.
        # Rollt der Ball los (Ebene gekippt), wird v_f gross -> hold endet
        # sofort, noch bevor der Ball den Sollbereich verlässt.
        u_new = ki*integral
    else:
        # Grob/Fein: kp zwischen KP_FINE und KP_COARSE je nach Abweichung
        kp = KP_FINE + (KP_COARSE - KP_FINE)*min(abs(e_new)/E_BIG, 1)

        # I nur nahe am Ziel aufsummieren (weit weg erledigt das P) und begrenzen
        if abs(e_new) < E_BIG:
            integral += e_new*dt
            integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral))

        # PID-Regler: Stellgrösse u = P + I + D
        u_new = kp*e_new + ki*integral + kd*derivative

    # Schiene kippen (setAngle begrenzt auf ±30°)
    setAngle(u_new, servo)

    # Plotter: Abweichung, Geschwindigkeit, Winkel, hold (0/10)
    print(f"{e_new:.1f},{v_f:.0f},{u_new:.1f},{10*hold}")

    # Aktuelle Werte werden im nächsten Durchlauf zu den "vorherigen"
    e_prev = e_new
    t_prev = t_new
