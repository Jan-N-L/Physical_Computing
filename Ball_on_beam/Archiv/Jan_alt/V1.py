
from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import ticks_us, ticks_diff

# Servo-Winkel, bei dem die Schiene waagrecht ist (Kalibrierung).
# Ball bei angle=0 hinlegen: rollt er weg -> Wert anpassen, bis er liegen bleibt.
# Spart dem I-Anteil das Lernen beim Start.
# Test: Programm stoppen, in der REPL ANGLE_OFFSET = 3 und setAngle(0, servo)
# eingeben, Ball links und rechts vom Ziel hinlegen.
# Geschätzt aus 2 Läufen: vor dem Ziel lag der Ball bei +9° still, dahinter
# bei -3° -> Mitte +3°. 12° Unterschied = Spiel im Gestänge oder Schiene geknickt
ANGLE_OFFSET = 0.5

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

i2c0 = I2C(id=0, scl=Pin(9), sda=Pin(8))
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
# PLOT = 1, Ball ruhig hinlegen, Kurve "de" anschauen,
# V_TOL etwas grösser als dieses Zappeln wählen.
V_TOL = 15      # mm/s

# P-Anteil grob/fein: bei e = 0 gilt KP_FINE, ab |e| >= E_BIG gilt KP_COARSE,
# dazwischen linear überblendet (kein Sprung beim Umschalten)
KP_FINE = 0.10    # °/mm  nahe am Ziel. Ball kriecht ans Ziel -> erhöhen
KP_COARSE = 0.15  # °/mm  weit weg: heftig
E_BIG = 50        # mm    ab hier gilt "weit weg"

ki = 0.02   # °/(mm*s)  Integral: lernt Schräglage. Zu gross -> überschiesst
kd = 0.08   # °/(mm/s)  Differential: bremst den Ball.
            # Ball rollt weit übers Ziel, pendelt langsam aus -> erhöhen.
            # Schnelles Schwingen, Kurve D viel grösser als P -> senken
            # (Servo + Sensor reagieren verzögert, zu viel D schaukelt auf)
I_MAX = 10  # °  Anti-Windup: I-Anteil höchstens ±10°

# Haftreibung: Ball rollt erst ab ca. ±5° los, dann aber leicht (Log 30.10.).
# Ohne Hilfe wartet der Regler, bis I genug aufgebaut hat -> Ball reisst los,
# rollt übers Ziel, klebt auf der anderen Seite (Stick-Slip, ~20 s Periode).
# FC = fixe Zusatz-Neigung Richtung Ziel, solange Ball ausserhalb TOL.
# Zu gross -> Ball pendelt ums Ziel. 0 = aus.
FC = 2      # °

# Holt-Filter (1 = kein Filter, kleiner = glatter).
# Dank Vorhersage hinkt die Position auch bei kleinem ALPHA nicht nach.
# Zu klein -> Filter "überschiesst" bei plötzlichem Stoss. Tunen mit PLOT = 1.
ALPHA = 0.3  # für die Position (geht in P und I)
BETA = 0.35  # für die Geschwindigkeit (geht in D und Ruhe-Erkennung)
             # kleiner = ruhigerer D-Anteil, bremst aber zu spät -> Schwingen

# Plotter (MicroPico > Open Plotter):
#   1 = Filter tunen: e_roh, e (gefiltert, mm), de (mm/s), hold
#   2 = Regler tunen: P, I, D, u (alle in °), hold
PLOT = 2

# Startwerte, die von Durchlauf zu Durchlauf gemerkt werden
e_prev = float(x_set - tof.ping())  # gefilterter Fehler, Start = erste Messung
derivative = 0.0                    # gefilterte Änderung des Fehlers (mm/s)
t_prev = ticks_us()                 # Zeitpunkt der vorherigen Messung
i = 0.0                             # I-Anteil (°) = ki * Summe der Fehler
hold = False                        # True = Ball ruhig im Sollbereich, Servo hält

# Kopfzeile = Kurvennamen im Plotter
if PLOT == 1:
    print("e_roh, e, de, hold")
else:
    print("P, I, D, u, hold")


# Regelschleife: läuft endlos, jeder Durchlauf = messen, vergleichen, stellen
while True:

    # Messen; Ausreisser abschneiden (ohne Ziel liefert der Sensor z.B. 8190)
    x_new = min(max(tof.ping(), 0), 400)

    # Zeit seit letzter Messung in Sekunden
    t_new = ticks_us()
    dt = ticks_diff(t_new, t_prev) / 1e6

    # Regelabweichung e = Soll - Ist (ungefiltert)
    e_raw = float(x_set - x_new)

    # Holt-Filter = exponentielle Glättung mit Trend.
    # Normale exp. Glättung mischt die Messung mit dem ALTEN Wert -> läuft
    # dem rollenden Ball hinterher. Hier mit der VORHERSAGE (alter Wert +
    # Geschwindigkeit*dt) -> folgt dem Ball ohne Verzögerung.
    e_pred = e_prev + derivative*dt
    e_new = ALPHA*e_raw + (1 - ALPHA)*e_pred
    # Geschwindigkeit (mm/s) gleich mitglätten. Ersetzt das rohe
    # (e_new - e_prev)/dt, das Rauschen durch dt teilt -> Servo zittert
    derivative = BETA*(e_new - e_prev)/dt + (1 - BETA)*derivative

    # Ball ruhig im Sollbereich?
    # Hysterese: zum Verlassen doppelte Geschwindigkeit nötig wie zum Eintreten
    v_limit = 2*V_TOL if hold else V_TOL
    hold = abs(e_new) < TOL and abs(derivative) < v_limit

    if hold:
        # Nur I-Anteil = gelernte "waagrechte" Stellung.
        # i wird nicht verändert -> u bleibt gleich -> Servo steht still.
        # Rollt der Ball los (Ebene gekippt), wird derivative gross -> hold
        # endet sofort, noch bevor der Ball den Sollbereich verlässt.
        p = d = 0.0
    else:
        # Grob/Fein: kp zwischen KP_FINE und KP_COARSE je nach Abweichung
        kp = KP_FINE + (KP_COARSE - KP_FINE)*min(abs(e_new)/E_BIG, 1)

        # I überall aufsummieren: hängt der Ball fest, kippt I die Schiene
        # langsam stärker, bis er sich löst. Begrenzt auf ±I_MAX.
        # Direkt in ° summiert: ki ändern verschiebt die Grenze I_MAX nicht
        i += ki*e_new*dt
        i = max(-I_MAX, min(I_MAX, i))

        p = kp*e_new
        # Reibungs-Kick (im Plot in P enthalten)
        if abs(e_new) > TOL:
            p += FC if e_new > 0 else -FC
        d = kd*derivative

    # PID-Regler: Stellgrösse u = P + I + D
    u_new = p + i + d

    # Schiene kippen (setAngle begrenzt auf ±30°)
    setAngle(u_new, servo)

    # Plotter (hold als 0/10, damit man es neben den anderen Kurven sieht)
    if PLOT == 1:
        print("%.1f, %.1f, %.1f, %d" % (e_raw, e_new, derivative, 10*hold))
    else:
        print("%.1f, %.1f, %.1f, %.1f, %d" % (p, i, d, u_new, 10*hold))

    # Aktuelle Werte werden im nächsten Durchlauf zu den "vorherigen"
    e_prev = e_new
    t_prev = t_new
