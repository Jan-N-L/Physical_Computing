###############################################################################
# Ball on beam: Test für ANGLE_OFFSET
###############################################################################
#
# Sucht den Servo-Winkel, bei dem die Schiene waagrecht ist. Läuft allein,
# braucht nur vl53l0x.py auf dem Pico. Dauer ca. 1.5 Minuten.
#
# Ablauf pro Durchgang:
#   1. Schiene stark kippen -> Ball rollt zum Sensor
#   2. Winkel schrittweise erhöhen, bis der Ball wegrollt   -> Winkel "weg"
#   3. Schiene stark andersrum -> Ball rollt ans andere Ende
#   4. Winkel schrittweise senken, bis der Ball zurückrollt -> Winkel "zurück"
#
# Grob nach fein: Durchgang 1 sucht mit grossen Schritten über den ganzen
# Bereich. Jeder weitere startet kurz vor dem letzten Ergebnis und nimmt
# kleinere Schritte (STEPS). Rollt der Ball schon beim Start, war der Start zu
# nah: dann läuft die Suche automatisch nochmal von weiter aussen.
#
# Ergebnis (steht am Schluss im Terminal):
#   waagrecht    = Mitte von "weg" und "zurück" -> als ANGLE_OFFSET in main.py
#   toter Bereich = "weg" - "zurück": so viele Grad kann der Regler ändern,
#                  ohne dass der Ball reagiert (Reibung, Servo-Spiel).
#                  Misst etwas zu viel, weil der Ball erst MOVED mm rollen muss.
#                  Normal: wenige Grad. Viel mehr -> Mechanik prüfen
#   "waagrecht" sollte von Durchgang zu Durchgang stabil bleiben. Springt der
#   Wert stark -> Test wiederholen, Mechanik prüfen.
#   Danach bleibt die Schiene waagrecht stehen: Ball in die Mitte legen,
#   er sollte liegen bleiben.
#
# Plotter (MicroPico > Open Plotter): winkel (°), distanz (cm)
###############################################################################

from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import sleep_ms

STEPS = (2, 0.5, 0.25)  # ° pro Schritt, ein Durchgang pro Wert: grob -> fein
MARGIN = 3              # Start so viele alte Schritte vor dem letzten Ergebnis
WAIT_MS = 3000          # pro Schritt warten: knapp über waagrecht rollt der Ball langsam an
SETTLE_MS = 3000        # Ball rollt ans Ende und kommt zur Ruhe
MOVED = 30              # mm  so weit muss der Ball rollen, damit es als "rollt" zählt
LOW = -15               # °   damit rollt der Ball sicher zum Sensor
HIGH = 20               # °   damit rollt der Ball sicher ans andere Ende


def setAngle(angle, pwm):
    # Wie in main.py, aber OHNE ANGLE_OFFSET: genau diesen suchen wir ja
    angle = max(-30, min(30, angle))
    pwm.duty_u16(int((angle + 90)/180*8000 + 1000))


tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
servo = PWM(Pin(16))
servo.freq(50)


def distance():
    # Median aus 3 Messungen: ein einzelner Ausreisser zählt nicht als "rollt"
    return sorted([tof.ping(), tof.ping(), tof.ping()])[1]


def sweep(end, start, step):
    """Schiene auf end, Ball ans Ende rollen lassen, dann ab start den Winkel
    in Schritten step ändern, bis der Ball MOVED mm gerollt ist.
    Gibt diesen Winkel zurück, None: Ball nicht gesehen oder nie losgerollt."""
    setAngle(end, servo)
    sleep_ms(SETTLE_MS)
    rest = distance()
    if rest > 1000:                 # Sensor liefert ~8190, wenn er nichts sieht
        return None
    angle = start
    while LOW <= angle <= HIGH:
        setAngle(angle, servo)
        sleep_ms(WAIT_MS)
        x = distance()
        print("%.2f, %.1f" % (angle, x/10))
        if x < 1000 and abs(x - rest) > MOVED:
            if angle == start and start != end:
                # Rollt schon beim Start: Start war zu nah -> von weiter aussen
                return sweep(end, max(LOW, min(HIGH, start - 4*step)), step)
            return angle
        angle += step
    return None


print("winkel, distanz")
away, back, margin = LOW, HIGH, 0   # Durchgang 1: ganzer Bereich
level = None
for i, step in enumerate(STEPS):
    a = sweep(LOW, max(LOW, away - margin), step)
    b = sweep(HIGH, min(HIGH, back + margin), -step)
    if a is None or b is None:
        print("Durchgang %d ohne Ergebnis: Ball nicht gesehen oder nie losgerollt" % (i + 1))
        break
    away, back, margin = a, b, MARGIN*step
    level = (away + back)/2
    print("Durchgang %d (%.2f°): weg ab %.2f°, zurück ab %.2f°  ->  waagrecht %.2f°, toter Bereich %.2f°"
          % (i + 1, step, away, back, level, away - back))

if level is not None:
    print("ANGLE_OFFSET = %.2f" % level)
    setAngle(level, servo)          # Kontrolle: Ball in die Mitte legen
