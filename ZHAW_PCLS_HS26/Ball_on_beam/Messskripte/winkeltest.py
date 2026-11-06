###############################################################################
# Ball on beam: Winkeltest VL53L0X
###############################################################################
#
# Prüft, ob der Sensorwert vom Kippwinkel abhängt.
# Wenn ja, reicht eine einfache Umrechnungstabelle nicht.
# Braucht nur vl53l0x.py auf dem Pico. Dauer ca. 1 Minute.
#
# Ablauf:
#   1. OHNE Ball starten. Schiene kippt durch alle WINKEL -> misst "leer".
#   2. Ball mit wenig Klebeband auf der 200-mm-Markierung fixieren, Hand weg,
#      dann Enter. Schiene kippt wieder durch alle WINKEL -> misst "ball".
#
# Ausgabe am Schluss: eine Tabelle. Bitte ganzen Text ab ERGEBNIS kopieren.
###############################################################################

from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import sleep_ms

OFFSET = 0.5                    # °  waagrecht (wie main.py)
WINKEL = (-10, -5, 0, 5, 10)    # °  relativ zu waagrecht
N = 40                          # Messungen pro Winkel
SERVO_MS = 600                  # Servo fährt und beruhigt sich


def setAngle(angle):
    angle = max(-30, min(30, angle + OFFSET))
    servo.duty_u16(int((angle + 90)/180*8000 + 1000))


tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
servo = PWM(Pin(16))
servo.freq(50)
results = []


def sweep(label):
    ref = None
    for w in WINKEL:
        setAngle(w)
        sleep_ms(SERVO_MS)
        vals = [tof.ping() for _ in range(N)]
        vals = [v for v in vals if v < 8000]
        if vals:
            m = sum(vals)/len(vals)
            sd = (sum((v - m)**2 for v in vals)/len(vals))**0.5
        else:
            m = sd = float("nan")
        if w == 0:
            ref = m
        results.append((label, w, m, sd, N - len(vals)))
        print("  %-4s %+3d°: mittel %6.1f  std %4.1f" % (label, w, m, sd))
    setAngle(0)
    return ref


print("=== Winkeltest startet (OHNE Ball) ===")
leer0 = sweep("leer")
setAngle(0)
input("\n>> Ball mit Klebeband auf 200-mm-Markierung fixieren, Hand weg, dann Enter: ")
ball0 = sweep("ball")

print("\n=== ERGEBNIS (bitte kopieren) ===")
print("art, winkel, mittel, std, ungueltig, abweichung_zu_0grad")
for (label, w, m, sd, bad) in results:
    ref = leer0 if label == "leer" else ball0
    print("%s, %d, %.1f, %.2f, %d, %.1f" % (label, w, m, sd, bad, m - ref))
print("=== Ende ===")
