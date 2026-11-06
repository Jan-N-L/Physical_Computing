###############################################################################
# Ball on beam: Neigung als Verlauf aufzeichnen (phyphox «Neigung»)
###############################################################################
#
# Der Servo fährt die WINKEL nach festem Zeitplan ab (OHNE Offset).
# phyphox zeichnet dabei die Neigung auf. Die Stufen im Verlauf zeigen,
# wie stark sich die Schiene pro Servowinkel neigt.
#
# Vorgehen:
#   1. Ball weg. Handy längs auf die Stäbe legen, Oberkante zum Sensor.
#   2. phyphox > Neigung > Reiter «Liegend» öffnen, noch NICHT starten.
#   3. Skript starten. Während dem Countdown phyphox mit ▶ starten.
#      Handy danach nicht mehr berühren.
#   4. Nach «Fertig»: phyphox stoppen, exportieren (⋮ > Daten exportieren > CSV).
#   5. Gleich nochmals mit um 180° gedrehtem Handy (Oberkante zum Klotz).
# Die zwei CSV-Dateien in den Ordner Ball_on_beam legen.
###############################################################################

from machine import Pin, PWM
from utime import sleep_ms

WINKEL = (0, 10, -10, 0, 5, -5, 0)  # ° Servo, roh (ohne Offset)
HALTEN_S = 10                       # s pro Winkel
COUNTDOWN_S = 8                     # s Zeit, um phyphox zu starten


def setAngleRaw(angle):
    angle = max(-30, min(30, angle))
    servo.duty_u16(int((angle + 90)/180*8000 + 1000))


servo = PWM(Pin(16))
servo.freq(50)
setAngleRaw(0)

print("phyphox jetzt starten! Messung beginnt in:")
for s in range(COUNTDOWN_S, 0, -1):
    print("  %d" % s)
    sleep_ms(1000)

t = 0
for w in WINKEL:
    setAngleRaw(w)
    print("t = %3d s: Servo %+d°" % (t, w))
    sleep_ms(HALTEN_S*1000)
    t += HALTEN_S

setAngleRaw(0)
print("t = %3d s: Fertig. phyphox stoppen und als CSV exportieren." % t)
