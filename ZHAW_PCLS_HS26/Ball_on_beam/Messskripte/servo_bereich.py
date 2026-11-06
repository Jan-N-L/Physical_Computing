###############################################################################
# Ball on beam: Servobereich prüfen (phyphox «Neigung»)
###############################################################################
#
# Fährt den Servo in 5°-Schritten bis -30° und bis +30° (OHNE Offset).
# phyphox zeichnet die Schienenneigung auf. So sehen wir:
#   - bis zu welchem Servowinkel die Schiene weiter kippt
#   - ob sie irgendwo stehen bleibt (Anschlag) oder zurückkippt (Totpunkt)
#
# Vorgehen:
#   1. Ball weg. Handy wie vorher längs auf die Stäbe, Bildschirm unten.
#   2. phyphox > Neigung > «Liegend» öffnen, noch NICHT starten.
#   3. Skript starten, während dem Countdown phyphox mit ▶ starten.
#   4. Zuschauen: Stösst etwas an? Brummt oder zittert der Servo?
#   5. Nach «Fertig»: phyphox stoppen, als CSV exportieren, hochladen.
#
# Achtung: Handy festhalten bereit, falls die Schiene stark kippt.
###############################################################################

from machine import Pin, PWM
from utime import sleep_ms

WINKEL = (0, -5, -10, -15, -20, -25, -30, 0, 5, 10, 15, 20, 25, 30, 0)
HALTEN_S = 6
COUNTDOWN_S = 8
SCHRITT_MS = 30     # langsam fahren: 1° pro SCHRITT_MS, kein grosser Stromstoss


def duty(angle):
    angle = max(-30, min(30, angle))
    return int((angle + 90)/180*8000 + 1000)


servo = PWM(Pin(16))
servo.freq(50)
pos = 0
servo.duty_u16(duty(pos))

print("phyphox jetzt starten! Messung beginnt in:")
for s in range(COUNTDOWN_S, 0, -1):
    print("  %d" % s)
    sleep_ms(1000)

t = 0.0
for w in WINKEL:
    # langsam auf den neuen Winkel fahren
    while pos != w:
        pos += 1 if w > pos else -1
        servo.duty_u16(duty(pos))
        sleep_ms(SCHRITT_MS)
        t += SCHRITT_MS/1000
    print("t = %5.1f s: Servo %+d°" % (t, w))
    sleep_ms(HALTEN_S*1000)
    t += HALTEN_S

print("t = %5.1f s: Fertig. phyphox stoppen und als CSV exportieren." % t)
