###############################################################################
# Ball on beam: Offset und Übersetzung mit dem Handy messen (phyphox «Neigung»)
###############################################################################
#
# Der Servo fährt nacheinander auf feste Winkel (OHNE Offset) und wartet.
# Du misst jeweils die Schienenneigung mit dem Handy und tippst die Werte ein.
# Am Schluss rechnet das Skript aus:
#   - Offset: Servowinkel, bei dem die Schiene waagrecht ist
#   - Übersetzung: Grad Schiene pro Grad Servo
#   - Spiel: Unterschied der zwei 0°-Messungen (von verschiedenen Seiten)
#
# Handy: phyphox > Neigung > Reiter «Liegend» > Wert «Kippwinkel auf/ab».
# Ball wegnehmen. Handy längs auf die Stäbe legen:
#   Wert A: Handy-Oberkante zeigt zum Sensor
#   Wert B: Handy um 180° gedreht (Oberkante zeigt zum Klotz)
# Immer gleich hinlegen (gleiche Stelle auf der Schiene).
# Dezimalkomma oder -punkt geht beides.
#
# Läuft auch am PC ohne Pico: dann fährt kein Servo, es wird nur gerechnet.
###############################################################################

WINKEL = (0, 10, -10, 0)    # ° Servo, roh (ohne Offset)
OFFSET_ALT = 0.5            # ° bisheriger Wert aus test_offset.py

try:
    from machine import Pin, PWM
    servo = PWM(Pin(16))
    servo.freq(50)
except ImportError:
    servo = None
    print("(Kein Pico: Servo wird nicht bewegt, nur Auswertung)")


def setAngleRaw(angle):
    if servo is None:
        return
    angle = max(-30, min(30, angle))
    servo.duty_u16(int((angle + 90)/180*8000 + 1000))


def ask(text):
    while True:
        s = input(text).strip().replace(",", ".")
        try:
            v = float(s)
        except ValueError:
            print("   Bitte eine Zahl eingeben, z.B. 0.8 oder -1,2")
            continue
        # Handy mit Bildschirm nach unten zeigt Werte um ±180°: auf ±90° falten.
        # Dafür muss das Vorzeichen genau wie angezeigt eingegeben werden!
        if v > 90:
            v -= 180
        elif v < -90:
            v += 180
        return v


data = []   # (servo, schiene, handyfehler)
for w in WINKEL:
    setAngleRaw(w)
    print("\n>> Servo steht auf %+d°" % w)
    a = ask("   Wert A (Oberkante zum Sensor): ")
    b = ask("   Wert B (um 180° gedreht):      ")
    # Drehen kehrt das Vorzeichen der Schienenneigung um, der Handyfehler bleibt
    theta = (a - b)/2
    err = (a + b)/2
    data.append((w, theta, err))
    print("   -> Schiene %+.2f°, Handyfehler %+.2f°" % (theta, err))

setAngleRaw(0)

# Gerade durch alle Punkte: schiene = k * (servo - offset)
n = len(data)
mx = sum(d[0] for d in data)/n
my = sum(d[1] for d in data)/n
sxx = sum((d[0] - mx)**2 for d in data)
sxy = sum((d[0] - mx)*(d[1] - my) for d in data)
k = sxy/sxx
offset = mx - my/k if abs(k) > 1e-6 else float("nan")
rest = max(abs(d[1] - k*(d[0] - offset)) for d in data)
null = [d[1] for d in data if d[0] == 0]
spiel = abs(null[0] - null[-1]) if len(null) > 1 else float("nan")
handy = sum(d[2] for d in data)/n

print("\n=== ERGEBNIS (bitte kopieren) ===")
for (w, theta, err) in data:
    print("servo %+3d°: schiene %+.2f°, handyfehler %+.2f°" % (w, theta, err))
print("Offset       = %+.2f°   (bisher %+.2f°, Differenz %.2f°)"
      % (offset, OFFSET_ALT, offset - OFFSET_ALT))
print("Übersetzung  = %.3f ° Schiene pro ° Servo" % abs(k))
print("Spiel bei 0° = %.2f°   (gross = Servo/Gestänge hat Spiel)" % spiel)
print("Max. Abweichung von der Geraden = %.2f°" % rest)
print("Handyfehler im Mittel = %+.2f°" % handy)
if abs(k) < 0.05:
    print("WARNUNG: Schiene hat sich kaum bewegt. Falscher Wert abgelesen")
    print("         (links/rechts statt auf/ab?) oder Vorzeichen vergessen?")
elif rest > 0.5:
    print("WARNUNG: Punkte liegen schlecht auf einer Geraden. Messung wiederholen.")
print("=== Ende ===")
