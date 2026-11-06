###############################################################################
# Ball on beam: Rauschtest VL53L0X
###############################################################################
#
# Misst, wie stark der Sensor an verschiedenen Ballpositionen rauscht und wie
# genau er ist. Vergleicht dabei 3 Messzeiten (Timing Budget).
# Braucht nur vl53l0x.py auf dem Pico. Dauer ca. 2 Minuten.
#
# Ablauf:
#   1. Automatisch: Ball rollt zum Sensor  -> misst "nah"
#   2. Automatisch: Ball rollt ans andere Ende -> misst "fern"
#   3. Von Hand: Ball nacheinander auf die Markierungen in POS legen.
#      Abstand = Sensorfront bis Ballvorderkante (Massstab).
#      Danach Enter im Terminal. Das Skript wartet so lange.
#   4. Ball wegnehmen, Enter -> misst "leer" (was der Sensor ohne Ball sieht)
#
# Ausgabe am Schluss: eine Tabelle. Bitte ganzen Terminal-Text kopieren.
#   mittel = Mittelwert, std = Rauschen (Standardabweichung), spanne = max-min
#   fehler = mittel - Massstab (nur bei Handpositionen)
#   ungueltig = Messungen ohne Ergebnis (8190), Hz = Messungen pro Sekunde
###############################################################################

from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import sleep_ms, ticks_ms, ticks_diff

OFFSET = 0.5                        # °  waagrecht (wie main.py)
POS = (100, 150, 200, 250, 300)     # mm Handpositionen, Sensorfront bis Ball
BUDGETS = (20000, 33000, 66000)     # us Messzeiten, die verglichen werden
N = 60                              # Messungen pro Position und Messzeit
LOW = -15                           # °  Ball rollt sicher zum Sensor
HIGH = 20                           # °  Ball rollt sicher ans andere Ende
SETTLE_MS = 3000                    # Ball rollt ans Ende und kommt zur Ruhe
INVALID = 8000                      # mm ab hier: Sensor sieht nichts


def setAngle(angle):
    angle = max(-30, min(30, angle + OFFSET))
    servo.duty_u16(int((angle + 90)/180*8000 + 1000))


tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
servo = PWM(Pin(16))
servo.freq(50)
results = []


def set_budget(us):
    ok = tof.set_measurement_timing_budget(us)
    if not ok:
        print("WARNUNG: Messzeit %d us nicht gesetzt" % us)
    return ok


def record(label, ruler=None):
    """Misst N Werte pro Messzeit und speichert die Statistik."""
    for b in BUDGETS:
        set_budget(b)
        vals, bad = [], 0
        t0 = ticks_ms()
        for _ in range(N):
            x = tof.ping()
            if x >= INVALID:
                bad += 1
            else:
                vals.append(x)
        hz = N*1000/max(1, ticks_diff(ticks_ms(), t0))
        if vals:
            m = sum(vals)/len(vals)
            sd = (sum((v - m)**2 for v in vals)/len(vals))**0.5
            span = max(vals) - min(vals)
            # Bewegt? Erstes gegen letztes Viertel vergleichen
            q = max(1, len(vals)//4)
            drift = sum(vals[-q:])/q - sum(vals[:q])/q
        else:
            m = sd = span = drift = float("nan")
        results.append((label, ruler, b//1000, m, sd, span, drift, bad, hz))
        print("  %-6s %2d ms: mittel %6.1f  std %5.1f  spanne %4.0f  ungueltig %2d  %4.1f Hz"
              % (label, b//1000, m, sd, span, bad, hz))


print("=== Rauschtest startet ===")
# 1. nah
setAngle(LOW)
sleep_ms(SETTLE_MS)
record("nah")
# 2. fern
setAngle(HIGH)
sleep_ms(SETTLE_MS)
record("fern")
# 3. Handpositionen
setAngle(0)
for p in POS:
    input("\n>> Ball auf %d mm legen (Sensorfront bis Ballvorderkante), dann Enter: " % p)
    record("%d" % p, p)
# 4. leer
input("\n>> Ball WEGNEHMEN, dann Enter: ")
record("leer")
set_budget(33000)

# Tabelle
print("\n=== ERGEBNIS (bitte kopieren) ===")
print("pos, massstab, budget_ms, mittel, std, spanne, drift, fehler, ungueltig, hz")
for (label, ruler, b, m, sd, span, drift, bad, hz) in results:
    err = (m - ruler) if ruler is not None else float("nan")
    print("%s, %s, %d, %.1f, %.2f, %.0f, %.1f, %.1f, %d, %.1f"
          % (label, ruler if ruler is not None else "-", b, m, sd, span, drift, err, bad, hz))
print("Hinweis: |drift| > 5 mm heisst, der Ball hat sich waehrend der Messung bewegt.")
print("=== Ende ===")
