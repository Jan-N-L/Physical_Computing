###############################################################################
# Ball on beam: Waagrecht-Test mit rollendem Ball (ersetzt test_offset.py)
###############################################################################
#
# Problem bei test_offset.py: Der Ball klebt (Haftreibung). Er rollt erst
# los, wenn die Schiene deutlich schief ist. Das Ergebnis misst also vor allem
# die Haftreibung und die Stelle an den Enden, nicht das Waagrecht in der Mitte.
#
# Idee hier: Der Ball wird angestossen und rollt durch die Mitte. Gemessen wird
# nur, solange er rollt. Rollt er weg vom Sensor, bremst ihn die Rollreibung.
# Rollt er zum Sensor, bremst sie ihn genauso. Der Mittelwert aus beiden
# Richtungen enthält deshalb nur noch die Schräglage.
#
#   a_weg  = a_schief - a_reibung
#   a_hin  = a_schief + a_reibung
#   a_schief = (a_weg + a_hin)/2      a_reibung = (a_hin - a_weg)/2
#
# Das für mehrere Servowinkel -> Gerade -> Nullstelle = Waagrecht (Offset).
# Nebenbei: Steigung (mm/s² pro ° Servo) und Rollreibung für die Simulation.
#
# Braucht auf dem Pico: vl53l0x.py und kennlinie.py
# Ablauf: Ball irgendwo auf die Schiene legen, Skript starten, nichts berühren.
# Dauer ca. 2 Minuten. Ergebnis wird in calib.json auf dem Pico gespeichert.
###############################################################################

from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import sleep_ms, ticks_ms, ticks_us, ticks_diff
from Ball_on_beam.Scripts.Test.kennlinie import to_mm
try:
    import json
except ImportError:
    import ujson as json

S_LIST = (0, 2, 4)      # ° Servo roh. Nahe am Waagrecht (ca. 1.9°), sonst bremst der
                        # Ball in einer Richtung zu stark und bleibt im Fenster stehen
REPEAT = 2              # Durchgänge pro Winkel und Richtung
PARK = 15               # ° Servo, um den Ball an ein Ende zu rollen
KICK = 25               # ° Servo für den Anstoss
KICK_MS = 650           # ms Anstossdauer am Anfang (passt sich pro Winkel an)
WIN = (80, 250)         # mm Messfenster in echten mm (gute Sensorzone)
T_MAX_MS = 6000         # ms max. Messzeit pro Durchgang
MIN_PTS = 15            # min. Messpunkte im Fenster
STOP_MM = 3             # mm Bewegung in 0.4 s darunter = Ball steht
A_THEORIE = 12.8        # mm/s² pro ° Servo (Hohlkugel auf Stäben, 0.141°/°)
LOG = "waagrecht_log.csv"   # Rohdaten aller Durchgänge (None = aus)

tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
servo = PWM(Pin(16))
servo.freq(50)


def setAngleRaw(angle):
    angle = max(-30, min(30, angle))
    servo.duty_u16(int((angle + 90)/180*8000 + 1000))


def read_raw():
    s = tof.ping()
    return None if s >= 8000 else s


def read_mm():
    s = read_raw()
    return None if s is None else to_mm(s)


def wait_still(timeout_ms=5000):
    """Wartet, bis der Ball 0.6 s lang ruhig liegt. Gibt Position zurück."""
    t0 = ticks_ms()
    win = []
    while ticks_diff(ticks_ms(), t0) < timeout_ms:
        x = read_mm()
        if x is None:
            continue
        win.append((ticks_ms(), x))
        while win and ticks_diff(win[-1][0], win[0][0]) > 600:
            win.pop(0)
        xs = [w[1] for w in win]
        if len(xs) > 10 and max(xs) - min(xs) < 4:
            return sorted(xs)[len(xs)//2]
    return None


def park(end):
    """end 0: Ball zum Sensor, end 1: Ball zum Klotz."""
    setAngleRaw(-PARK if end == 0 else PARK)
    sleep_ms(800)
    wait_still()


def solve3(m, v):
    # Gauss-Elimination 3x3
    a = [row[:] + [v[i]] for i, row in enumerate(m)]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(a[r][i]))
        a[i], a[p] = a[p], a[i]
        for r in range(i + 1, 3):
            f = a[r][i]/a[i][i]
            for c in range(i, 4):
                a[r][c] -= f*a[i][c]
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        x[i] = (a[i][3] - sum(a[i][c]*x[c] for c in range(i + 1, 3)))/a[i][i]
    return x


def fit2(pts):
    """x = c0 + c1*t + c2*t², kleinste Quadrate."""
    s = [0.0]*5
    b = [0.0]*3
    for t, x in pts:
        p = 1.0
        for k in range(5):
            s[k] += p
            if k < 3:
                b[k] += p*x
            p *= t
    m = [[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]]
    return solve3(m, b)


def robust_fit(pts):
    c = fit2(pts)
    res = [abs(x - (c[0] + c[1]*t + c[2]*t*t)) for t, x in pts]
    med = sorted(res)[len(res)//2]
    lim = max(3*1.4826*med, 3.0)
    keep = [p for p, r in zip(pts, res) if r <= lim]
    if len(keep) < len(pts) and len(keep) >= MIN_PTS:
        c = fit2(keep)
        pts = keep
    return c, len(pts)


def run(d, s, kick_ms):
    """Ein Durchgang. d=+1: weg vom Sensor, d=-1: zum Sensor.
    Gibt (a, v, n, dauer, grund) zurück, a=None wenn unbrauchbar."""
    setAngleRaw(d*KICK)
    sleep_ms(kick_ms)
    setAngleRaw(s)
    t0 = ticks_us()
    data = []
    raw = []
    last = None
    stopped = False
    while True:
        sr = read_raw()
        x = None if sr is None else to_mm(sr)
        t = ticks_diff(ticks_us(), t0)/1e6
        raw.append((t, sr))
        if t*1000 > T_MAX_MS:
            break
        if x is None:
            continue
        if last is not None and abs(x - last) > 40:   # Ausreisser
            continue
        last = x
        data.append((t, x))
        if d > 0 and x > WIN[1] + 30:
            break
        if d < 0 and x < WIN[0] - 30:
            break
        # steht der Ball?
        old = [p[1] for p in data if p[0] > t - 0.4]
        if t > 0.8 and len(old) > 5 and max(old) - min(old) < STOP_MM:
            stopped = True
            break
    if LOG:
        with open(LOG, "a") as f:
            for tr, sr in raw:
                f.write("%d,%d,%d,%.3f,%s\n" % (s, d, kick_ms, tr, sr))
    x_start = data[0][1] if data else None
    # Kehrt der Ball um, wechselt die Reibung die Richtung: nur bis zum
    # Umkehrpunkt verwenden, die letzten 0.4 s davor weglassen (fast Stillstand)
    if data:
        i_ext = 0
        for i, p in enumerate(data):
            if d*(p[1] - data[i_ext][1]) > 0:
                i_ext = i
        umkehr = data[-1][0] - data[i_ext][0] > 0.2
        if umkehr or stopped:
            t_end = data[i_ext][0] - 0.4
            data = [p for p in data if p[0] < t_end]
            stopped = True
    pts = [p for p in data if WIN[0] <= p[1] <= WIN[1]]
    if len(pts) < MIN_PTS:
        return None, 0, len(pts), 0, ("blieb stehen" if stopped else "zu wenig Punkte"), x_start

    tm = pts[0][0]
    pts = [(t - tm, x) for t, x in pts]
    c, n = robust_fit(pts)
    dauer = pts[-1][0]
    v_mitte = c[1] + 2*c[2]*dauer/2
    return 2*c[2], v_mitte, n, dauer, ("stand am Ende" if stopped else "ok"), x_start


# Gültig: Ball rollte bis zum Ende des Fensters, oder lange genug und schnell
# genug. Kurze Durchgänge, die abbremsen und stehen bleiben, verfälschen a stark.
def gueltig(r):
    return r[6] == "ok" or (r[5] >= 1.2 and abs(r[3]) >= 50)

print("=== Waagrecht-Test startet: Ball auf die Schiene, nichts beruehren ===")
if LOG:
    with open(LOG, "w") as f:
        f.write("servo,richtung,anstoss_ms,t,sensor\n")
start = {1: KICK_MS, -1: KICK_MS}   # letzter guter Anstoss pro Richtung
rows = []   # (s, d, a, v, n, dauer, grund)
for s in S_LIST:
    for r in range(REPEAT):
        for d in (1, -1):
            kick = start[d]
            for versuch in range(5):
                park(0 if d > 0 else 1)
                a, v, n, dauer, grund, x0 = run(d, s, kick)
                rr = (s, d, a, v, n, dauer, grund)
                if a is not None and gueltig(rr):
                    start[d] = kick
                    break
                # Ball schon beim Anstoss ins/durchs Fenster geschossen -> schwaecher
                zu_stark = x0 is None or (x0 > WIN[0] - 10 if d > 0 else x0 < WIN[1] + 10)
                kick = max(150, int(kick*0.75)) if zu_stark else min(1500, int(kick*1.25))
                print("  Servo %+d°, %s: %s, Start %s mm -> nochmals, Anstoss %d ms"
                      % (s, "weg" if d > 0 else "hin", grund,
                         "-" if x0 is None else "%.0f" % x0, kick))
            if a is None or not gueltig(rr):
                print("  Servo %+d°, %s: kein gueltiges Ergebnis" % (s, "weg" if d > 0 else "hin"))
                continue
            rows.append(rr)
            print("  Servo %+d°, %-3s: a %+7.1f mm/s²  v %+6.0f mm/s  %2d Punkte  %.2f s  %s  (Anstoss %d ms)"
                  % (s, "weg" if d > 0 else "hin", a, v, n, dauer, grund, kick))

ok_rows = rows
print("\n=== ERGEBNIS (bitte kopieren) ===")
print("verwendet %d von %d Durchgaengen" % (len(ok_rows), len(rows)))
mean = {}
for s in S_LIST:
    for d in (1, -1):
        a_l = [r[2] for r in ok_rows if r[0] == s and r[1] == d]
        if a_l:
            mean[(s, d)] = sum(a_l)/len(a_l)
# Rollreibung aus Winkeln mit beiden Richtungen
fr = [(mean[(s, -1)] - mean[(s, 1)])/2 for s in S_LIST if (s, 1) in mean and (s, -1) in mean]
pts = []
if fr:
    ar = sum(fr)/len(fr)
    print("servo, a_weg, a_hin, a_schief   (mm/s², - = ungueltig)")
    for s in S_LIST:
        w = mean.get((s, 1))
        h = mean.get((s, -1))
        if w is not None and h is not None:
            ag = (w + h)/2
        elif w is not None:
            ag = w + ar
        elif h is not None:
            ag = h - ar
        else:
            print("%+d, -, -, fehlt" % s)
            continue
        pts.append((s, ag))
        print("%+d, %s, %s, %.1f" % (s, "-" if w is None else "%.1f" % w,
                                    "-" if h is None else "%.1f" % h, ag))
else:
    print("Kein Winkel mit gueltigen Durchgaengen in beide Richtungen.")

if len(pts) >= 2:
    n = len(pts)
    mx = sum(p[0] for p in pts)/n
    my = sum(p[1] for p in pts)/n
    k = sum((p[0] - mx)*(p[1] - my) for p in pts)/sum((p[0] - mx)**2 for p in pts)
    s0 = mx - my/k
    rest = max(abs(p[1] - k*(p[0] - s0)) for p in pts)
    print("Waagrecht (Offset)  = %+.2f° Servo   (test_offset: +0.5, Handy: +3.7)" % s0)
    print("Steigung            = %.1f mm/s² pro ° Servo   (Theorie ca. %.1f)" % (k, A_THEORIE))
    print("Rollreibung         = %.1f mm/s²  (entspricht %.2f° Servo)" % (ar, ar/k))
    print("Max. Abweichung von der Geraden = %.1f mm/s²" % rest)
    if not (min(S_LIST) <= s0 <= max(S_LIST)):
        print("WARNUNG: Waagrecht liegt ausserhalb der Testwinkel. S_LIST anpassen.")
    try:
        with open("calib.json", "w") as f:
            json.dump({"offset": s0, "a_pro_grad": k, "a_reibung": ar}, f)
        print("Gespeichert in calib.json")
    except Exception as e:
        print("calib.json nicht gespeichert:", e)
    setAngleRaw(s0)
    print("Schiene steht jetzt auf Waagrecht.")
else:
    print("Zu wenig Ergebnisse fuer eine Auswertung.")
    setAngleRaw(0)
print("=== Ende ===")
