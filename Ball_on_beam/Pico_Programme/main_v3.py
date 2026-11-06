# main_v3.py: Ball on Beam, verbesserte Version von Scripts/main.py (06.11.2026)
#
# Gleicher Aufbau wie main.py (PID, Holt-Filter, Kick gegen Haftreibung, Halten im Ziel).
# Neu, aus den Messungen vom 06.11.:
#   1. Rechnet in echten mm über kennlinie.py. Ziel 169.5 mm (Ballmitte ca. 19 cm), ±10 mm.
#   2. OFFSET 1.9° (Erkenntnisse), Rest gleicht der I-Anteil aus.
#   3. Vorhalt T_LEAD: Servo + Messung verzögern ca. 0.12 s -> P rechnet mit der Position,
#      die der Ball in T_LEAD haben wird. Dämpft das Pendeln.
#   Parameter im Simulator v3 abgestimmt (an echten Läufen geeicht), siehe Kommentar unten.
#   4. Ball mit schwerer Seite: Der Haltewinkel ist oft nicht "waagrecht". Beim Halten bleibt
#      der Servo auf dem gelernten Winkel (I-Anteil) statt auf 0 zu springen.
#   5. Losbrechen: Steckt der Ball neben dem Ziel fest, wächst ein Zusatzwinkel bis er rollt.
#   6. Servo-Rampe RATE (°/s): keine grossen Sprünge -> keine Fehlmessungen danach.
#   7. Ausreisserfilter rastet nie ein (nach 3 verworfenen Werten wird akzeptiert).
#
# Start: in VS Code öffnen, MicroPico "Run". Auf dem Pico nötig: vl53l0x.py, kennlinie.py.
# Ausgabe: alle 0.1 s "x, u" (für den Plotter) und "RUHE nach x.x s".
# u > 0: Ball rollt weg vom Sensor.
#
# Simulator v3, 150 Läufe (Start an beiden Enden und dazwischen):
#   main.py alt: 116/150 erfolgreich, Median 9.8 s, Servo 29 °/s
#   main_v3:     140/150 erfolgreich, Median 7.3 s, Servo 59 °/s
#   Ruhiger, aber langsamer: RATE = 150 (137/150, 47 °/s). Schneller, aber unruhiger: ALPHA = 0.5.

from kennlinie import to_mm, MITTE_MM

OFFSET = 1.9        # °   Servo waagrecht (Erkenntnisse, Startwert)
TOL = 10.0          # mm  Ziel ±TOL
T_RUHE = 1.0        # s   so lange im Ziel = "zur Ruhe"
S_LEER = 440        # Sensorwert ab hier: kein Ball

# Regler (Einheiten: mm, mm/s, °)
KP_FINE = 0.35      # °/mm    nahe Ziel
KP_COARSE = 0.60    # °/mm    ab E_BIG
E_BIG = 50.0        # mm
KD = 0.10           # °/(mm/s)
T_LEAD = 0.20       # s       Vorhalt für P
KI = 0.15           # °/(mm*s)
I_MAX = 12.0        # °       Anti-Windup
FC = 3.0            # °       Kick gegen Haftreibung ausserhalb Halte-Zone
E_HOLD = 8.0        # mm      Halten: Ball ruhig innerhalb E_HOLD
V_HOLD = 10.0       # mm/s
KICK_T = 0.5        # s       steckt so lange fest -> Losbrechen
KICK_RATE = 15.0    # °/s
KICK_MAX = 15.0     # °
RATE = 250.0        # °/s     Servo-Rampe
PROG = True         #         Losbrechen auch, wenn der Ball nur kriecht (kein Fortschritt zum Ziel)
PROG_MM = 3.0       # mm      so viel näher zum Ziel in KICK_T gilt als Fortschritt
U_ABS = 30.0        # °       Servo absolut

# Holt-Filter
ALPHA = 0.35
BETA = 0.5


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Regler:
    """Logik ohne Hardware. x in echten mm (gefiltert gegen Ausreisser), ok = Messwert gültig."""

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)
        self.reset(MITTE_MM)

    # Parameter als Attribute, damit sie im Simulator variiert werden können
    kp_fine = KP_FINE
    kp_coarse = KP_COARSE
    e_big = E_BIG
    kd = KD
    t_lead = T_LEAD
    ki = KI
    i_max = I_MAX
    fc = FC
    e_hold = E_HOLD
    v_hold = V_HOLD
    kick_t = KICK_T
    kick_rate = KICK_RATE
    kick_max = KICK_MAX
    rate = RATE
    alpha = ALPHA
    beta = BETA
    prog = PROG
    prog_mm = PROG_MM
    ziel = MITTE_MM     # internes Ziel (mm); Bewertung bleibt immer MITTE_MM ±TOL

    def reset(self, x):
        self.e = self.ziel - x
        self.de = 0.0
        self.i = 0.0
        self.hold = False
        self.u = 0.0
        self.t_fest = 0.0
        self.kick = 0.0
        self.ae_ref = 1e9

    def step(self, x, ok, dt):
        # Holt-Filter auf e = Ziel - x (e > 0: Ball zu nah am Sensor)
        e_old = self.e
        e_raw = self.ziel - x
        e = self.alpha * (e_raw if ok else self.e) + (1 - self.alpha) * (self.e + self.de * dt)
        self.de = self.beta * (e - e_old) / dt + (1 - self.beta) * self.de
        self.e = e
        de = self.de
        ae = e if e > 0 else -e

        # Halten mit Hysterese: Ball ruhig nahe Ziel -> nur gelernter Haltewinkel
        if self.hold:
            self.hold = ae < TOL and -2 * self.v_hold < de < 2 * self.v_hold
        else:
            self.hold = ae < self.e_hold and -self.v_hold < de < self.v_hold

        if self.hold:
            # I lernt langsam weiter, damit der Ball genau in die Mitte kriecht
            self.i = clamp(self.i + 0.3 * self.ki * e * dt, -self.i_max, self.i_max)
            self.t_fest = 0.0
            self.kick = 0.0
            self.ae_ref = ae
            u = self.i
        else:
            ep = e + de * self.t_lead                       # Vorhalt
            aep = ep if ep > 0 else -ep
            kp = self.kp_fine + (self.kp_coarse - self.kp_fine) * min(aep / self.e_big, 1.0)
            p = kp * ep
            if ae > self.e_hold:
                p += self.fc if e > 0 else -self.fc
            self.i = clamp(self.i + self.ki * e * dt, -self.i_max, self.i_max)
            d = self.kd * de
            # Losbrechen: weit weg und steht still (oder kriecht nur, falls prog)
            still = -self.v_hold < de < self.v_hold
            if self.prog:
                if ae < self.ae_ref - self.prog_mm:      # Fortschritt Richtung Ziel
                    self.ae_ref = ae
                    still = False
                elif not still and (de > 0) == (e > 0):  # rollt schnell zum Ziel: kein Stoss nötig
                    self.ae_ref = ae
                else:
                    still = True
            if ae > self.e_hold and still:
                self.t_fest += dt
            else:
                self.t_fest = 0.0
                self.kick = 0.0
            if self.t_fest > self.kick_t:
                self.kick = min(self.kick + self.kick_rate * dt, self.kick_max)
            u = p + self.i + d + (self.kick if e > 0 else -self.kick)

        # Servo-Rampe
        lim = self.rate * dt
        u = clamp(u, self.u - lim, self.u + lim)
        u = clamp(u, -U_ABS - OFFSET, U_ABS - OFFSET)
        self.u = u
        return u


def run():
    from machine import Pin, I2C, PWM
    from vl53l0x import VL53L0X
    from utime import ticks_us, ticks_diff

    tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
    servo = PWM(Pin(16))
    servo.freq(50)

    def set_angle(u):
        a = clamp(u + OFFSET, -U_ABS, U_ABS)
        servo.duty_u16(int((a + 90) / 180 * 8000 + 1000))

    set_angle(0)
    reg = Regler()
    x_last = None
    n_bad = 0
    t_prev = ticks_us()
    t_start = None
    t_in = None
    ruhe = False
    n_print = 0
    print("x, u")
    while True:
        s = tof.ping()
        t = ticks_us()
        dt = ticks_diff(t, t_prev) / 1e6
        t_prev = t
        if dt <= 0:
            dt = 0.03

        # Messung: Kennlinie, Ausreisser verwerfen, nie einrasten
        ok = s < S_LEER
        if ok:
            x = to_mm(s)
            if x_last is not None and n_bad < 3 and abs(x - x_last) > 30 + 500 * dt * (n_bad + 1):
                ok = False
        if ok:
            n_bad = 0
            x_last = x
        else:
            n_bad += 1

        if x_last is None:                      # noch kein Ball
            continue
        if n_bad > 15:                          # Ball weg: waagrecht, neu starten
            set_angle(0)
            print("Ball weg")
            reg.reset(MITTE_MM)
            x_last = None
            t_start = None
            ruhe = False
            continue
        if t_start is None:
            t_start = t
            t_in = None
            reg.reset(x_last)
            print("Start bei %.0f mm" % x_last)

        u = reg.step(x_last, ok, dt)
        set_angle(u)

        # Zeit bis "zur Ruhe" (1 s durchgehend ±TOL)
        tt = ticks_diff(t, t_start) / 1e6
        if abs(x_last - MITTE_MM) <= TOL:
            if t_in is None:
                t_in = tt
            if not ruhe and tt - t_in >= T_RUHE:
                ruhe = True
                print("RUHE nach %.1f s" % t_in)
        else:
            t_in = None
            if ruhe:
                ruhe = False
                print("Ziel verlassen")

        n_print += 1
        if n_print >= 3:
            n_print = 0
            print("%.0f, %.1f" % (x_last, u))


if __name__ == "__main__":
    run()
