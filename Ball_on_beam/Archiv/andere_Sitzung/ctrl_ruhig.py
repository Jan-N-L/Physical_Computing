# ctrl_ruhig.py: Sanfter Regler nach dem Vorbild von Scripts/main.py, ergänzt mit den
# Erkenntnissen vom 06.11. (Kennlinie in echten mm, Ballstörung, Verzögerung, Servo-Zucken).
#
# Grundidee wie main.py: P (fein nahe Ziel, grob weiter weg) + I + D + kleiner Kick gegen
# Haftreibung, Holt-Filter. Neu:
#   1. Rechnet in echten mm (kennlinie), Ziel 169.5 mm = Ballmitte ca. 19 cm.
#   2. Ruhe-Halten: Ball ruhig im Ziel -> Servo wird EINGEFROREN (kein Sprung, kein
#      "waagrecht stellen"). Die schwere Seite des Balls braucht oft einen Haltewinkel.
#   3. Kein Zucken: Servo-Rampe (max. rate °/s) und Totband für kleine Änderungen.
#   4. Losbrechen: steckt der Ball neben dem Ziel in einer Mulde fest, wächst ein Zusatzwinkel.
# u relativ zu bob_core.OFFSET; positives u -> Ball rollt weg vom Sensor.
from bob_core import MITTE_MM


class Regler:
    def __init__(self, kp_fein=0.16, kp_grob=0.24, e_gross=40.0, ki=0.035, i_max=12.0,
                 kd=0.13, fc=2.5, alpha=0.3, beta=0.35,
                 e_halt=6.0, v_halt=12.0, t_halt=0.25, e_los=12.0, v_los=40.0,
                 rate=60.0, totband=0.3,
                 kick_e=8.0, kick_v=8.0, kick_t=0.8, kick_rate=10.0, kick_max=15.0):
        self.kp_fein = kp_fein
        self.kp_grob = kp_grob
        self.e_gross = e_gross
        self.ki = ki
        self.i_max = i_max
        self.kd = kd
        self.fc = fc
        self.alpha = alpha
        self.beta = beta
        self.e_halt = e_halt
        self.v_halt = v_halt
        self.t_halt = t_halt
        self.e_los = e_los
        self.v_los = v_los
        self.rate = rate
        self.totband = totband
        self.kick_e = kick_e
        self.kick_v = kick_v
        self.kick_t = kick_t
        self.kick_rate = kick_rate
        self.kick_max = kick_max
        self.reset(MITTE_MM)

    def reset(self, x):
        self.x = x
        self.v = 0.0
        self.i = 0.0
        self.u = 0.0           # zuletzt ausgegeben
        self.halt = False
        self.t_ruhig = 0.0
        self.t_fest = 0.0
        self.kick = 0.0

    def step(self, z, ok, dt):
        # Holt-Filter (wie main.py), ungültige Messung -> Vorhersage
        x_old = self.x
        xp = self.x + self.v * dt
        self.x = self.alpha * z + (1 - self.alpha) * xp if ok else xp
        self.v = self.beta * (self.x - x_old) / dt + (1 - self.beta) * self.v
        e = MITTE_MM - self.x
        ae = e if e > 0 else -e
        v = self.v
        av = v if v > 0 else -v

        # Ruhe-Halten mit Hysterese
        if self.halt:
            if ae > self.e_los or av > self.v_los:
                self.halt = False
                self.t_ruhig = 0.0
            else:
                return self.u                      # Servo eingefroren
        elif ae < self.e_halt and av < self.v_halt:
            self.t_ruhig += dt
            if self.t_ruhig >= self.t_halt:
                self.halt = True
                return self.u
        else:
            self.t_ruhig = 0.0

        # PID wie main.py
        f = ae / self.e_gross
        kp = self.kp_fein + (self.kp_grob - self.kp_fein) * (f if f < 1 else 1)
        p = kp * e
        if ae > self.e_halt:
            p += self.fc if e > 0 else -self.fc
        i = self.i + self.ki * e * dt
        self.i = self.i_max if i > self.i_max else -self.i_max if i < -self.i_max else i
        u = p + self.i - self.kd * v

        # Losbrechen aus einer Mulde
        if ae > self.kick_e and av < self.kick_v:
            self.t_fest += dt
        else:
            self.t_fest = 0.0
            self.kick = 0.0
        if self.t_fest > self.kick_t:
            k = self.kick + self.kick_rate * dt
            self.kick = k if k < self.kick_max else self.kick_max
            u += self.kick if e > 0 else -self.kick

        # kein Zucken: Rampe + Totband
        lim = self.rate * dt
        du = u - self.u
        if du > lim:
            du = lim
        elif du < -lim:
            du = -lim
        if -self.totband < du < self.totband:
            du = 0.0
        self.u += du
        return self.u
