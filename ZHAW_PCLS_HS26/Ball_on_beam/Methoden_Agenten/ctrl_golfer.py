# ctrl_golfer.py: Golfer-Impuls-Regelung fuer Ball on Beam (MicroPython und CPython).
#
# Ablauf: Liegt der Ball ausserhalb der Mitte, folgt ein Schlag: kippen Richtung Mitte
# (Stoss), bis der Ball nach Plan mit Bremswinkel und Rollreibung genau in der Mitte
# ausrollt. Danach gefuehrtes Bremsen: In jedem Schritt wird die noetige Verzoegerung neu
# gerechnet (Nachkorrektur gegen unbekannte Reibung und Ballstoerung). Nahe der Mitte
# uebernimmt die Haltephase: Haltewinkel (lernt wie ein I-Anteil) plus weiche Feder und
# Daempfung. Rollt der Ball aus der Haltezone, folgt ein neuer (kleinerer) Schlag.
#
# Interface: r = Regler(); r.reset(x); u = r.step(x, ok, dt)  (u in Grad relativ OFFSET)

from bob_core import MITTE_MM

WAIT, PUSH, BRAKE, HOLD = 0, 1, 2, 3


class Regler:
    def __init__(self, u_push=22.0, u_brk=26.0, plan=0.45, k=8.5, mu=18.0, t_lag=0.08,
                 rate=300.0, a_ab=0.45, b_ab=0.08, gate=15.0, e_hold=30.0, v_hold=60.0,
                 e_out=40.0, kp=1.2, kd=0.2, ki=0.8, h_max=25.0, kick=0.0,
                 v_stick=8.0, u_min=8.0, c_push=0.12, t_wait=0.1, e_db=6.0, v_learn=30.0,
                 v_esc=20.0, r_esc=40.0, u_lim=30.0):
        self.u_push = u_push    # Grad, maximaler Stosswinkel
        self.u_min = u_min      # Grad, Stosswinkel bei kurzer Distanz
        self.c_push = c_push    # Grad pro mm Distanz
        self.u_brk = u_brk      # Grad, maximaler Bremswinkel
        self.plan = plan        # Anteil Bremswinkel fuer die Planung (Reserve)
        self.k = k              # mm/s^2 pro Grad (Planwert)
        self.mu = mu            # mm/s^2 Rollreibung (Planwert)
        self.t_lag = t_lag      # s Totzeit Servo + Sensor (Vorhersage)
        self.rate = rate        # Grad/s Rampe (unter 20 Grad in 120 ms: keine Fehlwerte)
        self.a_ab = a_ab
        self.b_ab = b_ab
        self.gate = gate        # mm Ausreissertor
        self.e_hold = e_hold    # mm Haltezone (Eintritt)
        self.v_hold = v_hold    # mm/s
        self.e_out = e_out      # mm Haltezone (Austritt -> neuer Schlag)
        self.kp = kp            # Grad/mm Feder in der Haltephase
        self.kd = kd            # Grad s/mm Daempfung
        self.ki = ki            # Grad/(mm s) Lernrate Haltewinkel
        self.h_max = h_max      # Grad Grenze Haltewinkel
        self.kick = kick        # Grad Losbrech-Zuschlag, wenn Ball klebt
        self.v_stick = v_stick  # mm/s darunter gilt der Ball als klebend
        self.t_wait = t_wait
        self.v_esc = v_esc      # mm/s darunter klebt der Ball beim Stoss
        self.r_esc = r_esc      # Grad/s Erhoehung Stosswinkel beim Kleben
        self.u_lim = u_lim      # Grad Grenze Stoss
        self.esc = 0.0
        self.e_db = e_db        # mm Totband Haltephase
        self.v_learn = v_learn  # mm/s nur darunter Haltewinkel lernen
        self.reset(MITTE_MM)

    def reset(self, x):
        self.x = x
        self.v = 0.0
        self.u = 0.0
        self.hold = 0.0         # Haltewinkel (relativ OFFSET), bleibt ueber Schlaege
        self.state = WAIT
        self.tw = 0.0
        self.s = 1
        self.nrej = 0

    def step(self, x, ok, dt):
        if dt <= 0:
            dt = 0.029
        # Schaetzer: Alpha-Beta, Ausreissertor (nie einrastend)
        xp = self.x + self.v * dt
        if ok and abs(x - xp) > self.gate + 0.3 * abs(self.v) * dt and self.nrej < 3:
            ok = False
            self.nrej += 1
        if ok:
            self.nrej = 0
            r = x - xp
            self.x = xp + self.a_ab * r
            self.v = self.v + self.b_ab * r / dt
        else:
            self.x = xp
        T = self.t_lag
        xq = self.x + self.v * T
        v = self.v
        e = MITTE_MM - xq
        st = self.state
        u_t = self.hold

        if st == HOLD:
            if abs(e) > self.e_out:
                st = WAIT
                self.tw = self.t_wait
            else:
                if abs(MITTE_MM - self.x) < self.e_db and abs(v) < self.v_stick:
                    u_t = self.u          # Ball klebt in der Mitte: nichts bewegen
                else:
                    if abs(v) < self.v_learn:
                        self.hold += self.ki * e * dt
                        h = self.h_max
                        self.hold = h if self.hold > h else -h if self.hold < -h else self.hold
                    u_t = self.hold + self.kp * e - self.kd * v
                    if abs(v) < self.v_stick and abs(e) > self.e_db:
                        u_t += self.kick if e > 0 else -self.kick
        if st == WAIT:
            if abs(e) < self.e_hold and abs(v) < self.v_hold:
                st = HOLD
                u_t = self.hold
            else:
                self.tw -= dt
                if self.tw <= 0:
                    self.s = 1 if e > 0 else -1
                    p = self.u_min + self.c_push * abs(e)
                    self.pp = self.u_push if p > self.u_push else p
                    st = PUSH
                    self.esc = 0.0
        if st == PUSH or st == BRAKE:
            s = self.s
            d = s * e
            w = s * v
            if d < -5.0:
                # ueber das Ziel hinaus: neuer Schlag in Gegenrichtung
                st = WAIT
                self.tw = 0.0
            elif d < self.e_hold * 0.5 and abs(w) < self.v_hold:
                st = HOLD
            else:
                ab = self.k * self.u_brk * self.plan + self.mu
                if st == PUSH and w > 0 and w * w >= 2 * ab * d:
                    st = BRAKE
                if st == PUSH:
                    # Losbrechen: klebt der Ball (Mulde), Stosswinkel stetig erhoehen
                    if w < self.v_esc:
                        self.esc += self.r_esc * dt
                    elif w > 2 * self.v_esc:
                        self.esc = 0.0
                    p = (self.pp if w > 0 else self.u_push) + self.esc
                    u_t = s * (self.u_lim if p > self.u_lim else p)
                else:
                    if w <= 0:
                        u_t = s * self.u_push
                        if w < -20:
                            st = PUSH
                    else:
                        a_need = w * w / (2 * (d if d > 2.0 else 2.0))
                        du = (self.mu - a_need) / self.k
                        if du > self.u_push:
                            du = self.u_push
                        elif du < -self.u_brk:
                            du = -self.u_brk
                        u_t = s * du
        self.state = st
        # Rampe
        m = self.rate * dt
        if u_t > self.u + m:
            u_t = self.u + m
        elif u_t < self.u - m:
            u_t = self.u - m
        self.u = u_t
        return u_t
