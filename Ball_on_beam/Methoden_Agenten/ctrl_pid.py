# ctrl_pid.py: PID-Regler für Ball on Beam in echten mm (Pico und Simulator).
# Interface siehe bob_core.py: Regler(**param), reset(x), step(x, ok, dt) -> u in ° Servo relativ zu OFFSET.
#
# Aufbau pro Schritt:
#   1. Beobachter (Position xh, Tempo vh, Störbeschleunigung dh in mm/s²). Modell: a = kn*u + dh.
#      dh fasst Offsetfehler, Rollreibung und die Störung des unrunden Balls zusammen.
#      Mit g_ff > 0 wird dh vorgesteuert (u -= g_ff*dh/kn). g_ff = 0: reiner Alpha-Beta-Filter.
#   2. Sollwertführung: Tempo-Sollwert v_ref = min(kp/kd*|e|, sqrt(2*a_br*|e|), v_max), Richtung e.
#      Für kleine e ist das genau PD: u = kp*e - kd*v.
#   3. Reibungskompensation: +fc in Bewegungsrichtung (weich über v_fc).
#   4. Losbrechen: Ball steht ausserhalb der Ruhezone -> Zusatz in Zielrichtung, wächst mit
#      kick_rate bis kick_max. Steht er nach t_sw immer noch (Mulde des unrunden Balls):
#      «Schwung holen», t_back lang voll in Gegenrichtung, danach wieder voll Richtung Ziel.
#   5. I-Anteil nur nahe am Ziel (|e| < i_zone), begrenzt auf ±i_max, Anti-Windup (Integration
#      gestoppt, wenn der Ausgang am Anschlag ist).
#   6. Ruhezone: |e| < e_hold und |v| < v_hold -> u = I (Haftreibung hält den Ball), I lernt
#      dort langsam weiter (ki_hold), damit der Ball in der Mulde nicht abwandert.
#      Verlassen erst nach n_out Werten mit |e| > e_out.
#   7. Ausgang begrenzt (u_min..u_max) und Änderung pro Schritt begrenzt (du_max).
# Nur Grundtypen, läuft auf MicroPython.

from bob_core import MITTE_MM


class Regler:
    def __init__(self, kp=0.8, kd=0.55, ki=0.15, a_br=120.0, v_max=400.0,
                 alpha=0.55, beta=0.2, l3=0.0, kn=8.5, g_ff=0.0, d_max=250.0, t_lead=0.1,
                 fc=4.0, v_fc=25.0, kick0=3.0, kick_rate=12.0, kick_max=30.0, v_stuck=40.0,
                 t_sw=0.0, t_back=0.4, p_min=4.0, kp_hold=0.0, ki_hold=0.3, t_uf=0.0,
                 i_zone=25.0, i_max=12.0,
                 e_hold=5.0, e_out=7.0, v_hold=20.0, n_out=3, r_max=15.0,
                 u_min=-31.0, u_max=28.0, du_max=6.0, ziel=MITTE_MM):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.a_br = a_br
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.l3 = l3
        self.kn = kn
        self.g_ff = g_ff
        self.d_max = d_max
        self.t_lead = t_lead
        self.fc = fc
        self.v_fc = v_fc
        self.kick0 = kick0
        self.kick_rate = kick_rate
        self.kick_max = kick_max
        self.v_stuck = v_stuck
        self.t_sw = t_sw
        self.t_back = t_back
        self.p_min = p_min
        self.kp_hold = kp_hold
        self.ki_hold = ki_hold
        self.t_uf = t_uf
        self.i_zone = i_zone
        self.i_max = i_max
        self.e_hold = e_hold
        self.e_out = e_out
        self.v_hold = v_hold
        self.n_out = n_out
        self.r_max = r_max
        self.u_min = u_min
        self.u_max = u_max
        self.du_max = du_max
        self.ziel = ziel
        self.reset(ziel)

    def reset(self, x):
        self.xf = x
        self.v = 0.0
        self.d = 0.0
        self.i = 0.0
        self.u = 0.0
        self.t_stuck = 0.0
        self.t_rev = 0.0
        self.hold = False
        self.n_aus = 0
        self.x_ref = x
        self.uf = 0.0

    def step(self, x, ok, dt):
        # 1. Beobachter
        a = self.d
        if self.l3 > 0.0:
            a += self.kn * self.u
        xp = self.xf + self.v * dt + 0.5 * a * dt * dt
        vp = self.v + a * dt
        if ok:
            r = x - xp
            if r > self.r_max:
                r = self.r_max
            elif r < -self.r_max:
                r = -self.r_max
            self.xf = xp + self.alpha * r
            self.v = vp + self.beta * r / dt
            if self.l3 > 0.0:
                d = self.d + self.l3 * r / (dt * dt)
                if d > self.d_max:
                    d = self.d_max
                elif d < -self.d_max:
                    d = -self.d_max
                self.d = d
        else:
            self.xf = xp
            self.v = vp * 0.9
        v = self.v
        e = self.ziel - (self.xf + v * self.t_lead)
        ae = e if e >= 0 else -e
        av = v if v >= 0 else -v

        # 6. Ruhezone mit Hysterese
        if self.hold:
            if ae > self.e_out:
                self.n_aus += 1
                if self.n_aus >= self.n_out:
                    self.hold = False
            else:
                self.n_aus = 0
        elif ae < self.e_hold and av < self.v_hold:
            self.hold = True
            self.n_aus = 0
            self.u_hold = self.i
            if self.t_uf > 0.0:
                # Haltewinkel = gemittelter Stellwinkel kurz vor Ankunft (Ball langsam -> Gleichgewicht)
                self.u_hold = self.uf
                self.i = self.uf

        if self.hold:
            if self.ki_hold > 0.0:
                self.i += self.ki_hold * e * dt
                if self.i > self.i_max:
                    self.i = self.i_max
                elif self.i < -self.i_max:
                    self.i = -self.i_max
                self.u_hold = self.i
            if self.t_uf > 0.0:
                # Haltewinkel = gemittelter Stellwinkel kurz vor Ankunft (Ball langsam -> Gleichgewicht)
                self.u_hold = self.uf
                self.i = self.uf
            u = self.u_hold + self.kp_hold * e
            self.t_stuck = 0.0
            self.t_rev = 0.0
            self.x_ref = self.xf
        elif self.t_rev > 0.0:
            # 4b. Schwung holen: voll weg vom Ziel
            self.t_rev -= dt
            u = self.u_min if e > 0 else self.u_max
            if self.t_rev <= 0.0:
                self.t_rev = 0.0
                self.t_stuck = self.t_sw * 0.5
        else:
            # 2. Sollwertführung, für kleine e reiner PD
            vr = self.kp / self.kd * ae
            vb = (2.0 * self.a_br * ae) ** 0.5
            if vb < vr:
                vr = vb
            if self.v_max < vr:
                vr = self.v_max
            if e < 0:
                vr = -vr
            u = self.kd * (vr - v)
            # 3. Reibung: in Bewegungsrichtung, weich
            s = v / self.v_fc
            if s > 1.0:
                s = 1.0
            elif s < -1.0:
                s = -1.0
            u += self.fc * s
            # 1b. Vorsteuerung der geschätzten Störung
            if self.g_ff > 0.0:
                u -= self.g_ff * self.d / self.kn
            # 4. Losbrechen: kein Fortschritt Richtung Ziel um p_min seit x_ref
            pr = self.xf - self.x_ref
            if e < 0:
                pr = -pr
            if pr > self.p_min or av > self.v_stuck:
                self.x_ref = self.xf
                self.t_stuck = 0.0
            elif ae > self.e_hold:
                self.t_stuck += dt
                k = self.kick0 + self.kick_rate * self.t_stuck
                if k > self.kick_max:
                    k = self.kick_max
                u += k if e > 0 else -k
                if self.t_sw > 0.0 and self.t_stuck > self.t_sw:
                    self.t_rev = self.t_back
                    self.t_stuck = 0.0
            # 5. I-Anteil nur nahe Ziel, Anti-Windup
            if ae < self.i_zone:
                ui = u + self.i
                if self.u_min < ui < self.u_max:
                    self.i += self.ki * e * dt
                    if self.i > self.i_max:
                        self.i = self.i_max
                    elif self.i < -self.i_max:
                        self.i = -self.i_max
            u += self.i

        # 7. Begrenzung
        if u > self.u_max:
            u = self.u_max
        elif u < self.u_min:
            u = self.u_min
        du = u - self.u
        if du > self.du_max:
            u = self.u + self.du_max
        elif du < -self.du_max:
            u = self.u - self.du_max
        self.u = u
        if self.t_uf > 0.0:
            self.uf += (u - self.uf) * dt / self.t_uf
        return u
