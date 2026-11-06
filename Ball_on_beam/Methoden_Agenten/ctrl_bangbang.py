# ctrl_bangbang.py: Bang-Bang-Regelung mit Schaltkurve (Ball on Beam), Version 2
# Läuft auf MicroPython (Pico) und im Simulator. Nur Grundtypen.
#
# Drei Phasen:
#  1. Fern: zeitoptimal. Voll kippen (U) Richtung Ziel, auf der Bremskurve
#     s = e - v*T_LEAD - v*|v|/(2*a_b) voll gegenkippen. a_b = SAFETY*(K*U + MU).
#  2. Nah: Halteregler u = uh + KP*e - KD*v, uh = gelernter Haltewinkel (I-Anteil).
#     Nötig, weil der unrunde Ball lokal bis ±180 mm/s² (≈ ±15..30° Servo) stört.
#  3. Totband: Ball ruht in der Mitte -> Winkel einfrieren. Die Haftreibung hält ihn.
#
# Vorzeichen: e = MITTE - x, v = dx/dt, u > 0 -> Ball rollt zu grösserem x.
# Nach grossen Servosprüngen sind Messwerte verdächtig: grosse Abweichungen vom
# Schätzer werden dann kurz verworfen (nie dauerhaft, max. N_BAD in Folge).

from bob_core import MITTE_MM


def _sgn(v):
    return 1.0 if v > 0 else (-1.0 if v < 0 else 0.0)


class Regler:
    def __init__(self, U=25.0, K=8.0, MU=15.0, SAFETY=0.9, T_LEAD=0.12, RAMPE=4.5,
                 E_NAH=25.0, V_NAH=80.0, U_NAH=26.0, KP=1.5, KD=0.7, KI=1.5, UI_MAX=26.0,
                 E_HOLD=7.0, V_HOLD=10.0, T_HOLD=0.3, E_OUT=10.0, T_OUT=0.3, V_OUT=40.0,
                 ALPHA=0.5, BETA=0.15, GAMMA=0.02, D_MAX=250.0, FF=0.0, FF_FERN=0.0,
                 GATE=20.0, T_SUSP=0.2, D_SPRUNG=15.0, N_BAD=4,
                 E_IN=0.0, KD_IN=0.0, KP_IN=-1.0):
        self.U = U
        self.K = K
        self.MU = MU
        self.SAFETY = SAFETY
        self.T_LEAD = T_LEAD
        self.RAMPE = RAMPE
        self.E_NAH = E_NAH
        self.V_NAH = V_NAH
        self.U_NAH = U_NAH
        self.KP = KP
        self.KD = KD
        self.KI = KI
        self.UI_MAX = UI_MAX
        self.E_HOLD = E_HOLD
        self.V_HOLD = V_HOLD
        self.T_HOLD = T_HOLD
        self.E_OUT = E_OUT
        self.T_OUT = T_OUT
        self.V_OUT = V_OUT
        self.ALPHA = ALPHA
        self.BETA = BETA
        self.GAMMA = GAMMA      # Beobachter Störung d (mm/s²)
        self.D_MAX = D_MAX
        self.FF = FF            # Anteil Vorsteuerung -d/K nah
        self.FF_FERN = FF_FERN  # Anteil Vorsteuerung -d/K fern
        self.E_IN = E_IN        # mm Fangfenster: dort KD_IN / KP_IN statt KD / KP
        self.KD_IN = KD_IN
        self.KP_IN = KP_IN      # < 0: KP behalten
        self.GATE = GATE
        self.T_SUSP = T_SUSP
        self.D_SPRUNG = D_SPRUNG
        self.N_BAD = N_BAD
        self.reset(MITTE_MM)

    def reset(self, x):
        self.x = x
        self.v = 0.0
        self.d = 0.0
        self.u = 0.0
        self.uh = [0.0, 0.0, 0.0, 0.0]
        self.susp = 0.0
        self.n_bad = 0
        self.ui = 0.0           # Haltewinkel (gelernt)
        self.hold = False
        self.t_in = 0.0
        self.t_out = 0.0
        self.nah = False

    def _schaetzen(self, xm, ok, dt):
        # Modell: a = K*u (ca. 1..2 Schritte verzögert) + d, d = Sammelstörung
        a = self.K * 0.5 * (self.uh[1] + self.uh[2]) + self.d
        xp = self.x + self.v * dt + 0.5 * a * dt * dt
        vp = self.v + a * dt
        if ok:
            r = xm - xp
            if self.susp > 0 and abs(r) > self.GATE and self.n_bad < self.N_BAD:
                ok = False
                self.n_bad += 1
            else:
                self.n_bad = 0
        if ok:
            self.x = xp + self.ALPHA * r
            self.v = vp + self.BETA * r / dt
            d = self.d + self.GAMMA * r / (dt * dt)
            m = self.D_MAX
            self.d = m if d > m else (-m if d < -m else d)
        else:
            self.x = xp
            self.v = vp

    def step(self, xm, ok, dt):
        if dt <= 0:
            dt = 0.029
        if self.susp > 0:
            self.susp -= dt
        self._schaetzen(xm, ok, dt)
        x, v = self.x, self.v
        e = MITTE_MM - x
        ae, av = abs(e), abs(v)

        if self.hold:
            # Totband: Winkel bleibt, solange der Ball ruhig in der Mitte liegt
            if ae > self.E_OUT:
                self.t_out += dt
            else:
                self.t_out = 0.0
            if av > self.V_OUT or self.t_out > self.T_OUT:
                self.hold = False
                self.ui = self.u
            else:
                return self._out(self.u)
        # Phase nah / fern mit Hysterese
        if self.nah:
            if ae > 2.0 * self.E_NAH or av > 1.5 * self.V_NAH:
                self.nah = False
        elif ae < self.E_NAH and av < self.V_NAH:
            self.nah = True
            self.ui = self.u    # stossfrei: Haltewinkel startet beim aktuellen Winkel
            if self.ui > self.UI_MAX:
                self.ui = self.UI_MAX
            elif self.ui < -self.UI_MAX:
                self.ui = -self.UI_MAX

        if self.nah:
            ui = self.ui + self.KI * e * dt
            m = self.UI_MAX
            self.ui = m if ui > m else (-m if ui < -m else ui)
            kp, kd = self.KP, self.KD
            if ae < self.E_IN:
                kd = self.KD_IN
                if self.KP_IN >= 0:
                    kp = self.KP_IN
            uc = self.ui + kp * e - kd * v - self.FF * self.d / self.K
            m = self.U_NAH
            uc = m if uc > m else (-m if uc < -m else uc)
            # Ruhe erkennen -> einfrieren
            if ae < self.E_HOLD and av < self.V_HOLD:
                self.t_in += dt
                if self.t_in > self.T_HOLD:
                    self.hold = True
                    self.t_out = 0.0
                    return self._out(self.u)
            else:
                self.t_in = 0.0
        else:
            self.t_in = 0.0
            U = self.U
            ab = self.SAFETY * (self.K * U + self.MU)
            s = e - v * self.T_LEAD - v * av / (2.0 * ab)
            uc = (U if s > 0 else -U) - self.FF_FERN * self.d / self.K
        return self._out(uc)

    def _out(self, uc):
        d = uc - self.u
        if d > self.RAMPE:
            d = self.RAMPE
        elif d < -self.RAMPE:
            d = -self.RAMPE
        self.u += d
        uh = self.uh
        uh[3] = uh[2]
        uh[2] = uh[1]
        uh[1] = uh[0]
        uh[0] = self.u
        if abs(uh[0] - uh[3]) >= self.D_SPRUNG:
            self.susp = self.T_SUSP
        return self.u
