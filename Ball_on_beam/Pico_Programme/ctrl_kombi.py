# ctrl_kombi.py: Kombination der besten Methoden für Ball on Beam.
# Kern: Vorhersage-Regelung (MPC, ctrl_mpc.py) mit Online-Schätzung der Ballstörung.
# Dazu (abschaltbar, Standard siehe unten):
#   - Parameter abgestimmt auf den Simulator v3 (an echten Läufen geeicht: K ca. 14, Verzögerung ca. 0.12 s)
#   - Losbrech-Stoss aus Golfer/Fuzzy: steckt der Ball weit neben dem Ziel fest, wächst ein Zusatzwinkel
#     Richtung Ziel (kick_rate °/s bis kick_max), bis er rollt.
# u relativ zu OFFSET, positives u: Ball rollt zu grösseren x.
from bob_core import MITTE_MM
import ctrl_mpc

BEST = dict(wt=2.0, abrk=170.0, rate_u=200.0, l1=0.35, K=13.0, gs=0.15)


class Regler(ctrl_mpc.Regler):
    def __init__(self, kick_e=14.0, kick_t=1.2, kick_rate=15.0, kick_max=20.0, kick_v=10.0,
                 frz_e=0.0, frz_v=12.0, frz_n=4, frz_rel=9.0, **kw):
        p = dict(BEST)
        p.update(kw)
        self.kick_e = kick_e
        self.kick_t = kick_t
        self.kick_rate = kick_rate
        self.kick_max = kick_max
        self.kick_v = kick_v
        self.frz_e = frz_e        # Ruhezone (aus Fuzzy): 0 = aus
        self.frz_v = frz_v
        self.frz_n = frz_n
        self.frz_rel = frz_rel
        ctrl_mpc.Regler.__init__(self, **p)

    def reset(self, x):
        ctrl_mpc.Regler.reset(self, x)
        self.t_st = 0.0
        self.kick = 0.0
        self.n_frz = 0
        self.frz = None
        self.u_last = 0.0

    def step(self, z, ok, dt):
        u = ctrl_mpc.Regler.step(self, z, ok, dt)
        if self.frz_e > 0:
            e = MITTE_MM - self.x
            ae = e if e > 0 else -e
            v = self.v
            if self.frz is not None:
                if ae > self.frz_rel or v > 3 * self.frz_v or v < -3 * self.frz_v:
                    self.frz = None
                    self.n_frz = 0
                else:
                    self.uc = self.frz
                    return self.frz
            elif ae < self.frz_e and -self.frz_v < v < self.frz_v:
                self.n_frz += 1
                if self.n_frz >= self.frz_n:
                    self.frz = self.u_last
                    self.uc = self.frz
                    return self.frz
            else:
                self.n_frz = 0
        self.u_last = u
        if self.kick_rate <= 0:
            return u
        e = MITTE_MM - self.x
        v = self.v
        if (e > self.kick_e or e < -self.kick_e) and -self.kick_v < v < self.kick_v:
            self.t_st += dt
        else:
            self.t_st = 0.0
            self.kick = 0.0
        if self.t_st > self.kick_t:
            k = self.kick + self.kick_rate * dt
            self.kick = k if k < self.kick_max else self.kick_max
            u += self.kick if e > 0 else -self.kick
            u = ctrl_mpc.U_HI if u > ctrl_mpc.U_HI else ctrl_mpc.U_LO if u < ctrl_mpc.U_LO else u
            self.uc = u
        return u
