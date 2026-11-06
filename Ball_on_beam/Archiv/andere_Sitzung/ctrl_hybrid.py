# ctrl_hybrid.py: Schnell hin (MPC aus ctrl_kombi), ruhig halten (sanfter PID aus ctrl_ruhig).
#  Fern:  MPC mit Störungsschätzung, bringt den Ball in ca. 2 s in Zielnähe.
#  Nah:   ab |e| < e_nah und |v| < v_nah übernimmt der sanfte Regler (stossfrei: übernimmt
#         den aktuellen Winkel als I-Anteil), friert den Servo ein, sobald der Ball ruht.
#  Zurück zur MPC erst, wenn der Ball weiter als e_fern vom Ziel weg ist.
from bob_core import MITTE_MM
import ctrl_kombi
import ctrl_ruhig


class Regler:
    def __init__(self, e_nah=15.0, v_nah=40.0, e_fern=35.0, mpc=None, fein=None):
        self.e_nah = e_nah
        self.v_nah = v_nah
        self.e_fern = e_fern
        self.mpc = ctrl_kombi.Regler(**(mpc or {}))
        self.fein = ctrl_ruhig.Regler(**(fein or {}))
        self.reset(MITTE_MM)

    def reset(self, x):
        self.mpc.reset(x)
        self.fein.reset(x)
        self.nah = False

    def step(self, z, ok, dt):
        u_m = self.mpc.step(z, ok, dt)       # läuft immer mit (Schätzer bleibt aktuell)
        f = self.fein
        if self.nah:
            u = f.step(z, ok, dt)
            e = MITTE_MM - f.x
            if e > self.e_fern or e < -self.e_fern:
                self.nah = False
                self.mpc.uc = u
            return u
        e = MITTE_MM - self.mpc.x
        v = self.mpc.v
        if -self.e_nah < e < self.e_nah and -self.v_nah < v < self.v_nah:
            # stossfreie Übergabe
            self.nah = True
            f.x = self.mpc.x
            f.v = v
            f.u = u_m
            f.i = u_m - f.kp_fein * e + f.kd * v
            f.halt = False
            f.t_ruhig = 0.0
            f.t_fest = 0.0
            f.kick = 0.0
            return u_m
        f.u = u_m
        return u_m
