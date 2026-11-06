# Demo, nur zum Test des Simulators
from bob_core import AlphaBeta, MITTE_MM
class Regler:
    def __init__(self, kp=0.25, kd=0.35, fc=3.0):
        self.kp, self.kd, self.fc = kp, kd, fc
        self.f = AlphaBeta(0.5, 0.15); self.i = 0.0
    def reset(self, x):
        self.f.reset(x); self.i = 0.0
    def step(self, x, ok, dt):
        x, v = self.f.update(x, ok, dt)
        e = MITTE_MM - x
        self.i = max(-8, min(8, self.i + 0.03*e*dt))
        u = self.kp*e - self.kd*v + self.i
        if abs(e) > 8 and abs(v) < 30:
            u += self.fc if e > 0 else -self.fc
        return u
