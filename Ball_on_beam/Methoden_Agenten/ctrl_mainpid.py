# ctrl_mainpid.py: Jans Scripts/main.py 1:1 als Regler-Klasse (nur zum Vergleich am Aufbau).
# main.py rechnet in Sensorwerten (Soll 200) mit Offset 0.5. Hier: Messung kommt als mm,
# wird über die Kennlinie zurück in Sensorwerte gerechnet. Ausgabe relativ zu bob_core.OFFSET.
from kennlinie import to_sensor
from bob_core import OFFSET

x_set = 200
TOL = 7.5
V_TOL = 15
T_STEADY = 3.0
KP_FINE = 0.10
KP_COARSE = 0.15
E_BIG = 50
ki = 0.02
kd = 0.08
I_MAX = 10
FC = 2
ALPHA = 0.3
BETA = 0.35
OFF_MAIN = 0.5


class Regler:
    def __init__(self):
        self.reset(170.0)

    def reset(self, x):
        self.e = x_set - to_sensor(x)
        self.de = 0.0
        self.i = 0.0
        self.hold = False
        self.t_hold = 0.0
        self.steady = False

    def step(self, x, ok, dt):
        e_raw = x_set - to_sensor(x)
        e_old = self.e
        e = ALPHA * (e_raw if ok else self.e) + (1 - ALPHA) * (self.e + self.de * dt)
        self.de = BETA * (e - e_old) / dt + (1 - BETA) * self.de
        self.e = e
        de = self.de
        if self.steady:
            if abs(e) < 3 * TOL:
                return OFF_MAIN - OFFSET          # setAngle(0)
            self.steady = False
            self.hold = False
            self.t_hold = 0.0
        hold = abs(e) < TOL and abs(de) < (2 * V_TOL if self.hold else V_TOL)
        self.hold = hold
        if not hold:
            self.t_hold = 0.0
        else:
            self.t_hold += dt
            if self.t_hold > T_STEADY:
                self.steady = True
                return OFF_MAIN - OFFSET
        if hold:
            p = d = 0.0
        else:
            p = (KP_FINE + (KP_COARSE - KP_FINE) * min(abs(e) / E_BIG, 1)) * e
            if abs(e) > TOL:
                p += FC if e > 0 else -FC
            self.i = max(-I_MAX, min(I_MAX, self.i + ki * e * dt))
            d = kd * de
        u = p + self.i + d
        return u + OFF_MAIN - OFFSET
