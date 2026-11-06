"""
sim.py: Gemeinsamer Simulator Ball on Beam (PC, CPython). Nicht für den Pico.

Bildet die Messungen aus Erkenntnisse-Versuchsaufbau.md Kap. 9 ab und führt den
GLEICHEN Kern (Pico_Programme/bob_core.py: Messung, Vorbereitung, Bewertung) aus wie der Pico.
Regler: Pico_Programme/ oder Methoden_Agenten/ctrl_<name>.py, oder beliebige Klasse mit reset(x) / step(x, ok, dt).

Benutzung:
    from sim import evaluate
    res = evaluate(lambda: Regler(), n=40, seed0=0)       # Abstimmen: seed0 0..9999
    res = evaluate(lambda: Regler(), n=100, seed0=100000)  # Bewerten: seed0 >= 100000
    print(res['summary'])

    python sim.py ctrl_pid           # Kurzbewertung eines Reglers (Modulname)

Alle Zufallsgrössen pro Lauf in Plant.__init__ (Bereich siehe Kommentare).
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                     # Ball_on_beam
for _sub in ("Methoden_Agenten", "Pico_Programme"):   # Regler + bob_core/kennlinie
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import bob_core as core              # noqa: E402
from kennlinie import TABELLE        # noqa: E402

# Servo -> Schiene (°, gegenüber waagrecht), Erkenntnisse Kap. 5.1
SERVO_TAB = [(-30, -4.05), (-25, -3.51), (-20, -2.91), (-15, -2.32), (-10, -1.66), (-5, -0.97),
             (0, -0.27), (5, 0.41), (10, 1.18), (15, 1.89), (20, 2.55), (25, 3.19), (30, 3.89)]
# Rauschen (Sensor-Einheiten) nach Position, Kap. 4.1
NOISE_TAB = [(0, 1.5), (100, 1.4), (150, 2.0), (200, 2.5), (250, 3.1), (300, 4.2), (339, 3.6)]


def interp(tab, x):
    if x <= tab[0][0]:
        (x0, y0), (x1, y1) = tab[0], tab[1]
    elif x >= tab[-1][0]:
        (x0, y0), (x1, y1) = tab[-2], tab[-1]
    else:
        i = 0
        while x > tab[i + 1][0]:
            i += 1
        (x0, y0), (x1, y1) = tab[i], tab[i + 1]
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)


def rail(servo):
    return interp(SERVO_TAB, servo)


KENN_MM_TO_S = [(x, s) for (s, x) in TABELLE]   # (mm, Sensorwert)
CIRC = math.pi * 40.0                           # Balldurchmesser 40 mm
SUB = 0.004                                     # s Integrationsschritt


class Plant:
    """Strecke + Sensor. Zeit in s, Position x = Ballvorderkante in mm (0..339)."""

    def __init__(self, rng, x0=None, level_mode='random'):
        R = rng.uniform
        self.rng = rng
        # Offset: Servowinkel, bei dem die Schiene im Mittel waagrecht ist
        self.off = 1.9 + R(-3.0, 3.0) if level_mode == 'random' else 1.9   # v3: Fits -2.5..+3
        # Ballbeschleunigung 10.1 mm/s² pro ° Servo nahe Mitte (±20 %), umgerechnet auf ° Schiene
        # Scan 06.11.: Fit ergab waagrecht ca. +0.5° relativ zu 1.9, im Bereich ±1.5 enthalten
        k_servo = R(10.0, 16.0)                    # v3: Regression echte Läufe 12.4..15.2
        slope = (rail(self.off + 1) - rail(self.off - 1)) / 2.0
        self.k_rail = k_servo / slope              # mm/s² pro ° Schiene
        self.rail0 = rail(self.off)
        # Reibung (Scan 06.11.: Fit 13 mm/s², Erkenntnisse 25 mm/s² inkl. Ballstörung)
        self.mu_r = R(15, 35)                      # mm/s² Rollreibung
        self.mu_s = self.mu_r * R(1.0, 1.8)        # Haftreibung (Losbrechen)
        # Höhenprofil der Stäbe: Zusatzneigung ° Schiene, gesamt ca. ±0.2°
        self.prof = [(R(0, 0.09), R(60, 400), R(0, 2 * math.pi)) for _ in range(3)]
        # Ball mit Schwerpunkt neben der Mitte (Scan 06.11.: Fit ±157 mm/s², 109 mm pro Umdrehung).
        # Störung hängt vom Drehwinkel ab = Position/r_eff + Phase (rollt ohne Rutschen).
        # Scan 2 (Ball seitlich gelegt): ±86 mm/s², 96 mm. Bereich deckt beide Fits ab.
        self.circ = R(60, 120)                     # mm Rollweg pro Umdrehung
        self.wob = (R(30, 180), R(0, 2 * math.pi), R(0, 25), R(0, 2 * math.pi))
        # Anschläge: Rückprall
        self.e_s = R(0.05, 0.3)
        self.e_k = R(0.15, 0.45)
        # Servo
        self.tau = R(0.025, 0.05)
        self.rate = R(300, 600)                    # °/s
        self.dead = R(0.05, 0.09)                  # s bis PWM-Wert wirkt (v3: Fit gesamt ca. 0.12 s)
        # Sensor
        self.T_s = R(0.0285, 0.0300)               # s pro ping (ca. 34 Hz)
        self.lag = R(0.008, 0.016)                 # s Messung bildet Position von früher ab
        self.kenn_err = [(x, s + rng.gauss(0, 2.5)) for (x, s) in KENN_MM_TO_S]
        self.noise_k = R(0.9, 1.3)
        self.p_out = 0.003                         # zufällige Fehlmessung
        self.fault_until = -1.0
        # Zustand
        self.t = 0.0
        self.x = x0 if x0 is not None else 0.0
        self.v = 0.0
        self.roll = 0.0                            # mm Rollweg (vorzeichenbehaftet)
        self.th = self.off                         # Servo-Ist
        self.cmd = [(0.0, self.off)]               # (ab Zeit, Sollwert)
        self.c_now = self.off
        self.hist = []                             # (t, x) für Sensor-Lag
        self.th_hist = []                          # (t, th) für Sprung-Erkennung
        self.trace = []                            # (t, x_true) pro ping

    def accel_drive(self):
        x = self.x
        ang = rail(self.th) - self.rail0
        for a, L, ph in self.prof:
            ang += a * math.sin(2 * math.pi * x / L + ph)
        acc = self.k_rail * ang
        a1, p1, a2, p2 = self.wob
        ph = 2 * math.pi * self.roll / self.circ
        acc += a1 * math.sin(ph + p1) + a2 * math.sin(2 * ph + p2)
        return acc

    def step(self, dt):
        # Servo-Sollwert mit Totzeit
        while len(self.cmd) > 1 and self.cmd[1][0] <= self.t:
            self.cmd.pop(0)
        c = self.cmd[0][1]
        d = (c - self.th) / self.tau
        d = max(-self.rate, min(self.rate, d))
        self.th += d * dt
        # Ball
        a = self.accel_drive()
        v = self.v
        if v == 0.0:
            if abs(a) > self.mu_s:
                v = (a - math.copysign(self.mu_r, a)) * dt
        else:
            vn = v + (a - math.copysign(self.mu_r, v)) * dt
            v = 0.0 if vn * v < 0 else vn
        x = self.x + v * dt
        self.roll += v * dt
        if x < 0.0:
            x = 0.0
            if v != 0.0:
                self.roll += self.rng.gauss(0, 2.0)  # kleiner Rutsch beim Anschlag
            v = -v * self.e_s
            if abs(v) < 15:
                v = 0.0
        elif x > core.X_END:
            x = core.X_END
            if v != 0.0:
                self.roll += self.rng.gauss(0, 2.0)
            v = -v * self.e_k
            if abs(v) < 15:
                v = 0.0
        self.x, self.v = x, v
        self.t += dt

    def advance(self, T):
        t_end = self.t + T
        while self.t < t_end - 1e-9:
            dt = min(SUB, t_end - self.t)
            self.step(dt)
            self.hist.append((self.t, self.x))
            self.th_hist.append((self.t, self.th))
        cut = self.t - 0.3
        while self.hist and self.hist[0][0] < cut:
            self.hist.pop(0)
        while self.th_hist and self.th_hist[0][0] < cut:
            self.th_hist.pop(0)

    def sense(self):
        tl = self.t - self.lag
        x = self.x
        for (t, xx) in reversed(self.hist):
            if t <= tl:
                x = xx
                break
        rng = self.rng
        # Schiene schwingt nach grossem Servosprung -> falsche Werte 100..200 ms
        ths = [th for (t, th) in self.th_hist if t >= self.t - 0.12]
        if ths and max(ths) - min(ths) >= 20.0 and self.fault_until < self.t:
            self.fault_until = self.t + rng.uniform(0.1, 0.2)
        s = interp(self.kenn_err, x) + rng.gauss(0, interp(NOISE_TAB, x) * self.noise_k)
        if self.t < self.fault_until and rng.random() < 0.6:
            s = 8190 if rng.random() < 0.1 else s + rng.uniform(-90, 90)
        elif rng.random() < self.p_out:
            s = 8190 if rng.random() < 0.5 else s + rng.uniform(-60, 60)
        self.trace.append((self.t, self.x))
        return int(round(s))


class SimIO:
    """io-Objekt für bob_core: ping blockiert eine Messperiode."""

    def __init__(self, plant, rng):
        self.p = plant
        self.rng = rng

    def ping(self):
        self.p.advance(self.p.T_s + self.rng.uniform(-0.0005, 0.0005))
        return self.p.sense()

    def zeit(self):
        return self.p.t

    def servo(self, c):
        c = max(-core.U_ABS, min(core.U_ABS, c))
        self.p.cmd.append((self.p.t + self.p.dead, c))
        self.p.c_now = c


def judge_true(trace, t0):
    """Dieselbe Ruhe-Bewertung wie bob_core, aber mit der WAHREN Position."""
    r = core.Ruhe()
    for (t, x) in trace:
        if t < t0:
            continue
        if r.update(x, t - t0):
            break
    return r.t_ok, r.verlassen


def run_one(make, seed, start=None, keep_trace=False):
    """Ein Versuch. start: 'sensor', 'klotz' oder Position in mm (Ball ruht dort)."""
    rng = random.Random(seed)
    if start is None:
        k = seed % 4
        if k == 0:
            start = 'sensor'
        elif k == 1:
            start = 'klotz'
        else:
            start = rng.uniform(0, 150) if k == 2 else rng.uniform(190, core.X_END)
    if isinstance(start, str):
        plant = Plant(rng, x0=core.MITTE_MM)
        st = start
    else:
        plant = Plant(rng, x0=start)
        st = None
    io = SimIO(plant, rng)
    reg = make()
    log = []
    t_start = [None]

    def lg(t, s, x, ok, u):
        if t_start[0] is None:
            t_start[0] = plant.t - t
        if keep_trace:
            log.append((round(t, 4), s, round(x, 1), ok, round(u, 2), round(plant.x, 1)))

    res = core.versuch(io, reg, st, core.OFFSET, lg)
    t0 = t_start[0]
    T_true, verl = judge_true(plant.trace, t0)
    res.update({'seed': seed, 'start': start, 'T_true': T_true, 'ok_true': T_true is not None,
                'verlassen_true': verl,
                'params': {'off': round(plant.off, 2), 'k_servo': round(plant.k_rail *
                           (rail(plant.off + 1) - rail(plant.off - 1)) / 2, 2),
                           'mu_r': round(plant.mu_r, 1), 'mu_s': round(plant.mu_s, 1)}})
    if keep_trace:
        res['trace'] = log
    return res


def _pct(vals, q):
    if not vals:
        return None
    v = sorted(vals)
    return v[min(len(v) - 1, int(q * (len(v) - 1) + 0.5))]


def evaluate(make, n=40, seed0=0, starts=None, verbose=False):
    """Monte Carlo. Bewertet mit der WAHREN Position (ok_true / T_true).
    Fail = kein Ruhe-Fenster bis 30 s. Zeiten über alle Läufe, Fail zählt als 30 s."""
    runs = []
    for i in range(n):
        st = starts[i % len(starts)] if starts else None
        r = run_one(make, seed0 + i, st)
        runs.append(r)
        if verbose:
            print(i, r['start'] if isinstance(r['start'], str) else round(r['start']),
                  r['ok_true'], r['T_true'] and round(r['T_true'], 2), r['params'])
    oks = [r for r in runs if r['ok_true']]
    Ts = [r['T_true'] if r['ok_true'] else core.T_FAIL for r in runs]
    s = {
        'n': n,
        'erfolg': len(oks),
        'quote': len(oks) / n,
        'T_median': _pct(Ts, 0.5),
        'T_p90': _pct(Ts, 0.9),
        'T_max': max(Ts),
        'unter_5s': sum(1 for t in Ts if t <= 5.0),
        'unter_10s': sum(1 for t in Ts if t <= 10.0),
        'servo_weg_pro_s': _pct([r['servo_weg'] / max(r['dauer'], 0.1) for r in runs], 0.5),
        'mess_vs_wahr_abweichend': sum(1 for r in runs if r['ok'] != r['ok_true']),
    }
    groups = {}
    for r in runs:
        g = r['start'] if isinstance(r['start'], str) else ('nah_sensor' if r['start'] < core.MITTE_MM else 'nah_klotz')
        groups.setdefault(g, []).append(r['T_true'] if r['ok_true'] else core.T_FAIL)
    s['pro_start'] = {g: {'n': len(v), 'erfolg': sum(1 for t in v if t < core.T_FAIL),
                          'T_median': _pct(v, 0.5), 'T_max': max(v)} for g, v in groups.items()}
    s['summary'] = ("Erfolg %d/%d | T median %.2f s, p90 %.2f s, max %.2f s | <=5 s: %d, <=10 s: %d | "
                    "Servo %.0f °/s | Mess!=Wahr %d" % (s['erfolg'], n, s['T_median'], s['T_p90'],
                                                       s['T_max'], s['unter_5s'], s['unter_10s'],
                                                       s['servo_weg_pro_s'], s['mess_vs_wahr_abweichend']))
    return {'summary': s['summary'], 'stats': s, 'runs': runs}


if __name__ == '__main__':
    import importlib
    mod = importlib.import_module(sys.argv[1] if len(sys.argv) > 1 else 'ctrl_pid')
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    res = evaluate(lambda: mod.Regler(), n=n, seed0=int(sys.argv[3]) if len(sys.argv) > 3 else 100000)
    print(res['summary'])
