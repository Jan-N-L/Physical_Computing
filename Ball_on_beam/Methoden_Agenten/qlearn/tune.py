"""
tune.py: Training und Abstimmung des Q-Learning-Reglers (PC, CPython).

    python3 tune.py train  [cfg-json]   # trainiert mehrere Seeds, bewertet mit sim.evaluate (seed0 < 100000)
    python3 tune.py final               # Validierung seed0=100000, n=200 -> result.json

Training: eigene, schnelle Kopie der Strecke (FastPlant, gleiche Zufallsbereiche wie sim.Plant,
groberer Integrationsschritt, keine Listen). Domain Randomization: jede Episode neue Strecke.
Bewertung/Auswahl: nur sim.evaluate (seed0 0.. < 100000).
Lernregel: Q(s,a) += alpha*(r + gamma*max Q(s',.) - Q(s,a)), Entscheid nur jeden SKIP-ten Schritt
(SMDP, Kosten werden ueber die SKIP Schritte summiert). Spiegelung: nur e >= 0 wird gespeichert,
e < 0 wird gespiegelt (doppelte Daten pro Zelle, Tabelle symmetrisch).
"""
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for p in (os.path.join(ROOT, "pico"), os.path.join(ROOT, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import bob_core as core                      # noqa: E402
import sim                                   # noqa: E402
import ctrl_qlearn                           # noqa: E402

# ---------------------------------------------------------------- Standard-Konfiguration
CFG = dict(
    E_EDGES=(3, 7, 12, 20, 32, 50, 80, 120),                       # |e| mm (gespiegelt)
    V_EDGES=(-150, -90, -50, -25, -10, 10, 25, 50, 90, 150),       # mm/s
    ST_EDGES=(0.25, 0.8),                                          # s Steckzeit
    U_SET=(-28, -16, -9, -4, 0, 4, 9, 16, 28),                     # ° Servo (Zielwinkel)
    SKIP=3, RATE=150.0, AB_A=0.5, AB_B=0.15, V_ST=12.0, TOL_ST=6.0,
    U0=1.0, KI=0.0, I_ZONE=25.0, I_MAX=10.0,
    # Training
    EPISODES=16000, T_EP=8.0, GAMMA=0.98, EPS0=0.3, EPS1=0.02, ALPHA_MIN=0.02,
    W_OUT=1.0, W_VIN=0.3, W_E=0.3, W_DU=0.02, MIN_VISITS=3,
    # Vorwissen (Warmstart): PD-Tabelle u = -KP*e - KD*v - KICK*Steckstufe, Q0 ueberall, +BONUS fuer PD-Aktion,
    # N0 = Pseudo-Besuche (Lernrate startet bei 1/(N0+1)^0.6 statt 1). PRIOR=None: alles 0, N0=0.
    PRIOR=None, Q0=-40.0, BONUS=5.0, N0=5,
)
CKEYS = ('E_EDGES', 'V_EDGES', 'ST_EDGES', 'U_SET', 'SKIP', 'RATE', 'AB_A', 'AB_B', 'V_ST', 'TOL_ST',
         'U0', 'KI', 'I_ZONE', 'I_MAX')


# ---------------------------------------------------------------- schnelle Strecke
class FastPlant:
    """Gleiche Physik und Zufallsbereiche wie sim.Plant (Stand 06.11. mit unrundem Ball),
    groberer Integrationsschritt (sub), Sensor-Lag und Servo-Sprungfenster vereinfacht."""

    def __init__(self, rng, x0, v0=0.0, sub=0.006):
        R = rng.uniform
        self.rng = rng
        self.sub = sub
        self.off = 1.9 + R(-1.5, 3.5)
        k_servo = R(5.5, 12.0)
        slope = (sim.rail(self.off + 1) - sim.rail(self.off - 1)) / 2.0
        self.k_rail = k_servo / slope
        self.rail0 = sim.rail(self.off)
        self.mu_r = R(8, 30)
        self.mu_s = self.mu_r * R(1.0, 1.8)
        self.prof = [(R(0, 0.09), R(60, 400), R(0, 2 * math.pi)) for _ in range(3)]
        self.circ = R(92, 118)
        self.wob = (R(50, 180), R(0, 2 * math.pi), R(0, 25), R(0, 2 * math.pi))
        self.e_s = R(0.05, 0.3)
        self.e_k = R(0.15, 0.45)
        self.tau = R(0.02, 0.04)
        self.rate = R(300, 600)
        self.dead = R(0.005, 0.02)
        self.T_s = R(0.0285, 0.0300)
        self.lag = R(0.008, 0.016)
        self.kenn_err = [(x, s + rng.gauss(0, 2.5)) for (x, s) in sim.KENN_MM_TO_S]
        self.noise_k = R(0.9, 1.3)
        self.fault_until = -1.0
        self.t = 0.0
        self.x = x0
        self.v = v0
        self.roll = 0.0
        self.th = self.off
        self.cmd = [(0.0, self.off)]
        self.thmin = self.thmax = self.off
        self.t_win = 0.0

    def servo(self, c):
        c = max(-core.U_ABS, min(core.U_ABS, c))
        self.cmd.append((self.t + self.dead, c))

    def _acc(self):
        x = self.x
        ang = sim.rail(self.th) - self.rail0
        for a, L, ph in self.prof:
            ang += a * math.sin(2 * math.pi * x / L + ph)
        acc = self.k_rail * ang
        a1, p1, a2, p2 = self.wob
        ph = 2 * math.pi * self.roll / self.circ
        return acc + a1 * math.sin(ph + p1) + a2 * math.sin(2 * ph + p2)

    def advance(self, T):
        t_end = self.t + T
        x_lag = None
        t_lag = t_end - self.lag
        while self.t < t_end - 1e-9:
            dt = min(self.sub, t_end - self.t)
            cmd = self.cmd
            while len(cmd) > 1 and cmd[1][0] <= self.t:
                cmd.pop(0)
            d = (cmd[0][1] - self.th) / self.tau
            d = max(-self.rate, min(self.rate, d))
            self.th += d * dt
            a = self._acc()
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
                    self.roll += self.rng.gauss(0, 2.0)
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
            if x_lag is None and self.t >= t_lag:
                x_lag = x
            # Servo-Spanne im Fenster ~0.12 s (vereinfacht: Fenster neu alle 0.12 s)
            if self.th < self.thmin:
                self.thmin = self.th
            if self.th > self.thmax:
                self.thmax = self.th
        if self.t - self.t_win > 0.12:
            self.t_win = self.t
            self.thmin = self.thmax = self.th
        return x_lag if x_lag is not None else self.x

    def ping(self):
        rng = self.rng
        x = self.advance(self.T_s + rng.uniform(-0.0005, 0.0005))
        if self.thmax - self.thmin >= 20.0 and self.fault_until < self.t:
            self.fault_until = self.t + rng.uniform(0.1, 0.2)
        s = sim.interp(self.kenn_err, x) + rng.gauss(0, sim.interp(sim.NOISE_TAB, x) * self.noise_k)
        if self.t < self.fault_until and rng.random() < 0.6:
            s = 8190 if rng.random() < 0.1 else s + rng.uniform(-90, 90)
        elif rng.random() < 0.003:
            s = 8190 if rng.random() < 0.5 else s + rng.uniform(-60, 60)
        return int(round(s))


# ---------------------------------------------------------------- lernender Regler
class Lerner(ctrl_qlearn.Regler):
    """Gleicher Regler wie auf dem Pico, aber waehle() lernt (epsilon-greedy, Q-Update)."""

    def __init__(self, Q, N, cfg, rng):
        ctrl_qlearn.Regler.__init__(
            self, tab=b"", e_edges=cfg['E_EDGES'], v_edges=cfg['V_EDGES'], st_edges=cfg['ST_EDGES'],
            u_set=cfg['U_SET'], skip=cfg['SKIP'], rate=cfg['RATE'], ab_a=cfg['AB_A'], ab_b=cfg['AB_B'],
            v_st=cfg['V_ST'], tol_st=cfg['TOL_ST'], u0=cfg['U0'], ki=cfg['KI'], i_zone=cfg['I_ZONE'],
            i_max=cfg['I_MAX'])
        self.Q, self.N, self.cfg, self.rng = Q, N, cfg, rng
        self.eps = 0.1
        self.prev = None
        self.acc = 0.0
        self.learn = True
        self.n0 = cfg['N0'] if cfg.get('PRIOR') else 0

    def waehle(self, idx):
        na = self.na
        Q = self.Q
        base = idx * na
        row = Q[base:base + na]
        best = max(row)
        if self.prev is not None and self.learn:
            pi = self.prev
            n = self.N[pi] + 1
            self.N[pi] = n
            al = max(self.cfg['ALPHA_MIN'], 1.0 / (n + self.n0) ** 0.6)
            Q[pi] += al * (-self.acc + self.cfg['GAMMA'] * best - Q[pi])
        self.acc = 0.0
        if self.rng.random() < self.eps:
            a = self.rng.randrange(na)
        else:
            a = row.index(best)
        self.prev = base + a
        return a


def episode(lr, rng, cfg):
    r = rng.random()
    if r < 0.3:
        x0, v0 = (0.0 if rng.random() < 0.5 else core.X_END), 0.0
    elif r < 0.8:
        x0, v0 = rng.uniform(0, core.X_END), 0.0
    else:
        x0, v0 = rng.uniform(20, core.X_END - 20), rng.uniform(-150, 150)
    p = FastPlant(rng, x0, v0)
    m = core.Messung()
    m.x = x0 + rng.gauss(0, 2)
    lr.reset(m.x)
    lr.prev = None
    lr.acc = 0.0
    t = 0.0
    u_old = 0.0
    W_OUT, W_VIN, W_E, W_DU = cfg['W_OUT'], cfg['W_VIN'], cfg['W_E'], cfg['W_DU']
    M = core.MITTE_MM
    while t < cfg['T_EP']:
        t0 = p.t
        s = p.ping()
        dt = p.t - t0
        t += dt
        x, ok = m.update(s, dt)
        u = lr.step(x, ok, dt)
        p.servo(core.OFFSET + u)
        e = abs(p.x - M)
        if e > core.TOL:
            c = W_OUT
        else:
            c = W_VIN * min(1.0, abs(p.v) / 30.0)
        c += W_E * e / 170.0 + W_DU * abs(u - u_old) / 10.0
        u_old = u
        lr.acc += c


def greedy_tab(Q, N, na, min_vis, dflt=None):
    ns = len(Q) // na
    out = bytearray(ns)
    for i in range(ns):
        best, ba = None, None
        for a in range(na):
            if N[i * na + a] >= min_vis and (best is None or Q[i * na + a] > best):
                best, ba = Q[i * na + a], a
        if ba is None:                            # nie besucht: Vorwissen, sonst kraeftig Richtung Mitte
            ba = dflt[i] if dflt is not None else 0
        out[i] = ba
    return bytes(out)


def train(seed, cfg, verbose=False):
    import hashlib
    key = hashlib.md5(repr((sorted((k, v) for k, v in cfg.items() if k not in ('SEEDS', 'WRITE')), seed)).encode()).hexdigest()[:12]
    cdir = os.path.join(HERE, "cache2")
    cpath = os.path.join(cdir, key + ".bin")
    if os.path.exists(cpath):
        with open(cpath, "rb") as f:
            return f.read(), None, None
    tab, Q, N = _train(seed, cfg, verbose)
    if not os.path.isdir(cdir):
        os.makedirs(cdir)
    with open(cpath, "wb") as f:
        f.write(tab)
    return tab, Q, N


def prior_tab(cfg):
    """Aktion einer einfachen PD-Regel an den Bin-Mitten (nur fuer den Warmstart)."""
    kp, kd, kick = cfg['PRIOR']
    us = cfg['U_SET']

    def mid(edges, i, lo, hi):
        e = [lo] + list(edges) + [hi]
        return 0.5 * (e[i] + e[i + 1])
    tab = bytearray()
    for ie in range(len(cfg['E_EDGES']) + 1):
        for iv in range(len(cfg['V_EDGES']) + 1):
            for ist in range(len(cfg['ST_EDGES']) + 1):
                e = mid(cfg['E_EDGES'], ie, 0, 170)
                v = mid(cfg['V_EDGES'], iv, -250, 250)
                u = -kp * e - kd * v - (kick * ist if e > 8 else 0)
                tab.append(min(range(len(us)), key=lambda a: abs(us[a] - u)))
    return bytes(tab)


def _train(seed, cfg, verbose=False):
    rng = random.Random(seed)
    nst = (len(cfg['E_EDGES']) + 1) * (len(cfg['V_EDGES']) + 1) * (len(cfg['ST_EDGES']) + 1)
    na = len(cfg['U_SET'])
    Q = [0.0] * (nst * na)
    N = [0] * (nst * na)
    if cfg.get('PRIOR'):
        pt = prior_tab(cfg)
        for i in range(nst):
            for a in range(na):
                Q[i * na + a] = cfg['Q0'] + (cfg['BONUS'] if pt[i] == a else 0.0)
    lr = Lerner(Q, N, cfg, rng)
    E = cfg['EPISODES']
    for ep in range(E):
        f = ep / E
        lr.eps = cfg['EPS0'] + (cfg['EPS1'] - cfg['EPS0']) * f
        episode(lr, rng, cfg)
        if verbose and ep % 5000 == 0:
            print("  ep", ep, flush=True)
    dflt = prior_tab(cfg) if cfg.get('PRIOR') else None
    return greedy_tab(Q, N, na, cfg['MIN_VISITS'], dflt), Q, N


def make_factory(tab, cfg):
    def mk():
        return ctrl_qlearn.Regler(tab=tab, e_edges=cfg['E_EDGES'], v_edges=cfg['V_EDGES'],
                                  st_edges=cfg['ST_EDGES'], u_set=cfg['U_SET'], skip=cfg['SKIP'],
                                  rate=cfg['RATE'], ab_a=cfg['AB_A'], ab_b=cfg['AB_B'],
                                  v_st=cfg['V_ST'], tol_st=cfg['TOL_ST'], u0=cfg['U0'], ki=cfg['KI'], i_zone=cfg['I_ZONE'],
            i_max=cfg['I_MAX'])
    return mk


def score(st):
    """Auswahlnote: Zuverlaessigkeit vor Median."""
    return st['quote'] * 100 - st['T_p90'] - 0.3 * st['T_max'] - 0.5 * st['T_median']


def write_tab(tab, cfg, path, kommentar=""):
    keys = CKEYS
    with open(path, "w") as f:
        f.write("# ctrl_qlearn_tab.py: gelernte Q-Tabelle (nur beste Aktion pro Zustand).\n")
        f.write("# Erzeugt von methods/qlearn/tune.py. Nicht von Hand aendern.\n")
        if kommentar:
            f.write("# %s\n" % kommentar)
        for k in keys:
            f.write("%s = %r\n" % (k, cfg[k]))
        f.write("# Index = (ie * len(V_EDGES)+1 + iv) * (len(ST_EDGES)+1) + ist, Wert = Index in U_SET\n")
        f.write("TAB = (\n")
        for i in range(0, len(tab), 48):
            f.write("    %r\n" % tab[i:i + 48])
        f.write(")\n")


def run_cfg(cfg, seeds, n_eval=80, seed0=0, verbose=True):
    best = None
    for sd in seeds:
        t0 = time.time()
        tab, Q, N = train(sd, cfg)
        res = sim.evaluate(make_factory(tab, cfg), n=n_eval, seed0=seed0)
        sc = score(res['stats'])
        if verbose:
            print("seed %d  %.0fs  note %.1f  %s" % (sd, time.time() - t0, sc, res['summary']), flush=True)
            print("     ", {g: (v['erfolg'], round(v['T_median'], 1), round(v['T_max'], 1))
                            for g, v in sorted(res['stats']['pro_start'].items())}, flush=True)
        if best is None or sc > best[0]:
            best = (sc, sd, tab, res)
    return best


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'train'
    cfg = dict(CFG)
    if len(sys.argv) > 2:
        cfg.update(json.loads(sys.argv[2]))
    if mode == 'train':
        seeds = cfg.pop('SEEDS', (1, 2, 3))
        best = run_cfg(cfg, seeds)
        print("BESTE seed", best[1], best[3]['summary'])
        if cfg.get('WRITE'):
            write_tab(best[2], cfg, os.path.join(ROOT, "pico", "ctrl_qlearn_tab.py"),
                      "Trainings-Seed %d, Auswahl mit sim.evaluate n=80 seed0=0" % best[1])
    elif mode == 'final':
        import ctrl_qlearn_tab  # noqa: F401
        res = sim.evaluate(lambda: ctrl_qlearn.Regler(), n=200, seed0=100000)
        st = dict(res['stats'])
        print(res['summary'])
        print(json.dumps(st['pro_start'], indent=1))
        with open(os.path.join(HERE, "result.json"), "w") as f:
            json.dump(st, f, indent=1)
