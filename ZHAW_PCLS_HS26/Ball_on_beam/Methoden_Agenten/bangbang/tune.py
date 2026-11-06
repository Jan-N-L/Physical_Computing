"""tune.py: Reproduzierbares Abstimmen der Bang-Bang-Regelung (ctrl_bangbang, Version 2).

Koordinatensuche: jeder Parameter einzeln über ein Raster, bester Wert bleibt.
Nur Abstimm-Seeds (seed0 < 100000). Zielgrösse (kleiner = besser):
    score = mittlere T + T_p90 + 0.3 * T_max      (Fail zählt als 30 s)

    python3 tune.py            # Suche (Seeds 2000..2039), danach Kontrolle Seeds 20000..20119
    python3 tune.py stufe2     # Fortsetzung ab Stufe 1 (Zeitbudget)
    python3 tune.py check      # nur Kontrolle der Standardwerte
    python3 tune.py final      # Validierung seed0=100000, n=200 -> result.json (nur einmal am Ende)
"""
import json
import os
import sys
import time
sys.path.insert(0, '/home/claude/bob/sim')
sys.path.insert(0, '/home/claude/bob/pico')
from sim import evaluate          # noqa: E402
import ctrl_bangbang as C         # noqa: E402

N, SEED0 = 40, 2000
HERE = os.path.dirname(os.path.abspath(__file__))

# Startpunkt aus Vorversuchen (Raster KP/KD/RAMPE/KI auf Seeds 1000..1023)
START = {'RAMPE': 4.5, 'FF': 0.0, 'KI': 1.5, 'KP': 2.0, 'KD': 1.0}

# Ergebnis Stufe 1 (tune_log.txt)
STUFE1 = {'RAMPE': 4.5, 'FF': 0.0, 'KI': 1.5, 'KP': 1.5, 'KD': 0.7, 'SAFETY': 0.9, 'T_HOLD': 0.3}
STUFE2 = ['V_HOLD', 'E_HOLD', 'T_HOLD', 'E_NAH', 'V_NAH', 'U', 'KI', 'GATE', 'RAMPE']

GRID = [
    ('KD', [0.7, 1.0, 1.4, 2.0]),
    ('KP', [1.5, 2.0, 2.5]),
    ('KI', [0.5, 1.5, 3.0]),
    ('U', [18, 22, 25]),
    ('SAFETY', [0.5, 0.7, 0.9]),
    ('T_LEAD', [0.08, 0.12, 0.2]),
    ('E_NAH', [15, 25, 40]),
    ('V_NAH', [50, 80, 120]),
    ('V_HOLD', [10, 15, 25]),
    ('E_HOLD', [5, 7, 9]),
    ('T_HOLD', [0.1, 0.15, 0.3]),
    ('RAMPE', [3.5, 4.5, 6.0]),
    ('FF', [0.0, 0.5]),
    ('GATE', [12, 20, 35]),
]


def run_score(kw, n=N, seed0=SEED0):
    r = evaluate(lambda: C.Regler(**kw), n=n, seed0=seed0)
    Ts = [x['T_true'] if x['ok_true'] else 30.0 for x in r['runs']]
    st = r['stats']
    sc = sum(Ts) / len(Ts) + st['T_p90'] + 0.3 * st['T_max']
    return sc, r['summary']


def search(kw, rounds=1, t_budget=900):
    t0 = time.time()
    best, summ = run_score(kw)
    print('Start %.3f %s' % (best, summ), flush=True)
    for rd in range(rounds):
        for name, vals in GRID:
            for v in vals:
                if time.time() - t0 > t_budget:
                    return kw
                if kw.get(name, getattr(C.Regler(), name)) == v:
                    continue
                k2 = dict(kw)
                k2[name] = v
                sc, sm = run_score(k2)
                if sc < best - 0.05:
                    best, kw, summ = sc, k2, sm
                    print('  R%d %s=%s -> %.3f %s' % (rd, name, v, sc, sm), flush=True)
        print('Runde %d: %.3f %s' % (rd, best, kw), flush=True)
    return kw


if __name__ == '__main__':
    arg = sys.argv[1] if len(sys.argv) > 1 else ''
    if arg == 'final':
        r = evaluate(lambda: C.Regler(), n=200, seed0=100000)
        print(r['summary'])
        with open(os.path.join(HERE, 'result.json'), 'w') as f:
            json.dump(r['stats'], f, indent=1)
        sys.exit()
    if arg == 'stufe2':
        # Stufe 2: ab Ergebnis Stufe 1, nur die Parameter, die Stufe 1 im Zeitbudget nicht erreichte
        GRID[:] = [g for g in GRID if g[0] in STUFE2]
        kw = search(dict(STUFE1), t_budget=600)
    else:
        kw = {} if arg == 'check' else search(dict(START))
    sc, sm = run_score(kw, n=120, seed0=20000)
    print('Kontrolle Seeds 20000..20119: %.3f %s' % (sc, sm))
    print('Endparameter:', kw)
