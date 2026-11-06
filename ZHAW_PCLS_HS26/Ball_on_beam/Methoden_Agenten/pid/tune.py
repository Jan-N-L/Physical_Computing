"""tune.py: Reproduzierbares Abstimmen von pico/ctrl_pid.py im neuen Simulator (Ball mit schwerer Seite).

Vorgehen (nur Seeds < 100000):
  1. Koordinatensuche: pro Parameter einige Kandidaten, n=N_TUNE Läufe ab SEED_TUNE.
     Übernommen wird nur, was die Kosten senkt. ROUNDS Runden.
  2. Kontrolle auf frischen Seeds (SEED_CHECK, n=N_CHECK): Endwert gegen Varianten
     (Filter, Rampe/Sollwertführung, I nur nahe Ziel, Vorsteuerung der Ballstörung).
Kosten = 0.5*Median + p90 + 0.25*max + 10*Fails pro 40 Läufe (Zeiten in s, Fail = 30 s).

Aufruf:  python3 tune.py            (Suche + Kontrolle)
         python3 tune.py check      (nur Kontrolle)
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BOB = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(BOB, "sim"))
sys.path.insert(0, os.path.join(BOB, "pico"))

from sim import evaluate          # noqa: E402
import ctrl_pid                   # noqa: E402

N_TUNE = 40
SEED_TUNE = 0
N_CHECK = 120
SEED_CHECK = 50000
ROUNDS = 2

START = dict(i_max=12.0, t_sw=0.0, kp=0.6, t_lead=0.05, fc=2.0, ki_hold=0.0, e_out=9.0, i_zone=25.0)
# (alte Standardwerte explizit, damit die Suche reproduzierbar bleibt; Ergebnis BEST = Standardwerte in ctrl_pid.py)

GRID = [
    ('kp', [0.3, 0.4, 0.5, 0.6, 0.8]),
    ('kd', [0.35, 0.45, 0.55, 0.7]),
    ('a_br', [60.0, 90.0, 120.0, 200.0]),
    ('v_max', [150.0, 250.0, 400.0]),
    ('beta', [0.1, 0.15, 0.2, 0.3]),
    ('t_lead', [0.0, 0.05, 0.1]),
    ('fc', [0.0, 2.0, 4.0, 6.0]),
    ('kick0', [0.0, 3.0, 6.0]),
    ('kick_rate', [6.0, 12.0, 25.0, 50.0]),
    ('p_min', [2.0, 4.0, 8.0]),
    ('ki', [0.05, 0.15, 0.3, 0.6]),
    ('i_zone', [15.0, 25.0, 40.0]),
    ('i_max', [8.0, 12.0, 20.0]),
    ('e_hold', [3.0, 5.0, 7.0]),
    ('e_out', [7.0, 9.0, 12.0]),
    ('v_hold', [10.0, 20.0, 35.0]),
    ('ki_hold', [0.0, 0.1, 0.3]),
    ('t_uf', [0.0, 0.3, 0.6]),
    ('t_sw', [0.0, 1.0, 2.0]),
    ('du_max', [4.0, 6.0, 10.0]),
]


def cost(s):
    return 0.5 * s['T_median'] + s['T_p90'] + 0.25 * s['T_max'] + 10.0 * (s['n'] - s['erfolg']) * 40.0 / s['n']


def run(p, n=N_TUNE, seed0=SEED_TUNE):
    r = evaluate(lambda: ctrl_pid.Regler(**p), n=n, seed0=seed0)
    return cost(r['stats']), r


def search(p, rounds=ROUNDS):
    best, r = run(p)
    print('Start', round(best, 2), r['summary'], flush=True)
    for rd in range(rounds):
        for name, cands in GRID:
            for c in cands:
                if p.get(name) == c:
                    continue
                q = dict(p)
                q[name] = c
                k, r = run(q)
                if k < best - 0.5:
                    best, p = k, q
                    print('  R%d %s=%s -> %.2f | %s' % (rd, name, c, k, r['summary']), flush=True)
        print('Runde', rd, 'fertig:', round(best, 2), p, flush=True)
    return p


def check(variants):
    print('Kontrolle auf Seeds %d.., n=%d' % (SEED_CHECK, N_CHECK))
    for name, p in variants:
        k, r = run(p, n=N_CHECK, seed0=SEED_CHECK)
        print('%-26s Kosten %6.1f | %s' % (name, k, r['summary']), flush=True)


if __name__ == '__main__':
    t0 = time.time()
    if len(sys.argv) > 1 and sys.argv[1] == 'check':
        best = {}
    else:
        best = search(dict(START))
        print('BEST =', best, flush=True)
    check([('Endwert', best)])
    print('Zeit %.0f s' % (time.time() - t0))
