"""tune.py: Abstimmung Fuzzy-Regler (ctrl_fuzzy.py) im gemeinsamen Simulator.

Reproduzierbar: feste Seeds, deterministische Koordinatensuche.
Abstimmen nur mit seed0 < 100000:
  Suche:     seed0 = 0,    n = 80
  Kontrolle: seed0 = 5000, n = 200 (nur angeschaut, nicht optimiert)
Aufruf:  python3 tune.py [runden]     (ca. 4 min, Ergebnis siehe tune_log.txt)
         python3 tune.py validieren   (einmal ganz am Ende: seed0=100000, n=200 -> result.json)
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BOB = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(BOB, "sim"))
sys.path.insert(0, os.path.join(BOB, "pico"))

from sim import evaluate      # noqa: E402
import ctrl_fuzzy             # noqa: E402

# Startwerte = erste Auslegung von Hand (Bremskurve, Reibung 2.5°, Ruhezone ±9 mm).
# Die Suche geht von hier aus, nicht von den Standardwerten in ctrl_fuzzy.py.
# Zweite Auslegung (neue Sim mit Ballstörung): Haltewinkel bis ±25°, Stoss bis 25°,
# Steifigkeit kp0 nahe Mitte, Schaukeln nach 2.5 s Stillstand (Handsuche, siehe notizen.md).
START = dict(kv=0.30, t_lin=0.9, a_brems=110.0, v_max=220.0, u_max=24.0, kp0=0.6,
             fk_haft=3.0, fk_zeit=15.0, fk_haft_max=25.0, v_ruhig=12.0,
             fk_roll=0.0, v_roll=30.0, e_zone=5.0, e_zone_b=4.0, v_zone=15.0,
             ki=0.2, e_i=40.0, i_max=25.0, t_vor=0.04, du_max=150.0, ab_a=0.5, ab_b=0.15,
             t_pump=2.5, u_back=10.0, t_pmax=6.0, e_pend=12.0, v_fest=40.0, t_back=0.0)

# Parameter und Suchschritte (multiplikativ für Faktoren, additiv wo 0 erlaubt ist)
SCHRITTE = [
    ('kv', 'mul', 1.25), ('t_lin', 'mul', 1.25), ('a_brems', 'mul', 1.3), ('v_max', 'mul', 1.25),
    ('u_max', 'add', 2.0), ('fk_haft', 'add', 1.0), ('fk_zeit', 'mul', 1.5), ('fk_haft_max', 'add', 2.0),
    ('v_ruhig', 'mul', 1.3), ('fk_roll', 'add', 1.0), ('v_roll', 'mul', 1.4),
    ('e_zone', 'add', 1.5), ('e_zone_b', 'add', 1.5), ('v_zone', 'mul', 1.3),
    ('ki', 'add', 0.01), ('t_vor', 'add', 0.02), ('du_max', 'mul', 1.3),
    ('ab_a', 'add', 0.08), ('ab_b', 'mul', 1.3),
    ('kp0', 'add', 0.2), ('i_max', 'add', 3.0), ('e_i', 'mul', 1.3),
    ('t_pump', 'add', 0.8), ('u_back', 'add', 6.0), ('t_pmax', 'add', 2.0), ('e_pend', 'add', 4.0),
]
GRENZEN = {'ki': (0.0, 0.2), 'fk_roll': (0.0, 6.0), 'fk_haft': (0.0, 10.0), 't_vor': (0.0, 0.15),
           'ab_a': (0.15, 0.9), 'ab_b': (0.02, 0.6), 'u_max': (8.0, 28.0), 'e_zone': (0.0, 9.0),
           'e_zone_b': (1.0, 10.0), 'fk_haft_max': (0.0, 28.0), 'kp0': (0.0, 3.0), 'i_max': (0.0, 28.0),
           't_pump': (0.8, 10.0), 'u_back': (0.0, 28.0), 'e_pend': (6.0, 40.0)}


def kosten(st, runs):
    """Zuverlässigkeit vor Median: Mittel (Fail = 60 s) + p90 + Max/4."""
    Ts = [r['T_true'] if r['ok_true'] else 60.0 for r in runs]
    return sum(Ts) / len(Ts) + st['T_p90'] + 0.25 * st['T_max']


def bewerte(p, n=80, seed0=0):
    res = evaluate(lambda: ctrl_fuzzy.Regler(**p), n=n, seed0=seed0)
    return kosten(res['stats'], res['runs']), res


def suche(p, runden=3):
    best, res = bewerte(p)
    print('Start %.3f | %s' % (best, res['summary']), flush=True)
    for rd in range(runden):
        besser = False
        for name, art, s in SCHRITTE:
            cur = p[name]
            for richt in (1, -1):
                neu = cur * s ** richt if art == 'mul' else cur + richt * s
                lo, hi = GRENZEN.get(name, (1e-6, 1e9))
                neu = max(lo, min(hi, neu))
                if abs(neu - cur) < 1e-9:
                    continue
                q = dict(p)
                q[name] = round(neu, 4)
                k, r = bewerte(q)
                if k < best - 0.01:
                    best, p, besser = k, q, True
                    print('R%d %s=%.4g -> %.3f | %s' % (rd, name, neu, k, r['summary']), flush=True)
                    break
        if not besser:
            break
    return p, best


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'validieren':
        res = evaluate(lambda: ctrl_fuzzy.Regler(), n=200, seed0=100000)
        print(res['summary'])
        with open(os.path.join(HERE, 'result.json'), 'w') as f:
            json.dump(res['stats'], f, indent=1)
        sys.exit()
    t0 = time.time()
    runden = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    p, k = suche(dict(START), runden)
    print('Bestes (seed0=0, n=80): %.3f' % k)
    print('PARAM =', json.dumps(p))
    k2, r2 = bewerte(p, n=200, seed0=5000)
    print('Kontrolle seed0=5000 n=200: %.3f | %s' % (k2, r2['summary']))
    k3, r3 = bewerte({}, n=200, seed0=5000)
    print('Standardwerte auf Kontrolle:  %.3f | %s' % (k3, r3['summary']))
    print('Dauer %.0f s' % (time.time() - t0))
