# tune.py: reproduzierbares Abstimmen der Golfer-Regelung (nur seed0 < 100000).
# Ablauf wie im Lauf vom 06.11.: Rastersuchen nacheinander, je n=40 bzw. n=80.
# Aufruf: python3 tune.py            (ca. 2 bis 4 min)
#         python3 tune.py validieren (einmalig am Schluss, schreibt result.json)
import sys, json, itertools
sys.path.insert(0, '/home/claude/bob/sim')
sys.path.insert(0, '/home/claude/bob/pico')
from sim import evaluate
from ctrl_golfer import Regler

def ev(n, seed0, **p):
    return evaluate(lambda: Regler(**p), n=n, seed0=seed0)['stats']

def raster(grid, n, seed0, base):
    keys = list(grid)
    for vals in itertools.product(*[grid[k] for k in keys]):
        p = dict(base); p.update(zip(keys, vals))
        s = ev(n, seed0, **p)
        print(dict(zip(keys, vals)), s['erfolg'], '/', n, 'med %.2f p90 %.2f' % (s['T_median'], s['T_p90']), flush=True)

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'validieren':
        s = ev(200, 100000)
        json.dump(s, open('/home/claude/bob/methods/golfer/result.json', 'w'), indent=1)
        print(s['summary']); print(s['pro_start'])
        sys.exit()
    b = dict(plan=0.6, e_hold=22.0, kp=0.8, kd=0.2, ki=0.3, kick=4.0, rate=300.0)
    raster({'kp': [0.5, 0.8, 1.0], 'kd': [0.1, 0.2], 'rate': [150, 300]}, 40, 0, b)   # Haltephase
    raster({'e_db': [6, 8], 'ki': [0.3, 0.8], 'kick': [0, 4], 'kp': [0.8, 1.2]}, 40, 0, b)
    b.update(kp=1.2, ki=0.8, kick=0.0)
    raster({'plan': [0.45, 0.6], 'kd': [0.2, 0.3], 'e_hold': [22, 30]}, 80, 0, b)      # Schlag
    raster({'plan': [0.45, 0.6], 'e_hold': [22, 30]}, 80, 1000, b)                      # Gegenprobe
    print('Endwerte = Standardwerte in ctrl_golfer.py:', ev(80, 0)['summary'])
