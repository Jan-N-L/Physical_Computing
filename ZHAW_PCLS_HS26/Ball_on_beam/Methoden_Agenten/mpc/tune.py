"""tune.py: Abstimmung ctrl_mpc im gemeinsamen Simulator. Nur seed0 < 100000.
Aufruf:  python tune.py '[{"K": 8}, {"K": 10}]' [n] [seed0]
         python tune.py final      (Endvalidierung n=200, seed0=100000 -> result.json)
Bewertungszahl score: kleiner = besser, Zuverlässigkeit vor Median."""
import os
import sys
import json
HERE = os.path.dirname(os.path.abspath(__file__))
BOB = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(BOB, 'sim'))
sys.path.insert(0, os.path.join(BOB, 'pico'))
from sim import evaluate          # noqa: E402
import ctrl_mpc                   # noqa: E402


def score(st):
    return (30 - 30 * st['quote']) * 3 + st['T_p90'] * 2 + st['T_max'] * 0.5 + st['T_median']


def show(kw, n=40, seed0=0):
    st = evaluate(lambda: ctrl_mpc.Regler(**kw), n=n, seed0=seed0)['stats']
    ps = ' '.join('%s %d/%d' % (k[:6], v['erfolg'], v['n']) for k, v in sorted(st['pro_start'].items()))
    print('%-50s %6.2f | %s | %s' % (json.dumps(kw), score(st), st['summary'], ps), flush=True)
    return st


if __name__ == '__main__':
    if sys.argv[1] == 'final':
        st = evaluate(lambda: ctrl_mpc.Regler(), n=200, seed0=100000)['stats']
        print(st['summary'])
        with open(os.path.join(HERE, 'result.json'), 'w') as f:
            json.dump(st, f, indent=1, ensure_ascii=False)
    else:
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
        s0 = int(sys.argv[3]) if len(sys.argv) > 3 else 0
        assert s0 < 100000
        for kw in json.loads(sys.argv[1]):
            show(kw, n, s0)
