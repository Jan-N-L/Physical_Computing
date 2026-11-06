# auswertung.py: A/B-Log auswerten (Zeit bis Ruhe + wie ruhig es danach ist)
import sys
fn = sys.argv[1]
runs = []
cur = None
for l in open(fn, encoding='utf-8', errors='replace'):
    if l.startswith('# modul'):
        name = l.split('kwargs')[0].replace('# modul', '').strip()
        kw = l.split('kwargs')[1].split('offset')[0].strip()
        cur = {'name': name + ' ' + (kw if kw != 'None' else ''), 'D': []}
    elif l.startswith('D,') and cur is not None:
        p = l.strip().split(',')
        cur['D'].append((float(p[1]), float(p[3]), float(p[5])))
    elif l.startswith('R,') and cur is not None:
        p = l.strip().split(',')
        cur['start'] = p[2]
        cur['ok'] = p[3] == 'OK'
        cur['T'] = int(p[4]) / 1000.0
        runs.append(cur)
        cur = None
agg = {}
for r in runs:
    D = r['D']
    if r['ok']:
        w = [q for q in D if r['T'] + 0.5 <= q[0] <= r['T'] + 3.5]
    else:
        w = [q for q in D if q[0] >= D[-1][0] - 3]
    jit = sum(abs(w[k + 1][2] - w[k][2]) for k in range(len(w) - 1)) / 3.0 if len(w) > 2 else -1
    xs = [q[1] for q in w]
    rg = max(xs) - min(xs) if xs else -1
    r['jit'] = jit
    r['rg'] = rg
    print('%-70s %-6s %-4s T=%5.1f  Servo in Ruhe %5.1f °/s  Ball-Bereich %4.1f mm' % (
        r['name'][:70], r['start'], 'OK' if r['ok'] else 'FAIL', r['T'] if r['ok'] else 30, jit, rg))
    agg.setdefault(r['name'], []).append(r)
print()
for k, v in agg.items():
    Ts = sorted(r['T'] if r['ok'] else 30.0 for r in v)
    print('%-70s ok %d/%d  T median %.1f  max %.1f  Servo %.1f °/s  Ball %.1f mm' % (
        k[:70], sum(r['ok'] for r in v), len(v), Ts[len(Ts) // 2], Ts[-1],
        sorted(r['jit'] for r in v)[len(v) // 2], sorted(r['rg'] for r in v)[len(v) // 2]))
