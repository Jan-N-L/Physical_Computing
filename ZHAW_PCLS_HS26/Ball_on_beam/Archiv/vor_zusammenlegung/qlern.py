# qlern.py: Q-Learning für main_v3 am echten Aufbau (Ball on Beam).
# Zustand s = Startseite ('sensor' wenn Start x < Mitte, sonst 'klotz').
# Aktion a = einer von 4 Parametersätzen für main_v3.Regler.
# Belohnung r = -T (Zeit bis zur Ruhe, s); Fail = -40.
# Ein Lauf = eine Episode mit einer einzigen Entscheidung (gamma = 0, also ein
# "kontextueller Bandit", die einfachste Form von Q-Learning).
# Update: Q[s][a] += alpha * (r - Q[s][a]), alpha = max(1/N, ALPHA_MIN) -> folgt Drift.
# Auswahl: zuerst jede Aktion N_MIN mal, danach epsilon-greedy.
# Q-Tabelle wird in q_main.json auf dem Pico gespeichert und lernt bei jedem Lauf weiter.
import json
import random
import main_v3
import bob_core as core

ACTIONS = [{}, {"kp_fine": 0.438, "kd": 0.125, "t_lead": 0.3, "v_hold": 6.0, "kick_t": 0.4, "kick_rate": 9.0, "alpha": 0.525}, {"kp_fine": 0.525, "kd": 0.15, "ki": 0.12, "fc": 2.4, "e_hold": 4.8, "kick_t": 0.75, "kick_rate": 12.0, "kick_max": 18.75, "rate": 150.0}, {"kp_coarse": 0.48, "ki": 0.09, "i_max": 15.0, "fc": 4.5, "v_hold": 6.0, "kick_rate": 22.5, "beta": 0.75}]
NAMEN = ['A0 Standard', 'A1 schnell/unruhig', 'A2 schnell', 'A3 kraeftig', 'A4 Kombi-MPC', 'A5 A1+Ziel-8mm']
ACTIONS.append('kombi')   # A4: MPC-Kombination (ctrl_kombi.py) statt main_v3
A5 = dict(ACTIONS[1])
A5['ziel'] = core.MITTE_MM - 8.0
ACTIONS.append(A5)        # A5: wie A1, internes Ziel 8 mm Richtung Sensor
DATEI = 'q_main.json'
EPS = 0.2
N_MIN = 2
ALPHA_MIN = 0.25
R_FAIL = -40.0


def laden():
    try:
        with open(DATEI) as f:
            q = json.load(f)
    except Exception:
        q = {}
    for s in ('sensor', 'klotz'):
        if s not in q:
            q[s] = {'Q': [], 'N': []}
        while len(q[s]['Q']) < len(ACTIONS):     # neue Aktionen ergänzen
            q[s]['Q'].append(0.0)
            q[s]['N'].append(0)
    return q


def make(a):
    """Regler für Aktion a."""
    p = ACTIONS[a]
    if p == 'kombi':
        import ctrl_kombi
        return ctrl_kombi.Regler()
    return main_v3.Regler(**p)


def speichern(q):
    with open(DATEI, 'w') as f:
        json.dump(q, f)


def zustand(x0):
    return 'sensor' if x0 < core.MITTE_MM else 'klotz'


def waehle(q, s, eps=EPS):
    Q = q[s]['Q']
    N = q[s]['N']
    for a in range(len(ACTIONS)):
        if N[a] < N_MIN:
            return a
    if random.random() < eps:
        return random.randrange(len(ACTIONS))
    best = 0
    for a in range(1, len(ACTIONS)):
        if Q[a] > Q[best]:
            best = a
    return best


def lernen(q, s, a, r):
    N = q[s]['N']
    N[a] += 1
    al = max(1.0 / N[a], ALPHA_MIN)
    q[s]['Q'][a] += al * (r - q[s]['Q'][a])


def zeige(q):
    for s in ('sensor', 'klotz'):
        print('Q', s, ' '.join('%s=%.1f(%d)' % (NAMEN[a].split()[0], q[s]['Q'][a], q[s]['N'][a]) for a in range(len(ACTIONS))))


def training(starts, eps=EPS):
    import bob
    h = bob.io()
    q = laden()
    for st in starts:
        x0 = core.vorbereiten(h, st, core.OFFSET)
        s = zustand(x0)
        a = waehle(q, s, eps)
        reg = make(a)
        res = core.versuch(h, reg, None, core.OFFSET)
        h.servo(core.OFFSET)
        r = res['T'] if res['ok'] else None
        lernen(q, s, a, -r if r is not None else R_FAIL)
        speichern(q)
        print('EP,%s,%.0f,%d,%s,%s' % (s, x0, a, 'OK' if res['ok'] else 'FAIL', '-1' if r is None else '%.2f' % r))
    zeige(q)
