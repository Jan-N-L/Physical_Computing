# main_q.py: Ball on Beam mit main_v3 + Q-Learning (Live-Programm und Training).
# Zusammengelegt aus qlern.py (Q-Learning) und main_q.py (Live-Programm), Funktion unverändert.
#
# Q-Learning:
#   Zustand s = Startseite ('sensor' wenn Start x < Mitte, sonst 'klotz').
#   Aktion a = einer von 6 Reglern: A0 bis A3 und A5 = Parametersätze für main_v3.Regler,
#              A4 = Kombi-MPC (ctrl_kombi.py).
#   Belohnung r = -T (Zeit bis zur Ruhe, s); Fail = -40.
#   Ein Lauf = eine Episode mit einer einzigen Entscheidung (gamma = 0, also ein
#   "kontextueller Bandit", die einfachste Form von Q-Learning).
#   Update: Q[s][a] += alpha * (r - Q[s][a]), alpha = max(1/N, ALPHA_MIN) -> folgt Drift.
#   Auswahl: zuerst jede Aktion N_MIN mal, danach epsilon-greedy.
#   Q-Tabelle in q_main.json auf dem Pico, wird bei jedem Lauf nachgeführt.
#
# Live (in VS Code öffnen, MicroPico "Run"): wählt aus der Q-Tabelle den besten Regler für die
#   Startseite. Nach "RUHE nach x s" oder 30 s Fail wird die Tabelle nachgeführt.
#   LERNEN = False: nur das Gelernte nutzen, Tabelle nicht verändern (sicher für die Vorführung).
# Training über die Brücke: import main_q; main_q.training(['sensor', 'klotz'] * 10, 0.1)
#
# Auf dem Pico nötig: vl53l0x.py, kennlinie.py, bob_core.py, main_v3.py, q_main.json,
#                    ctrl_mpc.py, ctrl_kombi.py (für A4), bob.py (nur für das Training)

from kennlinie import to_mm, MITTE_MM
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


LERNEN = True
EPS_LIVE = 0.0      # 0 = immer die beste Aktion (kein Erkunden in der Vorführung)


def run():
    from machine import Pin, I2C, PWM
    from vl53l0x import VL53L0X
    from utime import ticks_us, ticks_diff

    tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
    servo = PWM(Pin(16))
    servo.freq(50)
    clamp = main_v3.clamp

    def set_angle(u):
        a = clamp(u + main_v3.OFFSET, -main_v3.U_ABS, main_v3.U_ABS)
        servo.duty_u16(int((a + 90) / 180 * 8000 + 1000))

    q = laden()
    zeige(q)
    set_angle(0)
    reg = None
    x_last = None
    n_bad = 0
    n_still = 0
    n_da = 0
    x_ref = None
    t_prev = ticks_us()
    t_start = None
    t_in = None
    ruhe = False
    gelernt = False
    s_q = a_q = None
    n_print = 0
    print("Warte auf Ball ...")
    while True:
        s = tof.ping()
        t = ticks_us()
        dt = ticks_diff(t, t_prev) / 1e6
        t_prev = t
        if dt <= 0:
            dt = 0.03

        ok = s < main_v3.S_LEER
        x = None
        if ok:
            x = to_mm(s)
            if x_last is not None and n_bad < 3 and abs(x - x_last) > 30 + 500 * dt * (n_bad + 1):
                ok = False
        if ok:
            n_bad = 0
            if t_start is None:
                if x_ref is None or abs(x - x_ref) > 5:
                    x_ref = x
                    n_still = 0
                else:
                    n_still += 1
            x_last = x
        else:
            n_bad += 1

        if x_last is None:
            continue
        if n_bad > 15:                          # Ball weg
            set_angle(0)
            print("Ball weg")
            x_last = None
            t_start = None
            ruhe = False
            n_still = 0
            n_da = 0
            x_ref = None
            continue
        if t_start is None:
            n_da += 1
            if n_still < 10 and n_da < 50:      # starten, wenn ruhig, spätestens nach ca. 1.5 s
                continue
            s_q = zustand(x_last)
            a_q = waehle(q, s_q, EPS_LIVE)
            reg = make(a_q)
            reg.reset(x_last)
            t_start = t
            t_in = None
            gelernt = False
            print("Start bei %.0f mm, Zustand %s, Aktion %s" % (x_last, s_q, NAMEN[a_q]))

        u = reg.step(x_last, ok, dt)
        set_angle(u)

        tt = ticks_diff(t, t_start) / 1e6
        if abs(x_last - MITTE_MM) <= main_v3.TOL:
            if t_in is None:
                t_in = tt
            if not ruhe and tt - t_in >= main_v3.T_RUHE:
                ruhe = True
                print("RUHE nach %.1f s" % t_in)
                if LERNEN and not gelernt:
                    lernen(q, s_q, a_q, -t_in)
                    speichern(q)
                    gelernt = True
                    zeige(q)
        else:
            t_in = None
            if ruhe:
                ruhe = False
                print("Ziel verlassen")
        if LERNEN and not gelernt and tt > 30.0:
            print("Fail (über 30 s)")
            lernen(q, s_q, a_q, R_FAIL)
            speichern(q)
            gelernt = True

        n_print += 1
        if n_print >= 3:
            n_print = 0
            print("%.0f, %.1f" % (x_last, u))


if __name__ == "__main__":
    run()
