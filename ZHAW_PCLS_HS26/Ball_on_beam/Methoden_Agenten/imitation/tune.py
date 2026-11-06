"""tune.py: Imitations-Regelung, reproduzierbares Abstimmen im Simulator (neue Sim mit unrundem Ball).

Kein Mensch verfügbar. Lehrer sind darum Regler im Simulator:
  * Experte ("Röntgen-Lehrer"): sieht die WAHRE Position/Tempo, kennt K, Offset, Reibung und die
    Ballstörung des aktuellen Laufs und kompensiert sie exakt (Vorsteuerung). Am echten Aufbau
    gibt es ihn nicht. Er dient nur als Lehrer in der Simulation ("learning by cheating").
  * Mensch: synthetischer Mensch am Poti aus dem alten Lauf (Reaktionszeit, Zittern).
  * PD: einfacher PD+I aus sim/ctrl_demo_pd.py (sieht nur, was der Pico sieht).
Der Schüler (pico/ctrl_imitation.py) sieht nur (e, v, s) aus Messwerten und lernt
u = Mittelwert des Lehrers pro Stützstelle (Behavioral Cloning), wahlweise mit DAgger:
der Schüler fährt selbst, der Lehrer sagt in jedem Schritt, was er getan hätte.

Alle Abstimm-Seeds < 100000. Trainings-Seeds 50000.., Abstimm-Bewertung seed0 = 0.
Aufruf:  python tune.py            (Abstimmen, Vergleiche)
         python tune.py --final    (Endtabelle schreiben + Validierung n=200 seed0=100000)
"""
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BOB = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(BOB, 'sim'))
sys.path.insert(0, os.path.join(BOB, 'pico'))

import sim                     # noqa: E402
import bob_core as core        # noqa: E402
import ctrl_imitation as ci    # noqa: E402
import ctrl_demo_pd            # noqa: E402

# --- Plant-Register: der Lehrer braucht die wahre Strecke (seine Augen) ---------------------
_AKTUELL = [None]
_Plant0 = sim.Plant


class PlantMitAugen(_Plant0):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        _AKTUELL[0] = self


sim.Plant = PlantMitAugen      # nur in diesem Prozess, sim.py bleibt unverändert


def stoerung(p, x, roll):
    """Wahre Zusatzbeschleunigung (mm/s²) bei Servo = Offset: Stäbe + unrunder Ball."""
    ang = 0.0
    for a, L, ph in p.prof:
        ang += a * math.sin(2 * math.pi * x / L + ph)
    a1, p1, a2, p2 = p.wob
    ph = 2 * math.pi * roll / p.circ
    return p.k_rail * ang + a1 * math.sin(ph + p1) + a2 * math.sin(2 * ph + p2)


class Experte:
    """Privilegierter Lehrer. Geschwindigkeitsprofil zur Mitte, exakte Vorsteuerung der Störung.
    u relativ zu core.OFFSET (wie jeder Regler)."""

    def __init__(self, a_br=150.0, v_max=350.0, kpv=2.0, kv=6.0, vor=0.05):
        self.a_br, self.v_max, self.kpv, self.kv, self.vor = a_br, v_max, kpv, kv, vor

    def reset(self, x):
        self.p = _AKTUELL[0]

    def u(self):
        p = self.p
        # etwas vorausschauen (Servo-Totzeit)
        x = p.x + p.v * self.vor
        v = p.v
        e = core.MITTE_MM - x
        ae = abs(e)
        v_ref = math.copysign(min(self.v_max, math.sqrt(2 * self.a_br * ae), self.kpv * ae), e)
        a_des = self.kv * (v_ref - v)
        d = stoerung(p, x, p.roll + p.v * self.vor)
        k = p.k_rail * (sim.rail(p.off + 1) - sim.rail(p.off - 1)) / 2
        if abs(v) > 2.0:
            fr = math.copysign(p.mu_r, v)
        elif ae < 3.0:
            fr = 0.0
            a_des = 0.0          # liegt in der Mitte: nur Störung halten
        else:
            fr = math.copysign(p.mu_s * 1.1, a_des)
        uu = (a_des - d + fr) / k + (p.off - core.OFFSET)
        return core.clamp(uu, -31.9, 28.1)

    def step(self, x, ok, dt):
        return self.u()


class Mensch:
    """Synthetischer Mensch am Poti (aus dem alten Lauf, unverändert in der Strategie)."""

    def __init__(self, rng, kp=0.12, kd=0.20, u_max=15.0, reakt=0.20, hand=0.12,
                 zitter=0.8, seh=2.5, schub=8.0, streu=0.2):
        g = lambda: math.exp(rng.gauss(0, streu))   # noqa: E731
        self.rng = rng
        self.kp, self.kd = kp * g(), kd * g()
        self.u_max = u_max
        self.reakt = reakt * g()
        self.hand = hand * g()
        self.zitter = zitter
        self.seh = seh
        self.schub_rate = schub * g()

    def reset(self, x):
        self.p = _AKTUELL[0]
        self.hist = []
        self.uh = 0.0
        self.z = 0.0
        self.schub = 0.0
        self.ve = 0.0

    def _sehen(self, t):
        tl = t - self.reakt
        for (tt, xx) in reversed(self.hist):
            if tt <= tl:
                return xx
        return self.hist[0][1]

    def step(self, x, ok, dt):
        p = self.p
        t = p.t
        self.hist.append((t, p.x))
        while len(self.hist) > 2 and self.hist[0][0] < t - 1.0:
            self.hist.pop(0)
        xa = self._sehen(t) + self.rng.gauss(0, self.seh)
        xb = self._sehen(t - 0.1)
        self.ve += ((xa - xb) / 0.1 - self.ve) * min(1.0, dt / 0.08)
        e = core.MITTE_MM - xa
        ziel = self.kp * e - self.kd * self.ve
        if abs(self.ve) < 12 and abs(e) > 7:
            self.schub = core.clamp(self.schub + math.copysign(self.schub_rate * dt, e), -8, 8)
        else:
            self.schub *= math.exp(-dt / 0.15)
        ziel = core.clamp(ziel + self.schub, -self.u_max, self.u_max)
        self.uh += (ziel - self.uh) * min(1.0, dt / self.hand)
        a = math.exp(-dt / 0.15)
        self.z = a * self.z + math.sqrt(1 - a * a) * self.rng.gauss(0, self.zitter)
        return self.uh + self.z


def lehrer_fabrik(name, seed=0):
    box = {'i': 0}

    def make():
        box['i'] += 1
        if name == 'experte':
            return Experte()
        if name == 'mensch':
            return Mensch(random.Random(777000 + seed + box['i']))
        return ctrl_demo_pd.Regler()
    return make


# --- Daten sammeln: wer fährt (Lehrer oder Schüler), Lehrer liefert immer das Label --------
class Sammler:
    def __init__(self, lehrer, lerner, schueler=None, beta=1.0, rng=None):
        self.L = lehrer
        self.lern = lerner
        self.S = schueler          # ci.Regler oder None
        self.beta = beta
        self.rng = rng
        self.m = ci.Merkmale(mit_s=True)

    def reset(self, x):
        self.L.reset(x)
        self.m.reset(x)
        self.lern.neu()
        self.lehrer_faehrt = self.S is None or self.rng.random() < self.beta

    def step(self, x, ok, dt):
        e, v, s = self.m.update(x, ok, dt)
        u_l = self.L.step(x, ok, dt)
        self.lern.merke(e, v, s, u_l)
        if self.lehrer_faehrt:
            return u_l
        return self.S.nachschlagen(e, v, s)


def sammeln(lerner, lehrer_make, n_ep, seed, schueler=None, beta=1.0):
    rng = random.Random(seed)
    for i in range(n_ep):
        sim.run_one(lambda: Sammler(lehrer_make(), lerner, schueler, beta, rng), seed + i)


def lerne(lehrer='experte', n_ep=120, dagger=(0.5, 0.0, 0.0), seed=50000, spiegeln=False, w_min=0.5):
    """BC mit n_ep Episoden, danach je DAgger-Runde n_ep Episoden (beta = Anteil Lehrer fährt).
    Die Daten aller Runden werden zusammen gemittelt (Standard-DAgger)."""
    mk = lehrer_fabrik(lehrer, seed)
    L = ci.Lerner(0, spiegeln)
    sammeln(L, mk, n_ep, seed)
    tb = L.tabelle(w_min)
    for r, beta in enumerate(dagger):
        S = ci.Regler(tab=tb)
        sammeln(L, mk, n_ep, seed + 1000 * (r + 1), S, beta)
        tb = L.tabelle(w_min)
    return tb, L


def kurz(res):
    s = res['stats']
    return '%3d/%d  med %5.2f  p90 %5.2f  max %5.2f  <=5s %3d  Servo %3.0f' % (
        s['erfolg'], s['n'], s['T_median'], s['T_p90'], s['T_max'], s['unter_5s'], s['servo_weg_pro_s'])


def tab_bewerten(tb, n=80, seed0=0):
    return sim.evaluate(lambda: ci.Regler(tab=tb), n=n, seed0=seed0)


def tab_text(tb, k=0):
    rows = ['Schnitt s = %.1f s' % ci.S_N[k], 'e\\v  ' + ' '.join('%5d' % v for v in ci.V_N)]
    for i, e in enumerate(ci.E_N):
        rows.append('%4d ' % e + ' '.join('%5.1f' % (ci._s8(tb[(i * ci.NV + j) * ci.NS + k]) * ci.Q)
                                          for j in range(ci.NV)))
    return '\n'.join(rows)


# Endwahl (aus dem Abstimmen unten, nur seed0 < 100000)
END = dict(lehrer='experte', n_ep=120, dagger=(0.5, 0.0, 0.0), seed=50000, spiegeln=False)

if __name__ == '__main__':
    N = 80
    t0 = time.time()
    if '--final' not in sys.argv:
        for nm in ('experte', 'mensch', 'pd'):
            print('Lehrer %-8s n=%d: %s' % (nm, N, kurz(sim.evaluate(lehrer_fabrik(nm), n=N, seed0=0))), flush=True)
        print('\nSchüler, Lehrer Experte:')
        for kw in (dict(dagger=()), dict(dagger=(0.5,)), dict(dagger=(0.5, 0.0, 0.0)),
                   dict(dagger=(0.5, 0.0, 0.0), spiegeln=True)):
            tb, L = lerne(**kw)
            print(' %-40s %s' % (kw, kurz(tab_bewerten(tb, N))), flush=True)
        print('\nSchüler, andere Lehrer (DAgger 0.5,0,0):')
        for nm in ('mensch', 'pd'):
            tb, L = lerne(lehrer=nm)
            print(' %-8s %s' % (nm, kurz(tab_bewerten(tb, N))), flush=True)
        print('\nZeit %.0f s' % (time.time() - t0))
    else:
        tb, L = lerne(**END)
        leer = sum(1 for w in L.sw if w < 0.5)
        print('leere Stützstellen:', leer, 'von', len(L.sw))
        print(tab_text(tb, 0))
        print(tab_text(tb, ci.NS - 1))
        with open(os.path.join(BOB, 'pico', 'ctrl_imitation_tab.py'), 'w') as f:
            f.write('# ctrl_imitation_tab.py: gelernte Tabelle (int8, Q=%.2f Grad), %d x %d x %d Stützstellen.\n'
                    '# Erzeugt von methods/imitation/tune.py --final, Lehrer: %s\n'
                    % (ci.Q, ci.NE, ci.NV, ci.NS, END))
            f.write('TAB = %r\n' % tb)
        import importlib
        import ctrl_imitation_tab
        importlib.reload(ctrl_imitation_tab)
        res = sim.evaluate(lambda: ci.Regler(), n=200, seed0=100000)
        print('Tabelle :', res['summary'])
        out = {'tabelle': res['stats']}
        for nm in ('experte', 'pd'):
            r = sim.evaluate(lehrer_fabrik(nm, 100000), n=200, seed0=100000)
            print('Lehrer %-8s: %s' % (nm, r['summary']))
            out[nm] = r['stats']
        with open(os.path.join(HERE, 'result.json'), 'w') as f:
            json.dump(res['stats'], f, indent=1, ensure_ascii=False)
        with open(os.path.join(HERE, 'vergleich_validierung.json'), 'w') as f:
            json.dump(out, f, indent=1, ensure_ascii=False)
        print('Zeit %.0f s' % (time.time() - t0))
