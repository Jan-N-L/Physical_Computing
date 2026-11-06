# ctrl_qlearn.py: Q-Learning-Regler fuer Ball on Beam (Pico und Simulator).
# Die Tabelle (nur die beste Aktion pro Zustand) wurde am PC gelernt:
#   methods/qlearn/tune.py  ->  ctrl_qlearn_tab.py
# Auf dem Pico wird nur nachgeschlagen (keine Gleitkomma-Tabelle, nur bytes).
#
# Zustand (gespiegelt, nur e >= 0 gespeichert):
#   e = x - Mitte (mm), v = Tempo (mm/s, AlphaBeta), Steckzeit (Ball steht ausserhalb der Mitte)
# Aktion: Winkel aus U_SET, neu gewaehlt nur jeden SKIP-ten Schritt.
# Ausgang: u = U0 + b + Aktion, mit begrenzter Rate (RATE °/s, keine grossen Servospruenge).
#   U0 = feste Verschiebung der Waagrechten (Mitte des Offset-Bereichs).
#   b  = langsamer I-Anteil nahe der Mitte (lernt Restneigung und Ballstoerung an der Mitte).
#        Er ist beim Training Teil der Strecke, die Tabelle hat ihn also mitgelernt.

from bob_core import AlphaBeta, MITTE_MM
import ctrl_qlearn_tab as T


def _bin(x, edges):
    i = 0
    n = len(edges)
    while i < n and x >= edges[i]:
        i += 1
    return i


class Regler:
    def __init__(self, tab=None, e_edges=None, v_edges=None, st_edges=None, u_set=None,
                 skip=None, rate=None, ab_a=None, ab_b=None, v_st=None, tol_st=None,
                 u0=None, ki=None, i_zone=None, i_max=None):
        self.tab = tab if tab is not None else T.TAB
        self.ee = e_edges if e_edges is not None else T.E_EDGES
        self.ve = v_edges if v_edges is not None else T.V_EDGES
        self.se = st_edges if st_edges is not None else T.ST_EDGES
        self.us = u_set if u_set is not None else T.U_SET
        self.skip = skip if skip is not None else T.SKIP
        self.rate = rate if rate is not None else T.RATE
        self.v_st = v_st if v_st is not None else T.V_ST
        self.tol_st = tol_st if tol_st is not None else T.TOL_ST
        self.u0 = u0 if u0 is not None else T.U0
        self.ki = ki if ki is not None else T.KI
        self.i_zone = i_zone if i_zone is not None else T.I_ZONE
        self.i_max = i_max if i_max is not None else T.I_MAX
        self.nv = len(self.ve) + 1
        self.ns = len(self.se) + 1
        self.na = len(self.us)
        self.f = AlphaBeta(ab_a if ab_a is not None else T.AB_A, ab_b if ab_b is not None else T.AB_B)
        self.reset(MITTE_MM)

    def reset(self, x):
        self.f.reset(x)
        self.n = 0
        self.u = self.u0
        self.ua = 0.0
        self.b = 0.0
        self.tst = 0.0

    def zustand(self, e, v):
        """Index im gespiegelten Zustandsraum und Vorzeichen (1 oder -1)."""
        sg = 1
        if e < 0:
            e = -e
            v = -v
            sg = -1
        ie = _bin(e, self.ee)
        iv = _bin(v, self.ve)
        ist = _bin(self.tst, self.se)
        return (ie * self.nv + iv) * self.ns + ist, sg

    def waehle(self, idx):
        return self.tab[idx]

    def step(self, x, ok, dt):
        x, v = self.f.update(x, ok, dt)
        e = x - MITTE_MM
        ae = abs(e)
        if abs(v) < self.v_st and ae > self.tol_st:
            self.tst += dt
        else:
            self.tst = 0.0
        if ae < self.i_zone:
            b = self.b - self.ki * e * dt
            m = self.i_max
            self.b = m if b > m else -m if b < -m else b
        if self.n % self.skip == 0:
            idx, sg = self.zustand(e, v)
            self.ua = sg * self.us[self.waehle(idx)]
        self.n += 1
        d = self.u0 + self.b + self.ua - self.u
        lim = self.rate * dt
        if d > lim:
            d = lim
        elif d < -lim:
            d = -lim
        self.u += d
        return self.u
