# ctrl_imitation.py: Imitations-Regelung (Behavioral Cloning) für Ball on Beam.
# Eine Tabelle u(e, v, s) wurde aus Vormach-Daten gelernt (ctrl_imitation_tab.py, 780 B int8).
# Stand 06.11. (neue Sim, unrunder Ball): Lehrer = privilegierter Experte in der Simulation
# (kennt die Ballstörung), gelernt mit Behavioral Cloning + 3 DAgger-Runden, siehe
# methods/imitation/tune.py und notizen.md.
#   e = MITTE_MM - x (mm), v = Balltempo (mm/s, AlphaBeta),
#   s = Standzeit (s): wie lange der Ball schon neben der Mitte klebt.
# s ist nötig, weil der Lehrer bei klebendem Ball "langsam mehr kippt, bis er rollt".
# Ohne s mittelt die Tabelle das weg und der Ball bleibt 10 bis 25 mm neben der Mitte liegen.
# Zwischen den Stützstellen wird trilinear gemischt (gelernte Fuzzy-Tabelle).
# u in ° Servo relativ zu OFFSET.
#
# Zusätzlich: Klasse Vormachen (Poti an GP26) zum Aufnehmen einer eigenen Tabelle auf dem Pico.
# Läuft auf MicroPython: nur bob_core, math und Grundtypen.

from math import exp
from bob_core import AlphaBeta, MITTE_MM, clamp

# Stützstellen (mm, mm/s, s). Fein nahe Ziel, grob aussen.
E_N = (-170, -110, -70, -45, -28, -16, -8, 0, 8, 16, 28, 45, 70, 110, 170)
V_N = (-240, -150, -90, -50, -25, -10, 0, 10, 25, 50, 90, 150, 240)
S_N = (0.0, 0.4, 0.8, 1.5)
NE, NV, NS = len(E_N), len(V_N), len(S_N)
Q = 0.25            # ° pro Tabelleneinheit (int8 -> ±31.75°)
V_STAND = 20.0      # mm/s: langsamer gilt als "steht"
E_STAND = 7.0       # mm: weiter weg von der Mitte gilt als "neben der Mitte"
T_LOS = 0.15        # s: Standzeit klingt ab, sobald der Ball rollt


def _stelle(x, n):
    """Index i und Anteil f mit x zwischen n[i] und n[i+1] (am Rand begrenzt)."""
    if x <= n[0]:
        return 0, 0.0
    k = len(n) - 1
    if x >= n[k]:
        return k - 1, 1.0
    i = 0
    while x > n[i + 1]:
        i += 1
    return i, (x - n[i]) / (n[i + 1] - n[i])


def _s8(b):
    return b - 256 if b > 127 else b


class Merkmale:
    """Was der Pico sieht: (e, v, s). Gleich beim Vormachen und beim Nachmachen."""

    def __init__(self, a=0.5, b=0.15, mit_s=True):
        self.f = AlphaBeta(a, b)
        self.s = 0.0
        self.mit_s = mit_s

    def reset(self, x):
        self.f.reset(x)
        self.s = 0.0

    def update(self, x, ok, dt):
        x, v = self.f.update(x, ok, dt)
        e = MITTE_MM - x
        if not self.mit_s:
            pass
        elif abs(v) < V_STAND and abs(e) > E_STAND:
            self.s = min(self.s + dt, S_N[-1])
        else:
            self.s *= exp(-dt / T_LOS)
        return e, v, self.s


class Regler:
    """ruetteln: Zusatz (nicht gelernt). Klebt der Ball trotz voller Tabelle (s am Anschlag),
    wird er kurz zurückgekippt (t_zur s mit u_zur) und dann voll geschoben (t_hin s).
    So bekommt er Schwung, um aus einer Mulde des unrunden Balls zu rollen."""

    def __init__(self, tab=None, skala=1.0, a=0.5, b=0.15, mit_s=True,
                 ruetteln=False, u_zur=15.0, t_zur=0.35, t_hin=0.6):
        if tab is None:
            from ctrl_imitation_tab import TAB
            tab = TAB
        self.t = bytes(tab)          # bleibt int8 als bytes (780 B), spart RAM auf dem Pico
        self.q = Q * skala
        self.m = Merkmale(a, b, mit_s)
        self.ruetteln = ruetteln
        self.u_zur, self.t_zur, self.t_hin = u_zur, t_zur, t_hin

    def reset(self, x):
        self.m.reset(x)
        self.tr = -1.0
        self.sg = 1.0

    def nachschlagen(self, e, v, s):
        i, fe = _stelle(e, E_N)
        j, fv = _stelle(v, V_N)
        k, fs = _stelle(s, S_N)
        t = self.t
        u = 0.0
        for di, we in ((0, 1.0 - fe), (1, fe)):
            for dj, wv in ((0, 1.0 - fv), (1, fv)):
                b = ((i + di) * NV + j + dj) * NS + k
                t0 = _s8(t[b])
                u += we * wv * (t0 + (_s8(t[b + 1]) - t0) * fs)
        return u * self.q

    def step(self, x, ok, dt):
        e, v, s = self.m.update(x, ok, dt)
        if self.ruetteln:
            if self.tr < 0.0 and s >= S_N[-1] - 1e-6:
                self.tr = 0.0
                self.sg = 1.0 if e > 0 else -1.0
            if self.tr >= 0.0:
                self.tr += dt
                if self.tr < self.t_zur:
                    return -self.sg * self.u_zur
                if self.tr < self.t_zur + self.t_hin:
                    return 28.0 if self.sg > 0 else -31.0
                self.tr = -1.0
                self.m.s = S_N[-2]       # Tabelle übernimmt wieder, noch "klebend"
        return self.nachschlagen(e, v, s)


class Lerner:
    """Sammelt (e, v, s, u) trilinear gewichtet auf die Stützstellen. Gleicher Code im Simulator.
    versatz: Schritte (à ca. 29 ms), um die die Hand der Situation nachläuft (0 = aus).
    spiegeln: jede Probe auch gespiegelt (-e, -v, s, -u) eintragen."""

    def __init__(self, versatz=0, spiegeln=True):
        n = NE * NV * NS
        self.sw = [0.0] * n
        self.su = [0.0] * n
        self.versatz = versatz
        self.spiegeln = spiegeln
        self.puf = []

    def neu(self):
        self.puf = []

    def _add(self, e, v, s, u):
        i, fe = _stelle(e, E_N)
        j, fv = _stelle(v, V_N)
        k, fs = _stelle(s, S_N)
        for di, we in ((0, 1 - fe), (1, fe)):
            for dj, wv in ((0, 1 - fv), (1, fv)):
                for dk, ws in ((0, 1 - fs), (1, fs)):
                    w = we * wv * ws
                    if w > 0:
                        b = ((i + di) * NV + j + dj) * NS + k + dk
                        self.sw[b] += w
                        self.su[b] += w * u

    def merke(self, e, v, s, u):
        self.puf.append((e, v, s))
        if len(self.puf) > self.versatz:
            e0, v0, s0 = self.puf.pop(0)
            self._add(e0, v0, s0, u)
            if self.spiegeln:
                self._add(-e0, -v0, s0, -u)

    def tabelle(self, w_min=0.5):
        """Mittelwert pro Stützstelle; Stellen mit wenig Daten vom nächsten gefüllten Nachbarn."""
        n = NE * NV * NS
        t = [None] * n
        voll = []
        for b in range(n):
            if self.sw[b] >= w_min:
                t[b] = self.su[b] / self.sw[b]
                voll.append(b)
        for b in range(n):
            if t[b] is None:
                ij, k = divmod(b, NS)
                i, j = divmod(ij, NV)
                best = None
                for bb in voll:
                    ij2, k2 = divmod(bb, NS)
                    i2, j2 = divmod(ij2, NV)
                    d = abs(i2 - i) + abs(j2 - j) + abs(k2 - k)
                    if best is None or d < best[0]:
                        best = (d, t[bb])
                t[b] = best[1] if best else 0.0
        return bytes([int(round(clamp(u / Q, -127, 127))) & 255 for u in t])

    def speichern(self, name='ctrl_imitation_tab.py', w_min=0.5):
        tb = self.tabelle(w_min)
        with open(name, 'w') as f:
            f.write('# Gelernte Tabelle (int8, Q=%.2f Grad), %d x %d x %d\n' % (Q, NE, NV, NS))
            f.write('TAB = %r\n' % tb)


class Vormachen:
    """Aufnahme am echten Aufbau: Regler-Interface, u kommt vom Poti (Schleifer an GP26).
    Pico:  import bob, bob_core as core, ctrl_imitation as ci
           vm = ci.Vormachen()
           for k in range(20): core.versuch(bob.io(), vm, None)   # Ball jeweils von Hand hinlegen
           vm.speichern()                                          # schreibt ctrl_imitation_tab.py"""

    def __init__(self, u_max=15.0, versatz=0, pin=26):
        from machine import ADC
        self.adc = ADC(pin)
        self.u_max = u_max
        self.m = Merkmale()
        self.lern = Lerner(versatz)

    def reset(self, x):
        self.m.reset(x)
        self.lern.neu()

    def step(self, x, ok, dt):
        e, v, s = self.m.update(x, ok, dt)
        u = (self.adc.read_u16() / 65535 - 0.5) * 2 * self.u_max
        self.lern.merke(e, v, s, u)
        return u

    def speichern(self, name='ctrl_imitation_tab.py'):
        self.lern.speichern(name)
