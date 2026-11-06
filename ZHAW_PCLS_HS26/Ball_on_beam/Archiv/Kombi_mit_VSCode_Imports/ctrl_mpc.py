# ctrl_mpc.py: Vorhersage-Regelung (MPC) mit Online-Schätzung der Ballstörung.
# MicroPython-tauglich (nur math, bob_core).
# Modell: a = K*u_ist + d(x) - Reibung,  d(x) = c0 + A*sin(w*x) + B*cos(w*x),  w = 2*pi/lam
#   c0 = Offsetfehler (+ Rest), A/B = unrunder Ball (Störung hängt vom Rollweg = x ab).
# Ablauf pro Schritt:
#  1. Servo-Ist schätzen (Totzeit + Trägheit).
#  2. Beobachter: Vorhersage x, v mit Modell, Korrektur mit Messung (alpha-beta),
#     Innovation r treibt die Schätzung von c0, A, B (LMS, Regressor 1, sin, cos).
#  3. Zustand um tpre vorausrechnen (Mess- und Servoverzögerung).
#  4. Wenige Kandidaten-Winkel u = u_halt + du (u_halt = Waagrecht-Schätzung am Ziel)
#     über nh Teilschritte h mit dem Störmodell durchrechnen. Kosten: Abstand zum Ziel
#     pro Teilschritt + Haltepunkt bei Vollbremsung am Ende. Kleinste Kosten gewinnen.
#  5. Halten: Ball ruht nahe Ziel -> lokales Waagrecht ausgeben, Haftreibung hält ihn.
# u ist relativ zu OFFSET (bob_core). Positives u: Ball rollt zu grösseren x.
import math
from Ball_on_beam.ALT.Kombi.bob_core import MITTE_MM, OFFSET, U_ABS, X_END

U_LO = -U_ABS - OFFSET
U_HI = U_ABS - OFFSET


class Regler:
    def __init__(self, K=10.0, mu=15.0, mus=22.0, lam=105.0,
                 du=(-30.0, -14.0, -6.0, 0.0, 6.0, 14.0, 30.0),
                 h=0.15, nh=3, wt=2.0, abrk=120.0,
                 tpre=0.05, tau=0.03, dead=0.012,
                 l1=0.45, l2=0.15, g0=0.003, g1=0.003, vb=25.0, gate=15.0,
                 cmax=250.0, amax=220.0, ve=8.0,
                 ehold=0.0, erel=9.0, vhold=12.0, vrel=0.0, harm=1, rate_u=140.0, gs=0.3, nst=3, fs=0.7):
        self.K = K
        self.mu = mu
        self.mus = mus
        self.w = 2.0 * math.pi / lam
        self.du = du
        self.h = h
        self.nh = nh
        self.wt = wt
        self.abrk = abrk
        self.tpre = tpre
        self.tau = tau
        self.dead = dead
        self.l1 = l1
        self.l2 = l2
        self.g0 = g0
        self.g1 = g1 if harm else 0.0
        self.hs = 0.5 if harm else 0.0   # Anteil Haft-Lernen auf sin/cos
        self.vb = vb
        self.gate = gate
        self.cmax = cmax
        self.amax = amax
        self.ve = ve
        self.ehold = ehold
        self.erel = erel
        self.vhold = vhold
        self.vrel = vrel if vrel > 0 else 2 * vhold   # Loslassen ab diesem Tempo
        self.rate_u = rate_u
        self.gs = gs
        self.nst = nst
        self.fs = fs
        self.reset(MITTE_MM)

    def reset(self, x):
        self.x = x
        self.v = 0.0
        self.c0 = 0.0
        self.A = 0.0
        self.B = 0.0
        self.ua = 0.0
        self.uc = 0.0
        self.ngate = 0
        self.hold = False
        self.nstk = 0

    def _d(self, x):
        wx = self.w * x
        return self.c0 + self.A * math.sin(wx) + self.B * math.cos(wx)

    # ein Schritt Dauer t mit Antrieb a (ohne Reibung); Reibung, Haften, Stopp
    def _mv(self, x, v, a, t):
        mu = self.mu
        if v == 0.0:
            if a > self.mus:
                an = a - mu
            elif a < -self.mus:
                an = a + mu
            else:
                return x, 0.0
            return x + 0.5 * an * t * t, an * t
        an = a - mu if v > 0 else a + mu
        vn = v + an * t
        if vn * v >= 0:
            return x + 0.5 * (v + vn) * t, vn
        t1 = -v / an
        return x + 0.5 * v * t1, 0.0

    def step(self, z, ok, dt):
        K = self.K
        # --- Servo-Ist
        te = dt - self.dead
        if te < 0:
            te = 0.0
        ua = self.ua
        dlt = (self.uc - ua) * te / (te + self.tau)
        um = ua + 0.5 * dlt
        self.ua = ua + dlt
        # --- Beobachter: Vorhersage
        v = self.v
        if -self.ve < v < self.ve:
            v = 0.0
        x = self.x
        wx = self.w * x
        s = math.sin(wx)
        c = math.cos(wx)
        d = self.c0 + self.A * s + self.B * c
        xp, vp = self._mv(x, v, K * um + d, dt)
        if xp < 0.0:
            xp = 0.0
            vp = 0.0
        elif xp > X_END:
            xp = X_END
            vp = 0.0
        # --- Korrektur
        if ok:
            r = z - xp
            if r > self.gate or r < -self.gate:
                self.ngate += 1
                if self.ngate <= 3:
                    r = 0.0
                else:
                    self.ngate = 0
            else:
                self.ngate = 0
            xp += self.l1 * r
            vp += self.l2 * r / dt
            if (vp > self.vb or vp < -self.vb) and 5.0 < xp < X_END - 5.0:
                ea = r / (dt * dt)
                cm = self.cmax
                c0 = self.c0 + self.g0 * ea
                self.c0 = cm if c0 > cm else -cm if c0 < -cm else c0
                g = self.g1 * ea
                A = self.A + g * s
                B = self.B + g * c
                n2 = A * A + B * B
                am = self.amax
                if n2 > am * am:
                    f = am / math.sqrt(n2)
                    A *= f
                    B *= f
                self.A = A
                self.B = B
        # --- Klebt der Ball, obwohl das Modell Losrollen erwartet? Dann ist die
        # geschätzte Störung an dieser Stelle zu gross: Richtung Haftgrenze ziehen.
        am = K * self.ua + d
        if -self.ve < vp < self.ve and (am > self.mus or am < -self.mus) and 5.0 < xp < X_END - 5.0:
            self.nstk += 1
            if self.nstk >= self.nst:
                tgt = self.fs * self.mus if am > 0 else -self.fs * self.mus
                dd = self.gs * (tgt - am)
                hs = self.hs
                self.c0 += (1.0 - hs) * dd
                self.A += hs * dd * s
                self.B += hs * dd * c
        else:
            self.nstk = 0
        self.x = xp
        self.v = vp
        # --- Halten nahe Ziel
        e0 = xp - MITTE_MM
        ae = e0 if e0 > 0 else -e0
        if self.hold:
            if ae > self.erel or vp > self.vrel or vp < -self.vrel:
                self.hold = False
        elif ae < self.ehold and -self.vhold < vp < self.vhold:
            self.hold = True
        if self.hold:
            u = -self._d(xp) / K
            u = U_HI if u > U_HI else U_LO if u < U_LO else u
            lim = self.rate_u * dt
            uc = self.uc
            if u > uc + lim:
                u = uc + lim
            elif u < uc - lim:
                u = uc - lim
            self.uc = u
            return u
        # --- Vorausrechnen bis der neue Befehl wirkt
        v = vp
        if -self.ve < v < self.ve:
            v = 0.0
        x0, v0 = self._mv(xp, v, K * self.ua + self._d(xp), self.tpre)
        if x0 < 0.0:
            x0 = 0.0
            v0 = 0.0
        elif x0 > X_END:
            x0 = X_END
            v0 = 0.0
        # Störung als Amplitude/Phase: eine sin-Auswertung pro Teilschritt
        w = self.w
        M = math.sqrt(self.A * self.A + self.B * self.B)
        ph = math.atan2(self.B, self.A)
        c0 = self.c0
        uh = -(c0 + M * math.sin(w * MITTE_MM + ph)) / K
        h = self.h
        nh = self.nh
        abrk = self.abrk
        wt = self.wt
        best = 1e30
        ub = uh
        for du in self.du:
            u = uh + du
            if u > U_HI:
                u = U_HI
            elif u < U_LO:
                u = U_LO
            ku = K * u + c0
            xe = x0
            ve = v0
            cst = 0.0
            for i in range(nh):
                xe, ve = self._mv(xe, ve, ku + M * math.sin(w * xe + ph), h)
                if xe < 0.0:
                    xe = 0.0
                    ve = 0.0
                elif xe > X_END:
                    xe = X_END
                    ve = 0.0
                e = xe - MITTE_MM
                cst += e * e
            e += ve * (ve if ve > 0 else -ve) / (2.0 * abrk)
            cst += wt * e * e
            if cst < best:
                best = cst
                ub = u
        lim = self.rate_u * dt
        uc = self.uc
        if ub > uc + lim:
            ub = uc + lim
        elif ub < uc - lim:
            ub = uc - lim
        self.uc = ub
        return ub
