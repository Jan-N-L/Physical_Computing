# ctrl_fuzzy.py: Fuzzy-Regler für Ball on Beam (Pico und Simulator)
#
# Eingänge (unscharf eingeteilt, Dreiecks-Zugehörigkeit = lineare Aufteilung
# zwischen zwei Stützpunkten):
#   e = MITTE - x  in mm     (positiv: Ball zu nah am Sensor)
#   v = Tempo      in mm/s   (positiv: Ball rollt vom Sensor weg)
# Ausgang: u in ° Servo relativ zum Offset (positiv: Ball rollt vom Sensor weg).
#
# Regelbasis (Sugeno, Produkt- bzw. min-Verknüpfung, gewichtete Summe):
#   A) Haupttabelle 9 x 9 (e, v): «Ball weit weg und ruhig: stark kippen» usw.
#      Einträge aus einer Bremskurve: Wunschtempo v_soll(e) wächst linear
#      (nahe Mitte) und dann mit der Wurzel (fern), gekippt wird proportional
#      zur Tempo-Abweichung, begrenzt auf den Stellbereich.
#   B) Haltewinkel lernen: Ball nahe Mitte -> Haltewert langsam Richtung
#      Fehler schieben (I-Anteil). Gleicht Offset UND die Ballstörung am
#      Zielort aus (die braucht bis ca. ±25°).
#   C) Ball steckt (ruhig UND nicht Mitte): zusätzlicher Stoss Richtung Mitte,
#      wächst mit der Wartezeit bis fast zum vollen Stellbereich (Mulden).
#   D) Rollreibung: Ball rollt -> zusätzlich FK_ROLL in Laufrichtung.
#   F) Schaukeln: steckt der Ball trotz vollem Stoss fest, «zuerst zurück
#      rollen lassen, dann voll Richtung Mitte» (Anlauf durch die Mulde).
#   E) Ruhezone: Ball in der Mitte UND ruhig -> Ausgang einfrieren. Die
#      Ballstörung hängt nur von der Position ab: ruht der Ball, bleibt er
#      bei festem Winkel liegen. Jede Bewegung des Servos stört nur.
#
# Nur bob_core: läuft auf MicroPython.

from bob_core import AlphaBeta, MITTE_MM

E_P = (-170.0, -80.0, -30.0, -10.0, 0.0, 10.0, 30.0, 80.0, 170.0)   # mm
V_P = (-250.0, -120.0, -50.0, -15.0, 0.0, 15.0, 50.0, 120.0, 250.0)  # mm/s


def _idx(p, x):
    """Index i und Gewicht w des rechten Stützpunkts: x liegt zwischen p[i] und p[i+1]."""
    n = len(p)
    if x <= p[0]:
        return 0, 0.0
    if x >= p[n - 1]:
        return n - 2, 1.0
    i = 0
    while x > p[i + 1]:
        i += 1
    return i, (x - p[i]) / (p[i + 1] - p[i])


def _cl(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def tabelle(kv=0.30, t_lin=0.9, a_brems=110.0, v_max=220.0, u_max=24.0, kp0=0.0):
    """Erzeugt die Regeltabelle (Zeile e, Spalte v) aus der Bremskurve.
    kp0: zusätzliche Steifigkeit (°/mm) nahe der Mitte, nur bis ±30 mm wirksam."""
    tab = []
    for e in E_P:
        ae = abs(e)
        vs = (a_brems * a_brems * t_lin * t_lin + 2.0 * a_brems * ae) ** 0.5 - a_brems * t_lin
        if vs > v_max:
            vs = v_max
        if e < 0:
            vs = -vs
        ep = e if ae <= 30.0 else (30.0 if e > 0 else -30.0)
        row = []
        for v in V_P:
            row.append(_cl(kv * (vs - v) + kp0 * ep, -u_max, u_max))
        tab.append(row)
    return tab


class Regler:
    def __init__(self, kv=0.3, t_lin=0.72, a_brems=110.0, v_max=220.0, u_max=24.0, kp0=0.6,
                 fk_haft=4.0, fk_zeit=15.0, fk_haft_max=25.0, v_ruhig=15.6, fk_roll=0.0, v_roll=30.0,
                 e_zone=3.5, e_zone_b=2.5, v_zone=15.0, ki=0.2, e_i=40.0, i_max=25.0,
                 t_vor=0.04, du_max=150.0, ab_a=0.5, ab_b=0.15,
                 t_pump=1.7, u_back=10.0, t_pmax=6.0, e_pend=8.0, v_fest=40.0, t_back=0.0, d_back=15.0,
                 u_lim=28.0, kd0=0.0, e_d=30.0, tab=None):
        self.tab = tab if tab is not None else tabelle(kv, t_lin, a_brems, v_max, u_max, kp0)
        self.fk_haft = fk_haft
        self.fk_zeit = fk_zeit
        self.fk_haft_max = fk_haft_max
        self.v_ruhig = v_ruhig
        self.fk_roll = fk_roll
        self.v_roll = v_roll
        self.e_zone = e_zone
        self.e_zone_b = e_zone_b
        self.v_zone = v_zone
        self.ki = ki
        self.e_i = e_i
        self.i_max = i_max
        self.t_vor = t_vor
        self.du_max = du_max
        self.u_lim = u_lim
        self.kd0 = kd0
        self.e_d = e_d
        self.t_pump = t_pump
        self.u_back = u_back
        self.t_pmax = t_pmax
        self.e_pend = e_pend
        self.v_fest = v_fest
        self.d_back = d_back
        self.t_back = t_back
        self.f = AlphaBeta(ab_a, ab_b)
        self.reset(MITTE_MM)

    def reset(self, x):
        self.f.reset(x)
        self.u = 0.0
        self.halt = 0.0
        self.t_steck = 0.0
        self.t_p = -1.0
        self.t_fest = 0.0
        self.ph = 0
        self.x_p = x
        self.t_ph = 0.0

    def step(self, x, ok, dt):
        x, v = self.f.update(x, ok, dt)
        e = MITTE_MM - (x + v * self.t_vor)
        # A) Haupttabelle, nur die 4 aktiven Regeln
        i, wi = _idx(E_P, e)
        j, wj = _idx(V_P, v)
        r0 = self.tab[i]
        r1 = self.tab[i + 1]
        u = (1.0 - wi) * ((1.0 - wj) * r0[j] + wj * r0[j + 1]) + wi * ((1.0 - wj) * r1[j] + wj * r1[j + 1])
        ae = e if e > 0 else -e
        av = v if v > 0 else -v
        # Zugehörigkeiten
        ruhig = 1.0 - av / self.v_ruhig
        if ruhig < 0.0:
            ruhig = 0.0
        mitte = _cl((self.e_zone + self.e_zone_b - ae) / self.e_zone_b, 0.0, 1.0)
        # C) Ball steckt: ruhig UND nicht Mitte (min-Verknüpfung)
        w = ruhig if ruhig < 1.0 - mitte else 1.0 - mitte
        if w > 0.0:
            self.t_steck += w * dt
        else:
            self.t_steck = 0.0
        fk = self.fk_haft + self.fk_zeit * self.t_steck
        if fk > self.fk_haft_max:
            fk = self.fk_haft_max
        u += w * fk * (1.0 if e > 0 else -1.0)
        # A2) Zusatzdämpfung nahe Mitte: «nahe Mitte und schnell: stark bremsen»
        if self.kd0 > 0.0 and ae < self.e_d:
            u -= self.kd0 * v * (1.0 - ae / self.e_d)
        # D) Rollreibung in Laufrichtung
        if self.fk_roll > 0.0:
            r = av / self.v_roll
            if r > 1.0:
                r = 1.0
            u += self.fk_roll * r * (1.0 if v > 0 else -1.0)
        # E) Ruhezone: Mitte UND ruhig
        z = 1.0 - av / self.v_zone
        if z < 0.0:
            z = 0.0
        wz = mitte if mitte < z else z
        # B) Haltewinkel lernen (nahe Mitte, ausserhalb der Ruhezone)
        if self.ki > 0.0 and ae < self.e_i:
            self.halt = _cl(self.halt + (1.0 - wz) * self.ki * e * dt, -self.i_max, self.i_max)
        u = (1.0 - wz) * (u + self.halt) + wz * self.u
        # F) Schaukeln (Anlauf nehmen), siehe Kopf
        if self.t_pump > 0.0:
            if ae > self.e_pend and av < self.v_fest:
                self.t_fest += dt
            elif self.t_fest > 0.0:
                self.t_fest -= 2.0 * dt
            sg = 1.0 if e > 0 else -1.0
            if self.t_p < 0.0 and self.t_fest > self.t_pump:
                self.t_p = 0.0          # Anlauf nehmen: zuerst zurückrollen lassen
                self.ph = 0
                self.x_p = x
                self.t_ph = 0.0
            if self.t_p >= 0.0:
                self.t_p += dt
                self.t_ph += dt
                if ae < self.e_pend or self.t_p > self.t_pmax:
                    self.t_p = -1.0
                    self.t_fest = 0.0
                elif self.t_back <= 0.0:
                    # passiv: rollt er zurück, Schiene mitkippen, sonst voll Richtung Mitte
                    u = -sg * self.u_back if v * sg < -10.0 else sg * self.u_lim
                elif self.ph == 0:
                    u = -sg * self.u_back
                    if (self.x_p - x) * sg > self.d_back or self.t_ph > self.t_back:
                        self.ph = 1
                        self.t_ph = 0.0
                else:
                    u = sg * self.u_lim
                    if v * sg < -10.0 and self.t_ph > 0.3:
                        self.ph = 0
                        self.x_p = x
                        self.t_ph = 0.0
        # Begrenzung und Rampe (grosse Sprünge stören den Sensor)
        u = _cl(u, -self.u_lim, self.u_lim)
        du = self.du_max * dt
        if u > self.u + du:
            u = self.u + du
        elif u < self.u - du:
            u = self.u - du
        self.u = u
        return u
