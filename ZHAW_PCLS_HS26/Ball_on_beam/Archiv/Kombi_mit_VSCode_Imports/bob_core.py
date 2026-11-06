# bob_core.py: Kern für Ball on Beam, läuft gleich auf dem Pico und im Simulator.
# Kennt keine Hardware. Die Hardware kommt über ein io-Objekt mit
#   io.ping()      -> Sensorwert (blockiert ca. 29 ms, wie tof.ping())
#   io.zeit()      -> Sekunden (float, steigend)
#   io.servo(c)    -> absoluter Servowinkel in Grad (wird hier auf ±30 begrenzt)
#
# Regler-Interface (jede Methode, Datei ctrl_<name>.py, Klasse Regler):
#   r = Regler()
#   r.reset(x)               x = Startposition in echten mm
#   u = r.step(x, ok, dt)    x = Position mm (letzter gültiger Wert, falls ok False)
#                            ok = False: dieser Messwert war ungültig/verworfen
#                            dt = Zeit seit letztem Aufruf in s (ca. 0.029)
#                            u  = Servowinkel in Grad RELATIV zu OFFSET (Waagrecht-Schätzung)
#   Positives u -> Ball rollt weg vom Sensor (zu grösseren x).
#   Ziel: MITTE_MM = 169.5 mm.  Stellbereich absolut ±30°, also u etwa -31.9 .. +28.1.

from Ball_on_beam.ALT.Kombi.kennlinie import to_mm, MITTE_MM

X_END = 339.0       # mm Anschlag Klotz (Ballvorderkante), 0 = Anschlag Sensor
TOL = 10.0          # mm um die Mitte
T_RUHE = 1.0        # s durchgehend in TOL = "zur Ruhe"
T_STAY = 3.0        # s nach dem Ruhe-Fenster weiter beobachten (bleibt er liegen?)
T_FAIL = 30.0       # s, Ruhe-Fenster muss bis hier abgeschlossen sein
U_ABS = 30.0        # ° Servo absolut
S_LEER = 440        # Sensorwert ab hier: kein Ball / ungültig (Klotz 419, leer 460+)
OFFSET = 1.9        # ° Servo, Startwert waagrecht


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Messung:
    """Sensorwert -> mm. Verwirft Ausreisser, rastet aber nie ein:
    nach max_verworfen Werten in Folge wird der neue Wert akzeptiert."""

    def __init__(self, sprung=30.0, max_verworfen=3):
        self.sprung = sprung
        self.max_v = max_verworfen
        self.x = None
        self.n = 0

    def update(self, s, dt):
        if s >= S_LEER:
            self.n += 1
            return (self.x if self.x is not None else X_END), False
        x = clamp(to_mm(s), -5.0, X_END + 5.0)
        if self.x is not None and self.n < self.max_v and abs(x - self.x) > self.sprung + 500.0 * dt * (self.n + 1):
            self.n += 1
            return self.x, False
        self.n = 0
        self.x = x
        return x, True


class AlphaBeta:
    """Einfacher Schätzer für Position und Tempo (mm, mm/s). Optional für Regler."""

    def __init__(self, a=0.5, b=0.15):
        self.a = a
        self.b = b
        self.x = None
        self.v = 0.0

    def reset(self, x):
        self.x = x
        self.v = 0.0

    def update(self, x, ok, dt):
        if self.x is None:
            self.x = x
        xp = self.x + self.v * dt
        if ok:
            r = x - xp
            self.x = xp + self.a * r
            self.v = self.v + self.b * r / dt
        else:
            self.x = xp
            self.v *= 0.9
        return self.x, self.v


class Ruhe:
    """Bewertung: Ball durchgehend T_RUHE in ±TOL um die Mitte.
    t_ok = Beginn des Ruhe-Fensters (das ist die gemeldete Zeit)."""

    def __init__(self, ziel=MITTE_MM, tol=TOL):
        self.ziel = ziel
        self.tol = tol
        self.t_in = None
        self.t_ok = None
        self.verlassen = 0

    def update(self, x, t):
        if abs(x - self.ziel) <= self.tol:
            if self.t_in is None:
                self.t_in = t
            if self.t_ok is None and t - self.t_in >= T_RUHE and self.t_in <= T_FAIL - T_RUHE:
                self.t_ok = self.t_in
        else:
            if self.t_ok is not None:
                self.verlassen += 1
            self.t_in = None
            self.t_ok = None
        # fertig?
        if self.t_ok is not None and t - self.t_ok >= T_RUHE + T_STAY:
            return True
        if self.t_ok is None and t > T_FAIL:
            return True
        return False


def rampe(io, von, nach, dauer):
    t0 = io.zeit()
    while True:
        io.ping()
        f = (io.zeit() - t0) / dauer
        if f >= 1:
            break
        io.servo(clamp(von + (nach - von) * f, -U_ABS, U_ABS))
    io.servo(clamp(nach, -U_ABS, U_ABS))


def vorbereiten(io, start, offset=OFFSET, kipp=20.0, t_max=12.0):
    """Ball an ein Ende rollen lassen und ruhig hinlegen. start: 'sensor', 'klotz' oder None.
    Gibt die gemessene Startposition (mm) zurück."""
    m = Messung()
    if start is None:
        io.servo(offset)
    else:
        z = offset - kipp if start == 'sensor' else offset + kipp
        rampe(io, offset, z, 0.4)
        t0 = io.zeit()
        x_ref = None
        t_ref = t0
        while io.zeit() - t0 < t_max:
            x, ok = m.update(io.ping(), 0.03)
            am_ende = x < 25 if start == 'sensor' else x > 315
            if not am_ende:
                x_ref = None
                continue
            if x_ref is None or abs(x - x_ref) > 4:
                x_ref = x
                t_ref = io.zeit()
            elif io.zeit() - t_ref > 1.0:
                break
        rampe(io, z, offset, 0.4)
    t0 = io.zeit()
    xs = []
    while io.zeit() - t0 < 0.5:
        x, ok = m.update(io.ping(), 0.03)
        if ok:
            xs.append(x)
    xs.sort()
    return xs[len(xs) // 2] if xs else m.x


def versuch(io, regler, start='sensor', offset=OFFSET, log=None):
    """Ein Versuch. log(t, s, x, ok, u) wird pro Schritt aufgerufen (optional).
    Rückgabe dict: ok, T (s, Beginn Ruhe-Fenster), verlassen, x0, n, servo_weg (°)"""
    x0 = vorbereiten(io, start, offset)
    m = Messung()
    m.x = x0
    r = Ruhe()
    regler.reset(x0)
    t0 = io.zeit()
    t_last = t0
    u_last = 0.0
    weg = 0.0
    n = 0
    while True:
        s = io.ping()
        t = io.zeit()
        dt = t - t_last
        if dt <= 0:
            dt = 0.029
        t_last = t
        x, ok = m.update(s, dt)
        u = regler.step(x, ok, dt)
        c = clamp(offset + u, -U_ABS, U_ABS)
        io.servo(c)
        weg += abs(c - offset - u_last)
        u_last = c - offset
        n += 1
        tt = t - t0
        if log:
            log(tt, s, x, ok, c - offset)
        if r.update(x, tt):
            break
    return {'ok': r.t_ok is not None, 'T': r.t_ok, 'verlassen': r.verlassen,
            'x0': x0, 'n': n, 'servo_weg': weg, 'dauer': t_last - t0}
