# Prüft main.py am PC mit festen Messwerten. Nicht für den Pico.
# Start: python test_main_sim.py
# ponytail: feste Messwerte ohne Ballmodell, ersetzt keinen Test an der Hardware
import contextlib, io, os, sys, types

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "main.py")
DT_US = 30000       # ping dauert ca. 30 ms


class Done(Exception):
    pass


def run(pings):
    """Führt main.py aus, bis pings leer sind.
    Gibt (Globals, Plot-Zeilen P/I/D/u/hold, letzter Duty) zurück."""
    s = dict(t=0, duty=None)
    pings = iter(pings)

    class VL53L0X:
        def __init__(self, i2c): pass
        def ping(self):
            s["t"] += DT_US
            for x in pings:
                return x
            raise Done

    class PWM:
        def __init__(self, pin): pass
        def freq(self, f): pass
        def duty_u16(self, d): s["duty"] = d

    machine = types.ModuleType("machine")
    machine.Pin, machine.I2C, machine.PWM = (lambda *a: None), (lambda **k: None), PWM
    vl53l0x = types.ModuleType("vl53l0x")
    vl53l0x.VL53L0X = VL53L0X
    utime = types.ModuleType("utime")
    utime.ticks_us = lambda: s["t"]
    utime.ticks_ms = lambda: s["t"]//1000
    utime.ticks_diff = lambda a, b: a - b
    sys.modules.update(machine=machine, vl53l0x=vl53l0x, utime=utime)

    g = {"__name__": "__main__"}
    out = io.StringIO()
    with open(SCRIPT, encoding="utf-8") as f, contextlib.redirect_stdout(out):
        try:
            exec(compile(f.read(), SCRIPT, "exec"), g)
        except Done:
            pass
    rows = [[float(v) for v in line.split(",")] for line in out.getvalue().splitlines()[1:]]
    return g, rows, s["duty"]


REST = [209]*50                         # Ball liegt bei e = -9, ausserhalb Sollbereich
MOVING = [209 + 3*k for k in range(30)] # Ball rollt mit 100 mm/s vom Sensor weg
STEADY = [200]*150                      # reicht für T_STEADY

# Einzelner Fehlmesswert: Stellgrösse bleibt ruhig
u = [r[3] for r in run(REST + [8190] + REST)[1]]
assert max(u) - min(u) < 2, "Fehlmesswert stört u: %.1f..%.1f" % (min(u), max(u))

# Fehlmesswert in der Steady-Schlaufe: bleibt drin, keine neuen Plot-Zeilen
assert len(run(STEADY + [8190] + [200]*20)[1]) == len(run(STEADY + [200]*21)[1]), "Steady verlassen"

# Sensor lange blind: e läuft nicht davon, Trend klingt ab
g = run(MOVING + [8190]*500)[0]
assert -130 < g["e"] < -90 and abs(g["de"]) < 1, "e=%.1f de=%.1f" % (g["e"], g["de"])

# u = 0 -> Schiene waagrecht
g, _, duty = run([200]*5)
assert duty == int((g["OFFSET"] + 90)/180*8000 + 1000), "OFFSET fehlt: Duty %s" % duty

# Erste Messung ungültig: kein Riesenausschlag beim Start
p = min(r[0] for r in run([8190] + REST)[1])
assert p > -40, "Startausschlag P=%.1f" % p

print("OK")
