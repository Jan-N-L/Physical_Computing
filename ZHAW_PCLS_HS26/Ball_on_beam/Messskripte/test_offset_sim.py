# Prüft test_offset.py am PC gegen einen simulierten Ball. Nicht für den Pico.
# Start: python test_offset_sim.py
# ponytail: einfaches Ballmodell (Haftreibung + Hangabtrieb), ersetzt keinen Test an der Hardware
import contextlib, io, os, random, sys, types

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_offset.py")
LENGTH = 400.0      # mm  Schienenlänge
ACCEL = 122.0       # mm/s² pro Grad, rollende Vollkugel


def run(level, dead, kick=False, blind=False):
    """Führt test_offset.py aus. Gibt (gefundener Winkel, Dauer in s) zurück."""
    s = dict(angle=0.0, p=200.0, v=0.0, t=0.0)

    def sleep_ms(ms):
        for _ in range(int(ms)):
            e = s["angle"] - level
            if s["v"] or abs(e) > dead/2:       # Haftreibung hält den Ball sonst fest
                s["v"] += ACCEL*e*0.001
                s["p"] += s["v"]*0.001
                if not 0 <= s["p"] <= LENGTH:   # Anschlag am Schienenende
                    s["p"], s["v"] = max(0.0, min(LENGTH, s["p"])), 0.0
        s["t"] += ms/1000

    class PWM:
        def __init__(self, pin): pass
        def freq(self, f): pass
        def duty_u16(self, d):
            new = (d - 1000)/8000*180 - 90
            if kick and abs(new - s["angle"]) > 5 and s["p"] < 5:
                s["p"] += 40                    # Servo-Ruck wirft den Ball vom Ende weg
            s["angle"] = new

    class VL53L0X:
        def __init__(self, i2c): pass
        def ping(self):
            sleep_ms(30)
            return 8190 if blind else int(s["p"] + 30 + random.gauss(0, 2))

    machine = types.ModuleType("machine")
    machine.Pin, machine.I2C, machine.PWM = (lambda *a: None), (lambda **k: None), PWM
    vl53l0x = types.ModuleType("vl53l0x")
    vl53l0x.VL53L0X = VL53L0X
    utime = types.ModuleType("utime")
    utime.sleep_ms = sleep_ms
    sys.modules.update(machine=machine, vl53l0x=vl53l0x, utime=utime)

    g = {"__name__": "__main__"}
    with open(SCRIPT, encoding="utf-8") as f, contextlib.redirect_stdout(io.StringIO()):
        exec(compile(f.read(), SCRIPT, "exec"), g)
    return g["level"], s["t"]


random.seed(1)
for level, dead, kick in [(0, 1, False), (2.3, 1.5, False), (-3.7, 3, False), (5, 0.5, False),
                          (-8, 4, False), (1.3, 2, True)]:
    found, secs = run(level, dead, kick)
    print("wahr %5.2f, gefunden %5.2f, %3.0f s%s" % (level, found, secs, ", Ruck" if kick else ""))
    assert abs(found - level) < 0.5
    assert secs < 120

assert run(0, 1, blind=True)[0] is None     # Sensor blind: Abbruch ohne Ergebnis
print("OK")
