# bob.py: Hardware-Teil für den Pico (Ball on Beam). Kern in bob_core.py.
# Benutzung über die Brücke, z.B.:
#   import bob; bob.reihe('ctrl_pid', ['sensor', 'klotz'])
from machine import Pin, I2C, PWM
from utime import ticks_us, ticks_diff
from vl53l0x import VL53L0X
import bob_core as core


class PicoIO:
    def __init__(self):
        self.tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
        self.pwm = PWM(Pin(16))
        self.pwm.freq(50)
        self._last = ticks_us()
        self._acc = 0
        self._tp = self._last
        self.rz_sum = 0
        self.rz_max = 0
        self.rz_n = 0

    def ping(self):
        s = self.tof.ping()
        self._tp = ticks_us()
        return s

    def zeit(self):
        now = ticks_us()
        self._acc += ticks_diff(now, self._last)
        self._last = now
        return self._acc / 1e6

    def servo(self, c):
        d = ticks_diff(ticks_us(), self._tp)      # Rechenzeit seit Messung
        self.rz_sum += d
        self.rz_n += 1
        if d > self.rz_max:
            self.rz_max = d
        c = core.clamp(c, -core.U_ABS, core.U_ABS)
        self.pwm.duty_u16(int((c + 90) / 180 * 8000 + 1000))


_io = None


def io():
    global _io
    if _io is None:
        _io = PicoIO()
    return _io


def reihe(modname, starts=('sensor', 'klotz'), kwargs=None, offset=core.OFFSET, spur=True):
    """Mehrere Versuche mit einem Regler. Gibt pro Schritt D-Zeilen und pro Versuch eine R-Zeile aus."""
    mod = __import__(modname)
    h = io()
    print('# modul', modname, 'kwargs', kwargs, 'offset', offset)
    print('# D,t,s,x,ok,u')

    def lg(t, s, x, ok, u):
        print('D,%.3f,%d,%.1f,%d,%.2f' % (t, s, x, 1 if ok else 0, u))

    for st in starts:
        reg = mod.Regler(**kwargs) if kwargs else mod.Regler()
        h.rz_sum = h.rz_max = h.rz_n = 0
        r = core.versuch(h, reg, st, offset, lg if spur else None)
        h.servo(offset)
        print('R,%s,%s,%s,%d,%.1f,%.1f,%d,%.2f,%d,%d' % (
            modname, st, 'OK' if r['ok'] else 'FAIL', -1 if r['T'] is None else int(r['T'] * 1000),
            r['x0'], r['servo_weg'], r['verlassen'], r['dauer'],
            h.rz_sum // max(h.rz_n, 1), h.rz_max))
    h.servo(offset)
    print('# fertig')
