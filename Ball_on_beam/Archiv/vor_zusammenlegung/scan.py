# scan.py: Klemmstellen über die ganze Schiene suchen.
# Ball von Ende zu Ende schrittweise: Servo langsam kippen bis er losrollt, dann nur noch
# leicht kippen (rollt aus und bleibt an der nächsten Klemmstelle stehen). Mehrere Durchgänge.
#   D,t,s,x,u,phase
#   B,durchgang,x_ruhe,u_los,richtung     Losbrechen
#   E,durchgang,richtung                   Ende erreicht
import bob
import bob_core as core


def scan(durchgaenge=3, rate=3.0, basis=5.0, level=3.0, t_max=200.0):
    h = bob.io()
    m = core.Messung()
    off = core.OFFSET
    st = {'u': 0.0}
    print('# scan rate', rate, 'basis', basis, 'level', level)

    def lese(ph):
        s = h.ping()
        x, ok = m.update(s, 0.03)
        print('D,%.3f,%d,%.1f,%.2f,%s' % (h.zeit(), s, x, st['u'], ph))
        return x, ok

    def setz(u):
        st['u'] = max(-core.U_ABS - off, min(core.U_ABS - off, u))
        h.servo(off + st['u'])

    core.vorbereiten(h, 'sensor', off)
    richtung = 1
    t0 = h.zeit()
    for d in range(durchgaenge):
        while h.zeit() - t0 < t_max:
            # ruhig?
            ts = h.zeit()
            x_ref = None
            t_ref = ts
            xs = []
            while h.zeit() - ts < 8:
                x, ok = lese('W')
                if not ok:
                    continue
                if x_ref is None or abs(x - x_ref) > 2.5:
                    x_ref, t_ref, xs = x, h.zeit(), [x]
                else:
                    xs.append(x)
                    if h.zeit() - t_ref > 0.6:
                        break
            xs.sort()
            x0 = xs[len(xs) // 2]
            if (richtung > 0 and x0 > 318) or (richtung < 0 and x0 < 18):
                print('E,%d,%d' % (d, richtung))
                break
            # Rampe ab Basis-Neigung
            u = level + richtung * basis
            setz(u)
            t_last = h.zeit()
            n = 0
            while True:
                x, ok = lese('R')
                t = h.zeit()
                u += richtung * rate * (t - t_last)
                t_last = t
                setz(u)
                if ok and (x - x0) * richtung > 4:
                    n += 1
                    if n >= 2:
                        break
                else:
                    n = 0
                if abs(u - level) > 33:
                    break
            print('B,%d,%.1f,%.2f,%d' % (d, x0, st['u'], richtung))
            setz(level + richtung * basis)
        richtung = -richtung
    setz(0.0)
    print('# scan fertig')
