# schiene_messen.py: Messungen an der Schiene (Ball on Beam), über die Brücke oder MicroPico.
# Zusammengelegt aus karte.py und scan.py (06.11.2026), Funktion unverändert.
#
# karte(dauer, rate, schwelle): Wo rollt der Ball los? Ball ruhig -> Servo langsam in eine
#   Richtung kippen bis er losrollt -> bremsen -> ruhig -> andere Richtung. (Auftrag 0003)
#   D,t,s,x,u,phase              jede Messung (phase: W warten, R rampe, S stoppen)
#   B,x_start,u_los,richtung     Losbrechwinkel (u relativ zu OFFSET)
#   N,x_start,u_grenze,richtung  kein Losrollen bis zum Stellbereich
#
# scan(durchgaenge, rate, basis, level, t_max): Klemmstellen über die ganze Schiene.
#   Ball von Ende zu Ende schrittweise: Servo langsam kippen bis er losrollt, dann nur noch
#   leicht kippen (rollt aus, bleibt an der nächsten Klemmstelle). (Aufträge 0004, 0005)
#   D,t,s,x,u,phase
#   B,durchgang,x_ruhe,u_los,richtung     Losbrechen
#   E,durchgang,richtung                   Ende erreicht
#
# Braucht auf dem Pico: bob.py, bob_core.py, kennlinie.py, vl53l0x.py (Ordner Pico_Programme).
# Beispiel: import schiene_messen; schiene_messen.scan(3, 3.0, 4.0, 1.5, 150.0)
import bob
import bob_core as core


def karte(dauer=150.0, rate=4.0, schwelle=4.0):
    h = bob.io()
    m = core.Messung()
    off = core.OFFSET
    u = 0.0
    h.servo(off)
    t_end = h.zeit() + dauer
    richtung = -1
    print('# karte dauer', dauer, 'rate', rate, 'schwelle', schwelle, 'offset', off)

    def lese(ph):
        s = h.ping()
        x, ok = m.update(s, 0.03)
        print('D,%.3f,%d,%.1f,%.2f,%s' % (h.zeit(), s, x, u, ph))
        return x, ok

    while h.zeit() < t_end:
        # 1) warten bis ruhig (0.8 s innerhalb 3 mm, max 6 s)
        ts = h.zeit()
        x_ref = None
        t_ref = ts
        xs = []
        while True:
            x, ok = lese('W')
            if not ok:
                continue
            if x_ref is None or abs(x - x_ref) > 3:
                x_ref = x
                t_ref = h.zeit()
                xs = [x]
            else:
                xs.append(x)
                if h.zeit() - t_ref > 0.8:
                    break
            if h.zeit() - ts > 6:
                break
        xs.sort()
        x0 = xs[len(xs) // 2]
        if x0 < 20:
            richtung = 1
        elif x0 > 320:
            richtung = -1
        # 2) Rampe bis Losrollen (2 Messungen in Folge > schwelle weg)
        n_weg = 0
        u_lo = -core.U_ABS - off
        u_hi = core.U_ABS - off
        t_last = h.zeit()
        los = False
        while True:
            x, ok = lese('R')
            t = h.zeit()
            u += richtung * rate * (t - t_last)
            t_last = t
            if u < u_lo or u > u_hi:
                u = max(u_lo, min(u_hi, u))
                print('N,%.1f,%.2f,%d' % (x0, u, richtung))
                break
            h.servo(off + u)
            if ok and (x - x0) * richtung > schwelle:
                n_weg += 1
                if n_weg >= 2:
                    los = True
                    break
            else:
                n_weg = 0
        if los:
            print('B,%.1f,%.2f,%d' % (x0, u, richtung))
            u_los = u
            # 3) bremsen: kurz Gegenneigung, dann nahe Losbrechwinkel minus Reserve
            u = u_los - richtung * 12
            h.servo(off + u)
            for i in range(8):
                lese('S')
            u = u_los - richtung * 6
            h.servo(off + u)
        richtung = -richtung
    u = 0.0
    h.servo(off)
    print('# karte fertig')


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
