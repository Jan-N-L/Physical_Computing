# karte.py: Schienen-Karte. Wo rollt der Ball los? (lokales Waagrecht + Haftreibung)
# Ablauf pro Zyklus: Ball ruhig -> Servo langsam in eine Richtung kippen bis der Ball losrollt
# -> bremsen -> ruhig -> andere Richtung. Ausgabe:
#   D,t,s,x,u,phase     jede Messung (phase: W warten, R rampe, S stoppen)
#   B,x_start,u_los,richtung     Losbrechwinkel (u relativ zu OFFSET)
#   N,x_start,u_grenze,richtung  kein Losrollen bis zum Stellbereich
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
