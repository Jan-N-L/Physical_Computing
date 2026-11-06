# demo_kombi.py: Live-Vorführung Ball on Beam mit dem kombinierten Regler.
# Ablauf: Ball irgendwo auf die Schiene legen -> Regler startet automatisch, Uhr läuft.
#         "RUHE nach x.x s" = Ball 1 s durchgehend in ±10 mm um die Mitte.
#         Ball wegnehmen -> Schiene waagrecht, warten auf den nächsten Ball.
# Auf dem Pico nötig: vl53l0x.py, kennlinie.py, bob_core.py, bob.py, ctrl_mpc.py, ctrl_kombi.py
# Start: in VS Code diese Datei öffnen, MicroPico "Run". (Oder als main.py speichern = Autostart.)
import Ball_on_beam.ALT.Kombi.bob as bob
import Ball_on_beam.ALT.Kombi.bob_core as core
import Ball_on_beam.ALT.Kombi.ctrl_kombi as ctrl_kombi

OFFSET = core.OFFSET


def warte_auf_ball(h, m):
    h.servo(OFFSET)
    print('Warte auf Ball ...')
    n = 0
    x_ref = None
    while True:
        s = h.ping()
        x, ok = m.update(s, 0.03)
        if s >= core.S_LEER:
            n = 0
            x_ref = None
            continue
        if x_ref is None or abs(x - x_ref) > 4:
            x_ref = x
            n = 0
        n += 1
        if n >= 15:                       # ca. 0.45 s ruhig
            return x


def lauf(h):
    m = core.Messung()
    x0 = warte_auf_ball(h, m)
    print('Ball erkannt bei %.0f mm, Start' % x0)
    reg = ctrl_kombi.Regler()
    reg.reset(x0)
    r = core.Ruhe()
    t0 = h.zeit()
    t_last = t0
    t_print = t0
    gemeldet = False
    fail = False
    leer = 0
    while True:
        s = h.ping()
        t = h.zeit()
        dt = t - t_last
        t_last = t
        if dt <= 0:
            dt = 0.029
        leer = leer + 1 if s >= core.S_LEER else 0
        if leer > 15:
            h.servo(OFFSET)
            print('Ball weg')
            return
        x, ok = m.update(s, dt)
        u = reg.step(x, ok, dt)
        h.servo(core.clamp(OFFSET + u, -core.U_ABS, core.U_ABS))
        tt = t - t0
        r.update(x, tt)
        if r.t_ok is not None and not gemeldet:
            print('RUHE nach %.1f s (bei %.0f mm)' % (r.t_ok, x))
            gemeldet = True
        if r.t_ok is None and gemeldet:
            gemeldet = False
            print('Ball hat das Ziel verlassen, regle weiter')
        if t - t_print > 0.5:
            t_print = t
            print('t=%5.1f s  x=%5.0f mm  u=%5.1f°' % (tt, x, u))
        if tt > core.T_FAIL and r.t_ok is None and not fail:
            print('Über 30 s: Fail, regle weiter')
            fail = True


def main():
    h = bob.io()
    try:
        while True:
            lauf(h)
    except KeyboardInterrupt:
        h.servo(OFFSET)
        print('Stopp')


main()
