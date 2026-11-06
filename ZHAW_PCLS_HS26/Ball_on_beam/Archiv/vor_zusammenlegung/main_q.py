# main_q.py: Ball on Beam mit main_v3 + Q-Learning (Live-Programm).
# Beim Start schaut es, auf welcher Seite der Ball liegt (Zustand), und wählt aus der
# Q-Tabelle (q_main.json) den Parametersatz mit der besten bisherigen Zeit.
# Nach jedem Lauf ("RUHE nach x s" oder 30 s Fail) wird die Tabelle nachgeführt und gespeichert.
#
# Auf dem Pico nötig: vl53l0x.py, kennlinie.py, bob_core.py, main_v3.py, qlern.py, q_main.json,
#                    ctrl_mpc.py, ctrl_kombi.py (für Aktion A4)
# Start: in VS Code öffnen, MicroPico "Run".
# LERNEN = False: nur das Gelernte nutzen, Tabelle nicht verändern (sicher für die Vorführung).

from kennlinie import to_mm, MITTE_MM
import main_v3
import qlern

LERNEN = True
EPS_LIVE = 0.0      # 0 = immer die beste Aktion (kein Erkunden in der Vorführung)


def run():
    from machine import Pin, I2C, PWM
    from vl53l0x import VL53L0X
    from utime import ticks_us, ticks_diff

    tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
    servo = PWM(Pin(16))
    servo.freq(50)
    clamp = main_v3.clamp

    def set_angle(u):
        a = clamp(u + main_v3.OFFSET, -main_v3.U_ABS, main_v3.U_ABS)
        servo.duty_u16(int((a + 90) / 180 * 8000 + 1000))

    q = qlern.laden()
    qlern.zeige(q)
    set_angle(0)
    reg = None
    x_last = None
    n_bad = 0
    n_still = 0
    n_da = 0
    x_ref = None
    t_prev = ticks_us()
    t_start = None
    t_in = None
    ruhe = False
    gelernt = False
    s_q = a_q = None
    n_print = 0
    print("Warte auf Ball ...")
    while True:
        s = tof.ping()
        t = ticks_us()
        dt = ticks_diff(t, t_prev) / 1e6
        t_prev = t
        if dt <= 0:
            dt = 0.03

        ok = s < main_v3.S_LEER
        x = None
        if ok:
            x = to_mm(s)
            if x_last is not None and n_bad < 3 and abs(x - x_last) > 30 + 500 * dt * (n_bad + 1):
                ok = False
        if ok:
            n_bad = 0
            if t_start is None:
                if x_ref is None or abs(x - x_ref) > 5:
                    x_ref = x
                    n_still = 0
                else:
                    n_still += 1
            x_last = x
        else:
            n_bad += 1

        if x_last is None:
            continue
        if n_bad > 15:                          # Ball weg
            set_angle(0)
            print("Ball weg")
            x_last = None
            t_start = None
            ruhe = False
            n_still = 0
            n_da = 0
            x_ref = None
            continue
        if t_start is None:
            n_da += 1
            if n_still < 10 and n_da < 50:      # starten, wenn ruhig, spätestens nach ca. 1.5 s
                continue
            s_q = qlern.zustand(x_last)
            a_q = qlern.waehle(q, s_q, EPS_LIVE)
            reg = qlern.make(a_q)
            reg.reset(x_last)
            t_start = t
            t_in = None
            gelernt = False
            print("Start bei %.0f mm, Zustand %s, Aktion %s" % (x_last, s_q, qlern.NAMEN[a_q]))

        u = reg.step(x_last, ok, dt)
        set_angle(u)

        tt = ticks_diff(t, t_start) / 1e6
        if abs(x_last - MITTE_MM) <= main_v3.TOL:
            if t_in is None:
                t_in = tt
            if not ruhe and tt - t_in >= main_v3.T_RUHE:
                ruhe = True
                print("RUHE nach %.1f s" % t_in)
                if LERNEN and not gelernt:
                    qlern.lernen(q, s_q, a_q, -t_in)
                    qlern.speichern(q)
                    gelernt = True
                    qlern.zeige(q)
        else:
            t_in = None
            if ruhe:
                ruhe = False
                print("Ziel verlassen")
        if LERNEN and not gelernt and tt > 30.0:
            print("Fail (über 30 s)")
            qlern.lernen(q, s_q, a_q, qlern.R_FAIL)
            qlern.speichern(q)
            gelernt = True

        n_print += 1
        if n_print >= 3:
            n_print = 0
            print("%.0f, %.1f" % (x_last, u))


run()
