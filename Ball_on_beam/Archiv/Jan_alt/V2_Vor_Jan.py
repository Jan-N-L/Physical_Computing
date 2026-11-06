from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import ticks_us, ticks_diff

ANGLE_OFFSET = 0.5  # °  Schiene waagrecht (test_offset.py)

x_set = 200         # mm    Soll
TOL = 7.5           # mm    Sollbereich
V_TOL = 15          # mm/s  "ruhig", knapp über Rauschen

# P: fein nahe Ziel, grob ab E_BIG, dazwischen linear
KP_FINE = 0.10      # °/mm
KP_COARSE = 0.15    # °/mm
E_BIG = 50          # mm
ki = 0.02           # °/(mm*s)
kd = 0.08           # °/(mm/s)  zu gross -> schaukelt auf
I_MAX = 10          # °  Anti-Windup
FC = 2              # °  Kick gegen Haftreibung, zu gross -> pendelt

# Holt-Filter: 1 = aus, kleiner = glatter
ALPHA = 0.3         # Position
BETA = 0.35         # Geschwindigkeit

PLOT = 2            # 1 = Filter, 2 = Regler


def setAngle(angle):
    # ±30°, -90..90° -> Duty 1000..9000 (50 Hz)
    angle = max(-30, min(30, angle + ANGLE_OFFSET))
    servo.duty_u16(int((angle + 90)/180*8000 + 1000))


def plot_header():
    print("e_roh, e, de, hold" if PLOT == 1 else "P, I, D, u, hold")


def plot(e_raw, e, de, p, i, d, u, hold):
    if PLOT == 1:
        print("%.1f, %.1f, %.1f, %d" % (e_raw, e, de, 10*hold))
    else:
        print("%.1f, %.1f, %.1f, %.1f, %d" % (p, i, d, u, 10*hold))


tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
servo = PWM(Pin(16))
servo.freq(50)

e = x_set - tof.ping()  # gefiltert
de = 0.0                # mm/s, gefiltert
i = 0.0                 # °
hold = False
t_prev = ticks_us()
plot_header()

while True:
    e_raw = x_set - min(tof.ping(), 400)  # 8190 = nichts gesehen
    t = ticks_us()
    dt = ticks_diff(t, t_prev)/1e6
    t_prev = t

    # Holt: Glättung mit Trend -> hinkt nicht nach
    e_old = e
    e = ALPHA*e_raw + (1 - ALPHA)*(e + de*dt)
    de = BETA*(e - e_old)/dt + (1 - BETA)*de

    # ruhig im Sollbereich -> Servo hält (Hysterese)
    hold = abs(e) < TOL and abs(de) < (2*V_TOL if hold else V_TOL)

    if hold:
        p = d = 0.0
    else:
        p = (KP_FINE + (KP_COARSE - KP_FINE)*min(abs(e)/E_BIG, 1))*e
        if abs(e) > TOL:
            p += FC if e > 0 else -FC
        i = max(-I_MAX, min(I_MAX, i + ki*e*dt))
        d = kd*de

    u = p + i + d
    setAngle(u)
    plot(e_raw, e, de, p, i, d, u, hold)
