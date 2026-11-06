###############################################################################
# Ball on beam: Kennlinie VL53L0X -> echte mm
###############################################################################
#
# Rechnet den Sensorwert in die echte Ballposition um.
# Echte Position = Massstab, 0 = Ball am Anschlag beim Sensor.
# Gemessen am 06.11.2026 mit rauschtest.py, MIT schwarzem Karton am Klotz.
# Wenn der Aufbau ändert (Karton weg, Sensor verschoben): neu messen!
#
# Verwendung:
#   from kennlinie import to_mm, MITTE_MM, MITTE_SENSOR
#   x = to_mm(tof.ping())
###############################################################################

# (Sensorwert, echte mm), aufsteigend
TABELLE = (
    (36, 0),      # Anschlag Sensor
    (109, 100),
    (174, 150),
    (256, 200),
    (333, 250),
    (391, 300),
    (419, 339),   # Anschlag Klotz
)

MIN_MM = TABELLE[0][1]
MAX_MM = TABELLE[-1][1]
MITTE_MM = (MIN_MM + MAX_MM)/2          # 169.5 mm


def to_mm(s):
    """Sensorwert -> echte mm. Ausserhalb der Tabelle: linear verlängert."""
    t = TABELLE
    if s <= t[0][0]:
        i = 0
    elif s >= t[-1][0]:
        i = len(t) - 2
    else:
        i = 0
        while s > t[i + 1][0]:
            i += 1
    (s0, x0), (s1, x1) = t[i], t[i + 1]
    return x0 + (s - s0)*(x1 - x0)/(s1 - s0)


def to_sensor(x):
    """Echte mm -> Sensorwert (Umkehrung, z.B. für Sollwert in Sensorwerten)."""
    t = TABELLE
    if x <= t[0][1]:
        i = 0
    elif x >= t[-1][1]:
        i = len(t) - 2
    else:
        i = 0
        while x > t[i + 1][1]:
            i += 1
    (s0, x0), (s1, x1) = t[i], t[i + 1]
    return s0 + (x - x0)*(s1 - s0)/(x1 - x0)


MITTE_SENSOR = to_sensor(MITTE_MM)      # ca. 206
