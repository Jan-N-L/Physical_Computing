# Übergabe: Ball on Beam, Alternativen zu PID und Simulation

Stand 2026-11-06. Diese Datei ist für eine neue Claude-Sitzung ohne Vorwissen geschrieben und in sich vollständig: Simulationsmethode, gesamter Code, alle Ergebnisse und was dabei schiefging. Die Simulationsskripte lagen nur in einem temporären Ordner und existieren dauerhaft nur hier (Abschnitt 7).

**Inhalt:** 1 Ausgangslage · 2 Hardware und Konventionen · 3 Simulationsmethode · 4 Alle Ergebnisse · 5 Lernweg und Sackgassen · 6 Methoden im Detail · 7 Simulationscode

## 1. Ausgangslage

**Projekt:** Ball on Beam im Modul Physical Computing (ZHAW, HS26). Raspberry Pi Pico mit MicroPython, ToF-Sensor misst die Ballposition, ein Servo kippt die Schiene. Der PID-Regler in `Scripts/main.py` existiert und wurde von Hand abgestimmt.

**Nutzer:** Neuling in Regelungstechnik, sucht ausdrücklich kreative Lösungen ohne PID. Kommuniziert auf Deutsch, Dokumente in Schweizer Schreibweise (ss statt ß). Bei Hardware gilt: Kalibrierwerte lassen, keine Idealwerte annehmen.

**Was erledigt ist:**

- Sieben Alternativen beschrieben, je ein MD in `Ball_on_beam/`.
- Für sechs davon läuft der Code aus den MDs in einer PC-Simulation, die siebte (Gebogene Schiene) hat eine eigene Physik-Simulation.
- `Kreative-Ansaetze.md`: 14 weitere Ideen, ausdrücklich nur Theorie, nichts gerechnet ausser Überschlägen.

**Was nicht erledigt ist: Keine Methode lief je auf der echten Schiene.** Alle Zahlen stammen aus dem Modell. Behaupte nie, etwas sei an der Hardware getestet.

**Empfehlung bisher:** Zuerst $A$ messen, dann Bang-Bang mit einem Winkel von 3°: echter Methodenwechsel, fünf Zeilen, im Modell schnell und robust gegen Haftreibung. Als Zweites MPC, das den Servo klar schont und bei $A$ +33 % besser hält, aber bei Haftreibung schwächer ist. Fuzzy ist im Fall «Normal» am schnellsten, aber nur ein nichtlinearer PD und instabil bei $A$ +33 %.

**Offene Punkte, am besten in dieser Reihenfolge:**

1. $A$ an der echten Schiene messen (Skript im MPC-Abschnitt). Alle Modellzahlen hängen an der Schätzung $A$ = 60.
2. Echte Schleifenzeit und Rechenzeit messen (`ticks_us`). Im Modell angenommen: 30 ms Schleife, MPC 15 ms, Fuzzy 3 ms.
3. Rauschen von `de`, toten Bereich und Häufigkeit von Fehlmessungen an der Schiene bestimmen, Modell nachführen.
4. MPC an der Schiene testen und mit PID vergleichen.
5. Schienenlänge (400 mm angenommen), Kugeltyp und Verhältnis Servo zu Schiene sind unbekannt.

**Bekannte Unstimmigkeit:** `Scripts/test_offset.py` gibt den Wert als `ANGLE_OFFSET` aus, `main.py` heisst seit dem letzten Stand `OFFSET`. Die MDs verwenden `OFFSET`. Nicht ungefragt ändern.

**Dateien in `Ball_on_beam/`:**

| Datei | Inhalt |
| ----- | ------ |
| `Vorhersage-Regelung-MPC.md`, `Bang-Bang-Regelung.md`, `Fuzzy-Regelung.md`, `Golfer-Impuls-Regelung.md`, `Imitations-Regelung.md`, `Q-Learning-Regelung.md`, `Gebogene-Schiene.md` | Methoden-Dokumente für den Nutzer, Inhalt auch in Abschnitt 6 |
| `Kreative-Ansaetze.md` | Ungetestete Ideen: Rastkerbe, Rüttelmotor, absichtliches Pendeln, Ball als Poti-Schleifer, Piezo-Mikrofon, Störgrössen-Beobachter (ADRC), Wirbelstrombremse, Schienen-Karte, gelerntes Physik-Modell, SPSA-Selbstoptimierung, Regler als Messgerät, Motorik-Trainer, Ball-Telefon, Wellen-Schiene |
| `PID-Regelung.md` | Kursunterlage |
| `Scripts/main.py` | Aktueller PID-Regler (Abschnitt 2) |
| `Scripts/test_offset.py` | Findet `OFFSET` und toten Bereich an der Schiene |
| `Scripts/test_offset_sim.py`, `Scripts/test_main_sim.py` | Bestehende PC-Tests dazu |
| `Scripts/vl53l0x.py` | Sensor-Treiber |
| `pico/sw05` bis `pico/sw08` | Kursmaterial |

## 2. Hardware und Konventionen

| Teil | Wert |
| ---- | ---- |
| Controller | Raspberry Pi Pico, MicroPython |
| Sensor | VL53L0X, I²C 0, SCL GP9, SDA GP8, liefert 8190, wenn er nichts sieht |
| Servo | PWM GP16, 50 Hz, Duty 1000 bis 9000 für −90° bis +90°, Software-Grenze ±30° |
| `OFFSET` | 0.5°, von `test_offset.py` |
| Sollwert | `x_set` = 200 mm, Sollbereich `TOL` = 7.5 mm |

**Vorzeichen, unbedingt einhalten:**

- `e = x_set - Distanz`, also `e > 0` heisst: Ball zu nah am Sensor.
- `de` ist die Änderungsrate von `e`, also Tempo weg vom Sensor gleich `de < 0`.
- Positiver Winkel rollt den Ball vom Sensor weg, also $\ddot e = -A \cdot u$.
- Stabilisierung braucht darum $u \approx +k_1 e + k_2 \dot e$ mit positiven Faktoren.

**Einheitliches Interface:** Jede Methode ist `regler(e, de, dt)` und gibt einen Winkel in Grad zurück. `measure()`, `setAngle()` und die Initialisierung bleiben aus `main.py`, die Regelschleife am Ende wird ersetzt durch:

```python
while True:
    e_raw, dt = measure()
    u = regler(e, de, dt)
    setAngle(u)
    print("%.1f, %.1f" % (e, u))
```

**Aktuelle `Scripts/main.py`:**

```python
from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from utime import ticks_us, ticks_ms, ticks_diff

OFFSET = 0.5  # °  Schiene waagrecht (test_offset.py)

x_set = 200         # mm    Soll
X_MAX = 400         # mm    ab hier kein gültiger Messwert (8190 = nichts gesehen)
TOL = 7.5           # mm    Sollbereich
V_TOL = 15          # mm/s  "ruhig", knapp über Rauschen
T_STEADY = 3000     # ms    so lange hold -> Steady (Schiene waagrecht)

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
    angle = max(-30, min(30, angle + OFFSET))
    servo.duty_u16(int((angle + 90)/180*8000 + 1000))


def plot_header():
    print("e_roh, e, de, hold" if PLOT == 1 else "P, I, D, u, hold")


def plot(e_raw, e, de, p, i, d, u, hold):
    if PLOT == 1:
        print("%.1f, %.1f, %.1f, %d" % (e_raw, e, de, 10*hold))
    else:
        print("%.1f, %.1f, %.1f, %.1f, %d" % (p, i, d, u, 10*hold))


def measure():
    # misst, filtert e/de (global), gibt e_roh und dt zurück
    global e, de, t_prev
    x = tof.ping()
    e_raw = x_set - min(x, X_MAX)  # nur für Plot begrenzt
    t = ticks_us()
    dt = ticks_diff(t, t_prev)/1e6
    t_prev = t

    # Holt: Glättung mit Trend -> hinkt nicht nach
    # Fehlmessung -> e statt e_raw: e bleibt beschränkt, Trend klingt ab
    e_old = e
    e = ALPHA*(e_raw if x < X_MAX else e) + (1 - ALPHA)*(e + de*dt)
    de = BETA*(e - e_old)/dt + (1 - BETA)*de
    return e_raw, dt


tof = VL53L0X(I2C(id=0, scl=Pin(9), sda=Pin(8)))
servo = PWM(Pin(16))
servo.freq(50)

e = x_set - min(tof.ping(), X_MAX)  # gefiltert
de = 0.0                # mm/s, gefiltert
i = 0.0                 # °
hold = False
t_prev = ticks_us()
t_hold = ticks_ms()
plot_header()

while True:
    # Regel-Schlaufe: bis Ball T_STEADY lang im hold
    while True:
        e_raw, dt = measure()

        # ruhig im Sollbereich -> Servo hält (Hysterese)
        hold = abs(e) < TOL and abs(de) < (2*V_TOL if hold else V_TOL)
        if not hold:
            t_hold = ticks_ms()
        elif ticks_diff(ticks_ms(), t_hold) > T_STEADY:
            break

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

    # Steady-Schlaufe: Schiene waagrecht, nichts tun bis Ball aus 2*TOL
    setAngle(0)
    while abs(e) < 3*TOL:
        e_raw, dt = measure()
```

## 3. Simulationsmethode

**Zweck:** Vorzeichen, Logik und grobe Einordnung prüfen. Kein Ersatz für die Hardware. **Prinzip:** `check_docs.py` zieht den Code per Regex direkt aus den MDs (erster ` ```python `-Block mit `def regler`) und führt ihn gegen das Modell in `sim_methods.py` aus. Damit sind Doku und getesteter Code identisch. Ausserdem prüft es, ob die Tabellen in den MDs noch stimmen.

**Modell:** Zustand Position $p$, Tempo $v$, Servowinkel $\theta$. Euler-Integration mit 1 ms.

- **Servo:** Verzögerung erster Ordnung $\tau$ = 40 ms mit Ratenbegrenzung 400 °/s.
- **Schieflage:** wirksamer Winkel $\varphi = \theta - \text{level}$.
- **Ball ruht** ($v = 0$): bleibt liegen, solange $|\varphi| \le$ `dead`/2, sonst $a = A(\varphi - \text{sign}(\varphi) \cdot \text{roll})$.
- **Ball rollt:** $a = A(\varphi - \text{sign}(v) \cdot \text{roll})$. Wechselt $v$ das Vorzeichen bei $|\varphi| <$ `dead`/2, stoppt er.
- **Enden:** $0 \le p \le 400$ mm, am Anschlag $v = 0$, kein Abprall.
- **Sensor:** Distanz $= p + 30 + \mathcal{N}(0, \sigma)$, optional Fehlmessung 8190 mit Wahrscheinlichkeit `outlier`.
- **Schleife:** 15 ms rollen, messen, 15 ms rollen. Das ergibt 30 ms pro Durchgang wie in `test_offset_sim.py` angenommen und bildet die Messverzögerung ab.
- **Filter:** Klasse `Holt`, identisch zu `measure()` in `main.py`, $\alpha$ = 0.3, $\beta$ = 0.35, Fehlmessung lässt `e` auf dem Trend weiterlaufen.
- **Rechenzeit:** `latency` verzögert das Setzen des Winkels. MPC 15 ms, Fuzzy 3 ms, Rest 0. Alles Annahmen.
- **PID:** `PIDMain` ist eine Handkopie von `main.py` samt Hold und Steady. **Bei Änderungen an `main.py` nachziehen, zusammen mit `Holt`.**

**Fälle:** Der Regler rechnet immer mit $A$ = 60.

| Fall | real $A$ | `dead` | `roll` | `level` | $\sigma$ |
| ---- | -------- | ------ | ------ | ------- | -------- |
| Normal | 60 | 1.5° | 0.2° | 0° | 2 mm |
| A +33 % | 80 | 1.5° | 0.2° | 0° | 2 mm |
| A −25 % | 45 | 1.5° | 0.2° | 0° | 2 mm |
| Schwer | 45 | 3° | 0.3° | 0.5° | 3 mm |

**Warum $A$ = 60?** Theorie für rollende Vollkugel: $\tfrac{5}{7} g \sin 1° \approx 122$ mm/s² pro Grad **Schiene**. Pro Grad **Servo** kann es wegen des Hebels weniger sein. Indiz: Der PID aus `main.py` erreicht im Modell bei $A$ = 30 und 60 das Ziel in 16/20 Läufen, bei 90 und 122 in 0/20 mit hartem Schwingen. Weil er an der Schiene offenbar läuft, liegt das echte $A$ wohl deutlich unter 122. Das ist ein Rückschluss, keine Messung.

**Protokoll:** 20 Läufe, Startpositionen $p_0$ = 40, 100, 300, 360 mm, je Seeds 0 bis 4. Ball ruht am Start. Dauer 20 s, bei $t$ = 12 s Stoss $v$ += 120 mm/s.

**Kennzahlen, jeweils gegen die wahre Position:**

| Kennzahl | Definition |
| -------- | ---------- |
| Am Ziel | Nach 12 s innerhalb $\pm$7.5 mm und seit mindestens 1 s nicht mehr draussen |
| Zeit | Median über erfolgreiche Läufe: letzter Zeitpunkt ausserhalb des Bereichs |
| Nach Stoss wieder am Ziel | Dasselbe am Ende des Laufs, Zeit ab dem Stoss |
| Restfehler | Median über Läufe der mittleren $|e|$ in den letzten 100 Schleifen (3 s) vor dem Stoss |
| Servo-Weg | Median über Läufe von $\sum |\Delta u|$ pro Sekunde |
| Anschläge pro Lauf | Mittel, Ankünfte am Schienenende, Andrücken zählt nicht |

**Grenzen des Modells:** Kein Servo-Getriebespiel, kein Abprall, Rauschen ideal normalverteilt, Schiene überall gleich, Rechenzeiten geschätzt, Servo-zu-Schiene als Konstante. Die Methoden wurden auf das Modell abgestimmt, der PID auf die echte Schiene. **Der PID-Vergleich ist darum unfair und nur eine grobe Einordnung.**

**Weitere Einzelmodelle:**

- **Imitation:** `Human` ist kein Mensch, sondern $0.06 e + 0.06 \dot e$ mit 150 ms Verzögerung, träger Hand und Zittern von 0.3°. Lernt 240 s, Neustart alle 20 s.
- **Gebogene Schiene:** `curved()`, Physik auf Kreisbogen mit $A$ = 122 pro Grad Schiene, `roll` 0.1°, `dead` 1.5°, Kippen um 3°, 25 s.

## 4. Alle Ergebnisse

Reproduzierbar mit `python check_docs.py`, das bricht ab, sobald eine MD-Tabelle veraltet ist. Python 3.14, nur Standardbibliothek.

| Methode | Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ------- | ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| PID `main.py` | Normal | 16/20 | 6.6 s | 19/20 | 3.0 s | 4.0 mm | 27 °/s | 0.0 |
| PID `main.py` | A +33 % | 0/20 | – | 0/20 | – | 24.5 mm | 150 °/s | 0.0 |
| PID `main.py` | A −25 % | 13/20 | 8.2 s | 19/20 | 0.9 s | 4.3 mm | 17 °/s | 0.0 |
| PID `main.py` | Schwer | 14/20 | 4.7 s | 16/20 | 0.8 s | 4.2 mm | 26 °/s | 0.0 |
| MPC | Normal | 20/20 | 2.1 s | 20/20 | 1.0 s | 1.2 mm | 5 °/s | 0.0 |
| MPC | A +33 % | 18/20 | 2.8 s | 16/20 | 4.0 s | 2.7 mm | 14 °/s | 0.0 |
| MPC | A −25 % | 20/20 | 1.9 s | 20/20 | 1.8 s | 1.9 mm | 4 °/s | 0.0 |
| MPC | Schwer | 8/20 | 3.6 s | 18/20 | 1.1 s | 7.4 mm | 9 °/s | 0.0 |
| Bang-Bang | Normal | 20/20 | 1.9 s | 20/20 | 1.7 s | 1.4 mm | 21 °/s | 0.0 |
| Bang-Bang | A +33 % | 12/20 | 4.0 s | 19/20 | 1.6 s | 2.3 mm | 21 °/s | 0.0 |
| Bang-Bang | A −25 % | 20/20 | 2.0 s | 20/20 | 2.0 s | 1.5 mm | 18 °/s | 0.0 |
| Bang-Bang | Schwer | 19/20 | 7.1 s | 19/20 | 2.7 s | 3.6 mm | 26 °/s | 0.0 |
| Fuzzy | Normal | 20/20 | 1.6 s | 20/20 | 1.3 s | 1.0 mm | 21 °/s | 0.0 |
| Fuzzy | A +33 % | 0/20 | – | 0/20 | – | 13.8 mm | 67 °/s | 0.0 |
| Fuzzy | A −25 % | 20/20 | 1.3 s | 20/20 | 0.8 s | 0.6 mm | 19 °/s | 0.0 |
| Fuzzy | Schwer | 20/20 | 1.5 s | 20/20 | 0.7 s | 3.3 mm | 27 °/s | 0.0 |
| Golfer | Normal | 13/20 | 3.1 s | 9/20 | 5.2 s | 2.9 mm | 13 °/s | 0.1 |
| Golfer | A +33 % | 9/20 | 3.7 s | 7/20 | 3.2 s | 8.5 mm | 16 °/s | 0.7 |
| Golfer | A −25 % | 15/20 | 8.0 s | 12/20 | 4.9 s | 3.2 mm | 15 °/s | 0.0 |
| Golfer | Schwer | 0/20 | – | 3/20 | 4.4 s | 63.3 mm | 14 °/s | 0.1 |
| Synth. Mensch (Lehrer) | Normal | 8/20 | 7.3 s | 0/20 | – | 7.0 mm | 20 °/s | 0.0 |
| Imitation | Normal | 18/20 | 5.2 s | 15/20 | 4.9 s | 3.2 mm | 11 °/s | 0.0 |
| Imitation | A +33 % | 20/20 | 3.6 s | 19/20 | 3.4 s | 3.6 mm | 10 °/s | 0.0 |
| Imitation | A −25 % | 13/20 | 6.9 s | 6/20 | 4.5 s | 4.2 mm | 12 °/s | 0.0 |
| Imitation | Schwer | 19/20 | 6.6 s | 18/20 | 4.1 s | 3.0 mm | 13 °/s | 0.0 |
| Q-Learning | Normal | 15/20 | 4.8 s | 14/20 | 5.4 s | 1.9 mm | 24 °/s | 0.0 |
| Q-Learning | A +33 % | 5/20 | 9.1 s | 5/20 | 6.1 s | 11.1 mm | 46 °/s | 0.0 |
| Q-Learning | A −25 % | 18/20 | 7.6 s | 17/20 | 4.2 s | 2.2 mm | 19 °/s | 0.0 |
| Q-Learning | Schwer | 7/20 | 8.8 s | 2/20 | 3.8 s | 5.4 mm | 28 °/s | 0.0 |

**Einordnung:**

- **MPC** hält bei $A$ +33 % am besten unter den Reglern mit fest eingestelltem Modell und schont den Servo (5 °/s). Schwäche: Haftreibung im Fall «Schwer», der Ball klebt nahe am Rand (7.4 mm).
- **Fuzzy** ist bei stimmendem $A$ am schnellsten, aber bei $A$ +33 % dauernd instabil (Servo-Weg 67 °/s).
- **Bang-Bang** mit 3° ist schnell, robust gegen Reibung und braucht fünf Zeilen. Schwächen: viel Servo-Weg (rund viermal MPC) und nur 12/20 bei $A$ +33 %.
- **Golfer** bricht im Fall «Schwer» ein: Die Schieflage von 0.5° übersteigt die Rollreibung von 0.3°, der Ball rollt nach dem Schlag davon.
- **Imitation** ist oberflächlich gut, hängt aber am synthetischen Lehrer und sagt wenig über echte Menschen.
- **Q-Learning** schwankt extrem mit dem Zufall (unten).

**Robustheit, Anzahl Läufe «Am Ziel» von 20.** Regler rechnen immer mit $A$ = 60. «Tot 4°» ist «Schwer» mit `dead` = 4°, «$A$ = 122» die theoretische Vollkugel:

| Regler | Normal | A +33 % | A −25 % | Schwer | Tot 4° | $A$ = 122 |
| ------ | ------ | ------- | ------- | ------ | ------ | --------- |
| Bang-Bang 2° | 20 | 20 | 20 | 0 | 0 | 20 |
| Bang-Bang 2.5° | 20 | 20 | 20 | 18 | 0 | 11 |
| **Bang-Bang 3°** | **20** | **12** | **20** | **19** | **20** | **2** |
| Bang-Bang 4° | 16 | 8 | 20 | 18 | 19 | 0 |
| Bang-Bang 10° | 0 | 0 | 0 | 0 | 0 | 0 |
| Bang-Bang 10° fern, 4° nah | 16 | 3 | 20 | 20 | 18 | 0 |
| MPC | 20 | 18 | 20 | 8 | 12 | 0 |
| Fuzzy | 20 | 0 | 20 | 20 | 20 | 0 |
| PID `main.py` | 16 | 0 | 13 | 14 | 13 | 0 |

Bei $A$ = 122 scheitern fast alle: Darum zuerst $A$ messen. Für Bang-Bang zählt das Produkt $A \cdot U$, für MPC und Fuzzy das eingetragene $A$ beziehungsweise die Tabellenwerte.

**Einzelmessungen:**

| Frage | Ergebnis |
| ----- | -------- |
| PID, «Am Ziel» bei real $A$ = 30, 60, 90, 122 | 16, 16, 0, 0 von 20 |
| PID, 0.5 % Fehlmessungen im Fall Normal, mit heutigem `measure()` | 14/20, Restfehler 4.2 mm |
| MPC, Fall Schwer, `U_KICK` = 3, 4, 5 | 8, 11, 10 von 20 |
| Messskript für $A$, im Modell wahr 60 | ergibt 50.4, also Faktor 1.2 nötig |
| Rauschen von `de` bei ruhendem Ball, $\sigma$ = 2 mm | Streuung 10.5 mm/s, Spitze 38 mm/s; bei 3 mm 15.7 und 57 |
| Q-Learning, fünf Trainings (Seeds 1, 2, 3, 7, 11), Fall Normal | 15, 4, 5, 4, 17 von 20, Median 5 |
| Skript-Test dieser fünf Trainings, Ball nach 12 s am Ziel | 45, 28, 33, 24, 40 von 50, wählt Seed 1 |
| Q-Learning mit `BONUS` = 3, `PEN` = 0.05, dieselben Seeds | 11, 0, 8, 10, 4 von 20, Median 8 |
| Imitation, leere Felder nach 4 min Training | 10 von 63 |
| Synthetischer Mensch, $A$ −25 % | 20/20, die Tabelle nur 13/20 |

**Gebogene Schiene,** Kippen um 3°, Position nach 25 s, «im Bereich ab» heisst letzter Zeitpunkt ausserhalb $\pm$7.5 mm um das ideale Ziel, 25 s also nie:

| $R$ | Periode | Ziel | Ein Schritt: Ende, im Bereich ab | Zwei Stufen: Ende, im Bereich ab | $T$ +10 % | $T$ +20 % | $T$ −20 % |
| --- | ------- | ---- | -------------------------------- | -------------------------------- | --------- | --------- | --------- |
| 500 mm | 1.68 s | 26.2 mm | 20.9 mm, 8.6 s | 24.4 mm, 0.6 s | 22.5 mm, 0.6 s | 31.3 mm, 0.6 s | 21.0 mm, 0.6 s |
| 1000 mm | 2.38 s | 52.4 mm | 41.9 mm, nie | 48.9 mm, 1.0 s | 45.0 mm, 1.0 s | 62.5 mm, nie | 41.9 mm, nie |
| 2000 mm | 3.36 s | 104.7 mm | 83.8 mm, nie | 97.7 mm, 1.6 s | 90.2 mm, nie | 125.1 mm, nie | 83.8 mm, nie |

## 5. Lernweg und Sackgassen

Damit die nächste Sitzung diese Fehler nicht wiederholt.

**Modell:**

- Erster Lauf mit $A$ = 122 und 0.5 % Ausreissern: PID 0/20. Grund war das Modell, nicht der PID. Darum $A$ = 60 und Ausreisser standardmässig aus.
- Eine Spur mit fester Schrittweite täuschte: Bei jedem 25. Wert sah ein schnelles Schwingen wie eine langsame Bahn mit falschem Vorzeichen aus (Aliasing). Spuren nie mit fester Schrittweite ausdünnen, wenn Schwingen möglich ist.
- Der Anschlag-Zähler zählte zuerst jede Millisekunde Andrücken (13.6 pro Lauf). Jetzt nur noch Ankünfte.

**`main.py` änderte sich während der Arbeit (Nutzer oder parallele Sitzung, 2026-11-06 00:08):** Fehlmessungen werden seither abgefangen (Zeile mit `e_raw if x < X_MAX else e`), `ANGLE_OFFSET` heisst `OFFSET`, die Steady-Schwelle ist `3*TOL`. Eine Warnung, der Fehlwert 8190 gehe als echte 400 mm durch, war dadurch veraltet und wurde zurückgenommen. Vor Aussagen zu `main.py` also frisch lesen. Der damalige Vergleich: ohne Abfangen 5/20, mit 14/20.

**MPC:** Ohne Verzögerungsausgleich und mit `W_V` = 0.05 nur 9/20. Rastersuche über `T_LEAD`, `W_V`, `W_U` ergab 0.08, 0.2, 2. Kick sofort: Schwer 18/20, aber $A$ +33 % nur 2/20 durch Pendeln. Kick erst nach `T_STUCK` = 0.4 s: Schwer 8/20, $A$ +33 % 18/20. **Bewusster Kompromiss.** Ein Modell-Haftreibungszweig `U_STICK` hatte mit den Winkelkandidaten keine Wirkung und wurde entfernt.

**Golfer:** `V_REST` = 8 lag unter dem Rauschen von `de` (Streuung rund 10 mm/s), der Schlag startete nie. Jetzt 20. Der Bremsstoss ohne `T_QUIET` reagierte auf den Filter-Nachlauf und verschlechterte auf 3/20. Mit `T_QUIET` = 0.3 s und `V_BRAKE` = 60 sind es 13/20. Ohne Bremse rollt der Ball nach dem Stoss ans Ende.

**Imitation:** Der erste synthetische Mensch ($0.15 e + 0.10 \dot e$, 200 ms) war selbst instabil, das sagte nichts aus. Jetzt langsamer und stabil.

**Q-Learning:**

- Entscheid in jedem Schritt (33 Hz) führte zu Zittern bei 106 °/s. Jetzt neue Aktion nur jeden dritten Durchgang.
- Gespiegeltes Lernen und kleine Aktionen ±2° statt ±3° brachten den Restfehler merklich herunter.
- **Ein einzelner Trainingslauf sagt nichts aus.** Eine zuvor notierte Warnung, `BONUS` = 3 lasse den Ball 50 mm daneben, war nur ein Zufallsergebnis und wurde gestrichen. Darum trainiert das Skript fünfmal und nimmt die beste Tabelle.
- Die Aussage, man müsse den Ball von Hand zurücklegen, war falsch: Die Schiene rollt ihn selbst zurück. Das eigentliche Argument sind 20 Stunden Servobetrieb pro Training.

**Fuzzy:** Direkt im ersten sinnvollen Anlauf abgestimmt, nahe am Ziel entspricht die Tabelle $K_p \approx 0.15$, $K_d \approx 0.08$.

**Bang-Bang:** Die erste Version mit 10° fern und 4° nah erreichte 16/20 in 7.5 s, und die Doku behauptete, ein einziger Winkel überschiesse immer. **Das war ungeprüft und falsch.** Beim Nachmessen schaffte ein einziger Winkel von 3° 20/20 in 1.9 s. Wahrscheinlich kostete der grosse Winkel in der Ferne mehr, als er brachte. Lehre: Jede Begründung im Text messen, nicht nur die Tabelle.

**Allgemein:** Winkel, Gewichte und Fuzzy-Tabelle wurden anhand genau dieser Fälle gewählt. Die Zahlen sind also etwas zu optimistisch, und auch der Vergleich der Methoden hängt davon ab, wie viel Mühe jede bekam.

**Bewusst weggelassen:** LQR und Zustandsregler sind im Kern PD mit berechneten Faktoren, also kein echter Wechsel.

## 6. Methoden im Detail

Identisch mit den einzelnen MDs, ohne die dort wiederholten Abschnitte zu Testprotokoll, Modell und Einbau (siehe Abschnitte 2 und 3).

### 6.1 Vorhersage-Regelung (MPC)

Der Pico spielt vor jedem Schritt mehrere Neigungswinkel im Kopf durch und wählt den Winkel, der den Ball laut Physik am besten ans Ziel bringt. MPC steht für *Model Predictive Control*. Hier steht eine stark vereinfachte Version.

#### Die Idee

Beim Autofahren fragst du nicht nur, wie weit du gerade von der Spurmitte entfernt bist. Du schätzt, wo du in zwei Sekunden bist, wenn du so weiterlenkst, und korrigierst schon jetzt. Genau das macht dieser Regler: Er schaut nicht auf den Fehler, sondern auf die vorhergesagte Zukunft.

#### So funktioniert es

Alle 30 ms läuft derselbe Ablauf:

1. **Messen:** Position `e` und Geschwindigkeit `de` kommen aus `measure()`.
2. **Vorhersagen:** Für jeden Kandidaten-Winkel $u$ berechnet der Pico die Ballbahn der nächsten 0.5 s:

$$
\hat e(t) = e + \dot e \cdot t - \tfrac{1}{2} A \, u \, t^2
$$

3. **Bewerten:** Jede Bahn bekommt Kosten. Kleine Kosten bedeuten: nah am Ziel, am Schluss langsam, kleiner Winkel.

$$
J(u) = \sum_t \hat e(t)^2 + W_V \cdot \hat v_{\text{Ende}}^2 + W_U \cdot u^2
$$

4. **Wählen:** Der Winkel mit den kleinsten Kosten wird eingestellt. Im nächsten Durchgang beginnt alles von vorne.

**Das Modell ist nur eine Zahl.** $A$ ist die Beschleunigung des Balls pro Grad Neigung. Für eine rollende Vollkugel gilt theoretisch $\tfrac{5}{7} \cdot g \cdot \sin(1°) \approx 122$ mm/s² pro Grad **Schiene**. Pro Grad **Servo** kann der Wert wegen des Hebels kleiner sein, darum wird er gemessen.

**Beispiel** mit `e` = 30 mm und ruhendem Ball, $A$ = 60:

| Winkel $u$ | Summe $\hat e^2$ | $W_V \cdot \hat v^2$ | $W_U \cdot u^2$ | Kosten $J$ |
| ---------- | ---------------- | -------------------- | --------------- | ---------- |
| 0°         | 4500             | 0                    | 0               | 4500       |
| **2°**     | **2872**         | **720**              | **8**           | **3600**   |
| 4°         | 1950             | 2880                 | 32              | 4862       |
| 8°         | 2219             | 11520                | 128             | 13867      |

2° gewinnt. 8° brächte den Ball zwar schnell hin, aber mit zu viel Tempo.

#### Code für deine Schiene

Vorzeichen wie in `main.py`: `e = x_set - Distanz`, und ein positiver Winkel rollt den Ball vom Sensor weg, also wird `e` kleiner.

```python
A = 60              # mm/s² pro Grad Servo: messen, siehe unten
ANGLES = (-16, -8, -4, -2, 0, 2, 4, 8, 16)  # ° Kandidaten
TIMES = (0.1, 0.2, 0.3, 0.4, 0.5)           # s Vorhersage
T_LEAD = 0.08       # s   Verzögerung von Sensor, Filter und Servo
W_V = 0.2           # Gewicht Restgeschwindigkeit, grösser = vorsichtiger
W_U = 2             # Gewicht grosse Winkel
V_STOP = 15         # mm/s  darunter gilt der Ball als ruhend
TOL = 6             # mm  so nah am Ziel braucht es keinen Kick
T_STUCK = 0.4       # s   so lange warten, bevor ein Kick kommt
U_KICK = 3          # °   Kick gegen Haftreibung
t_stuck = 0.0

def regler(e, de, dt):
    global t_stuck
    e = e + de*T_LEAD               # dort ist der Ball, wenn der Winkel wirkt
    best_u, best_cost = 0, 1e18
    for u in ANGLES:
        a = -A*u
        cost = W_U*u*u
        for t in TIMES:
            e_t = e + de*t + 0.5*a*t*t
            cost += e_t*e_t
        v_end = de + a*TIMES[-1]
        cost += W_V*v_end*v_end
        if cost < best_cost:
            best_u, best_cost = u, cost

    # Das Modell kennt keine Haftreibung: klebt der Ball, hilft ein Kick
    if best_u == 0 and abs(e) > TOL and abs(de) < V_STOP:
        t_stuck += dt
        if t_stuck > T_STUCK:
            best_u = U_KICK if e > 0 else -U_KICK
    else:
        t_stuck = 0.0
    return best_u
```

#### A messen

Schreibe das nach der Initialisierung in ein eigenes Skript, ohne Regelschleife. Der Ball rollt mit 5° Neigung 200 mm weit, aus der Zeit folgt $A = 2s / (t^2 \cdot u)$.

```python
from utime import sleep_ms
U, S = 5, 200       # ° Neigung, mm Strecke

setAngle(-15)
sleep_ms(3000)      # Ball rollt zum Sensor
d0 = tof.ping()
setAngle(U)
t0 = ticks_ms()
while not d0 + S < tof.ping() < 1000:
    pass
t = ticks_diff(ticks_ms(), t0)/1000
print("A = %.0f" % (2*S/(t*t*U)))
```

Das Resultat liegt wegen Rollreibung und Servo-Verzögerung zu tief: Im Modell ergab die Messung 50 statt der wahren 60. Rechne also mit dem Messwert mal 1.2. Ein etwas zu grosses $A$ schadet in der Simulation weniger als ein zu kleines (siehe unten).

#### Einstellen

| Beobachtung                       | Massnahme                               |
| --------------------------------- | --------------------------------------- |
| Ball schwingt um das Ziel         | `W_V` erhöhen, `A` erhöhen              |
| Ball kommt nur langsam ans Ziel   | `W_V` senken                            |
| Winkel springt nervös hin und her | `W_U` erhöhen, grössere Winkel streichen |
| Ball klebt knapp neben dem Ziel   | `U_KICK` erhöhen                        |
| Ball wird vom Kick weggestossen   | `U_KICK` senken, `T_STUCK` erhöhen      |

Miss auch die Rechenzeit: `ticks_us()` vor und nach `regler()`. Zählt sie mehr als rund 15 ms, nimm weniger Winkel oder weniger Zeitpunkte. Als `U_KICK` passt ungefähr der halbe «tote Bereich» aus `test_offset.py` plus 1°.

#### Stärken und Schwächen

| Stärken                                          | Schwächen                                          |
| ------------------------------------------------ | -------------------------------------------------- |
| Faktoren folgen aus Physik statt aus Ausprobieren | Braucht ein gemessenes $A$                         |
| Begrenzte Winkel sind von Anfang an eingebaut    | Rechenaufwand auf dem Pico                         |
| Ruhiger Servo                                    | Modell kennt keine Reibung, darum der Kick         |
| Leicht erweiterbar: Hindernis, Zielbahn          | Kein I-Anteil: `OFFSET` muss stimmen         |

#### Was die Simulation zeigt

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| **Normal** | 20/20 | 2.1 s | 20/20 | 1.0 s | 1.2 mm | 5 °/s | 0.0 |
| **A +33 %** | 18/20 | 2.8 s | 16/20 | 4.0 s | 2.7 mm | 14 °/s | 0.0 |
| **A −25 %** | 20/20 | 1.9 s | 20/20 | 1.8 s | 1.9 mm | 4 °/s | 0.0 |
| **Schwer** | 8/20 | 3.6 s | 18/20 | 1.1 s | 7.4 mm | 9 °/s | 0.0 |

Im Fall «Schwer» bleibt der Ball oft knapp am Rand des Sollbereichs (Restfehler 7.4 mm). `U_KICK = 4` hob den Wert im Modell nur auf 11/20.

Das Modell schätzt nur grob ab, ob Vorzeichen und Logik stimmen. **Ein Test an der echten Schiene steht noch aus.** Rechenzeit ist mit 15 ms angenommen, nicht gemessen.

#### Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. $A$ messen und eintragen.
3. Nur `print("%.1f, %.1f" % (e, regler(e, de, dt)))` laufen lassen, Servo noch aus: Ball von Hand verschieben, Vorzeichen prüfen. Bei `e > 0` muss der Winkel positiv sein.
4. Servo aktivieren, Ball in Zielnähe legen, danach weiter weg.

#### Verwandtschaft mit PID

Es gibt keine Kp-, Ki- und Kd-Werte und kein Fehlerintegral. Ganz ohne Verwandtschaft ist die Methode aber nicht: Mit unbegrenzten Winkeln ergäbe die Optimierung rechnerisch $u = k_1 e + k_2 \dot e$, also eine PD-Form. Der echte Unterschied liegt im Entwurf und bei grossen Fehlern: Die Faktoren entstehen aus Physik und Gewichten, und die Kandidatenliste begrenzt das Verhalten ausdrücklich.

### 6.2 Bang-Bang-Regelung mit Schaltkurve

Die Schiene kennt nur drei Zustände: kippen, gegenkippen, waagrecht. Eine einfache Formel aus der Physik entscheidet, wann gebremst wird.

#### Die Idee

Du fährst auf ein Rotlicht zu. Am schnellsten bist du, wenn du Vollgas gibst und im letzten möglichen Moment voll bremst. Dafür musst du nur eines wissen: Wie lang ist mein Bremsweg? Solange die Strecke zum Ziel länger ist als der Bremsweg, gibst du Gas. Sonst bremst du.

#### So funktioniert es

Mit der Bremsbeschleunigung $a$ ist der Bremsweg bei Tempo $v$:

$$
s_{\text{Brems}} = \frac{v^2}{2a}
$$

Daraus entsteht eine Entscheidungsgrösse $s$, die **Schaltkurve**:

$$
s = e + \dot e \cdot T_{\text{Lead}} + \frac{\dot e \, |\dot e|}{2a}
$$

- $s > 0$: Es bleibt genug Weg. Winkel $+U$.
- $s < 0$: Der Ball ist zu schnell. Winkel $-U$.
- Ball nah am Ziel und langsam: Winkel 0.

Der Term mit $T_{\text{Lead}}$ gleicht aus, dass Sensor, Filter und Servo verzögert reagieren.

**Beispiel** mit $a$ = 126 mm/s² ($A$ = 60, $U$ = 3°, Sicherheit 0.7), `e` = 20 mm, $T_{\text{Lead}}$ = 0.1 s:

| Tempo $\dot e$ | Bremsweg | $s$                 | Entscheid       |
| -------------- | -------- | ------------------- | --------------- |
| −30 mm/s       | 3.6 mm   | 20 − 3 − 3.6 = +13  | Weiter Richtung Ziel |
| −80 mm/s       | 25 mm    | 20 − 8 − 25 = −13   | Bremsen         |

#### Code für deine Schiene

Vorzeichen wie in `main.py`: `e = x_set - Distanz`, und ein positiver Winkel rollt den Ball vom Sensor weg.

```python
A = 60                  # mm/s² pro Grad Servo, siehe Vorhersage-Regelung-MPC.md
U = 3                   # °   Kippwinkel, der wichtigste Wert
SAFETY = 0.7            # rechnet mit schwächerer Bremse, bremst also früher
T_LEAD = 0.10           # s   Verzögerung von Sensor, Filter und Servo
TOL, V_TOL = 6, 15      # mm, mm/s  Ziel erreicht

def regler(e, de, dt):
    if abs(e) < TOL and abs(de) < V_TOL:
        return 0
    a = A*U*SAFETY
    s = e + de*T_LEAD + de*abs(de)/(2*a)
    return U if s > 0 else -U
```

Ein einziger, kleiner Winkel genügt. Das ist überraschend: Eine erste Version mit 10° in der Ferne und 4° in Zielnähe brauchte im Modell 7.5 s, diese hier nur 1.9 s. Grosse Winkel überschiessen, und der Ball pendelt dann lange um das Ziel.

#### Einstellen

| Beobachtung                    | Massnahme                                 |
| ------------------------------ | ----------------------------------------- |
| Ball pendelt um das Ziel       | `U` senken, danach `SAFETY` senken oder `T_LEAD` erhöhen |
| Ball kommt nur langsam ans Ziel | `U` in kleinen Schritten erhöhen         |
| Ball bleibt liegen, obwohl die Schiene kippt | `U` erhöhen: muss über halbem totem Bereich aus `test_offset.py` plus Schieflage liegen |
| Zittern um das Ziel            | `TOL` und `V_TOL` erhöhen                 |

$A$ misst du wie in [Vorhersage-Regelung-MPC.md](Vorhersage-Regelung-MPC.md) beschrieben.

#### Stärken und Schwächen

| Stärken                                 | Schwächen                                     |
| --------------------------------------- | --------------------------------------------- |
| Sehr wenig Code, kaum Rechenzeit        | Servo wechselt dauernd die Richtung: Verschleiss und Lärm |
| Physik gut sichtbar: Bremsweg als Formel | `U` muss zwischen Haftgrenze und Pendeln liegen |
| Im Modell schnell und robust gegen Reibung | Immer gleich starkes Kippen, kein sanftes Anfahren |
| Haftreibung stört kaum, solange `U` gross genug ist | Reagiert der Ball viel stärker als angenommen, wird es unruhig |

#### Was die Simulation zeigt

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| **Normal** | 20/20 | 1.9 s | 20/20 | 1.7 s | 1.4 mm | 21 °/s | 0.0 |
| **A +33 %** | 12/20 | 4.0 s | 19/20 | 1.6 s | 2.3 mm | 21 °/s | 0.0 |
| **A −25 %** | 20/20 | 2.0 s | 20/20 | 2.0 s | 1.5 mm | 18 °/s | 0.0 |
| **Schwer** | 19/20 | 7.1 s | 19/20 | 2.7 s | 3.6 mm | 26 °/s | 0.0 |

**Wie stark zählt der Winkel?** Anzahl Läufe «Am Ziel» von 20, im Fall «Normal» mit Zeit. «Toter Bereich 4°» ist der Fall «Schwer» mit noch stärkerer Haftreibung, «real $A$ = 122» die theoretische Vollkugel.

| Winkel | Normal | A +33 % | A −25 % | Schwer | Toter Bereich 4° | real $A$ = 122 | Servo-Weg Normal |
| ------ | ------ | ------- | ------- | ------ | ---------------- | -------------- | ---------------- |
| 2° | 20, 2.0 s | 20 | 20 | 0 | 0 | 20 | 13 °/s |
| 2.5° | 20, 1.8 s | 20 | 20 | 18 | 0 | 11 | 17 °/s |
| **3°** | **20, 1.9 s** | **12** | **20** | **19** | **20** | **2** | **21 °/s** |
| 4° | 16, 1.8 s | 8 | 20 | 18 | 19 | 0 | 28 °/s |
| 10° | 0 | 0 | 0 | 0 | 0 | 0 | 39 °/s |
| 10° fern, 4° nah | 16, 7.5 s | 3 | 20 | 20 | 18 | 0 | 26 °/s |

$U$ = 2° ist im Modell bei wenig Reibung sehr gut, bleibt aber bei starker Haftreibung liegen. Die Wahl von 3° hält Abstand zu dieser Grenze. Sie fiel anhand genau dieser Fälle, die Zahlen sind also etwas zu optimistisch.

Das Modell schätzt nur grob ab, ob Vorzeichen und Logik stimmen. **Ein Test an der echten Schiene steht noch aus.** Beobachte dort den Servo: Dauernde Richtungswechsel belasten Getriebe und Hebel stärker als im Modell sichtbar.

#### Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. $A$ messen und eintragen.
3. Mit `U = 2` beginnen und erhöhen, bis der Ball zuverlässig losrollt. Schiene und Servo gut festhalten.
4. Ball nah am Ziel starten, danach weiter weg.

#### Verwandtschaft mit PID

Bang-Bang ist kaum verwandt mit PID: Der Winkel wächst nicht mit dem Fehler, sondern springt zwischen festen Werten. Es gibt keine Verstärkung, sondern nur die Frage «Gas oder Bremse?». Wer die Schaltgrösse auf die Form $s = e + \lambda \dot e$ vereinfacht, landet bei der **Sliding-Mode-Regelung**, einem verwandten eigenen Forschungsgebiet.

### 6.3 Fuzzy-Regelung

Du schreibst dein Wissen als Regeln in Worten auf, etwa «Ball weit weg und ruhig: stark kippen». Der Regler prüft, wie stark jede Regel gerade zutrifft, und mischt die Antworten.

#### Die Idee

Beim Duschen rechnest du nicht. Du denkst: «etwas zu kalt, also ein wenig wärmer». «Etwas» und «ein wenig» sind unscharf, englisch *fuzzy*. Fuzzy-Regelung macht solche Aussagen rechenbar. 22 °C ist dann zum Beispiel 70 % «kühl» und 30 % «angenehm», statt hart in eine Schublade zu fallen.

#### So funktioniert es

**1. Eingänge einordnen.** Position und Geschwindigkeit werden auf fünf Stützpunkte verteilt. Zwischen zwei Stützpunkten teilt sich das Gewicht linear auf:

| Stufe          | stark negativ | leicht negativ | null | leicht positiv | stark positiv |
| -------------- | ------------- | -------------- | ---- | -------------- | ------------- |
| `e` in mm      | −60           | −20            | 0    | +20            | +60           |
| `de` in mm/s   | −150          | −50            | 0    | +50            | +150          |

Bei `e` = 10 mm zählt die Position also je zu 50 % als «null» und «leicht positiv».

**2. Regeln nachschlagen.** Für jede Kombination steht ein Winkel in der Tabelle. Bei `e` = +20 mm und `de` = 0 ist der Ball zum Beispiel leicht zu nah am Sensor und ruhig: 3° reicht.

| `e` \ `de` | −150 | −50 | 0  | +50 | +150 |
| ---------- | ---- | --- | -- | --- | ---- |
| **−60**    | −16  | −13 | −9 | −5  | 0    |
| **−20**    | −15  | −7  | −3 | 0   | 9    |
| **0**      | −12  | −4  | 0  | 4   | 12   |
| **+20**    | −9   | 0   | 3  | 7   | 15   |
| **+60**    | 0    | 5   | 9  | 13  | 16   |

**3. Mischen.** Jede Regel zählt so viel, wie ihre beiden Eingänge zutreffen:

$$
u = \sum_{i,j} w_e(i) \cdot w_v(j) \cdot R_{ij}
$$

**Beispiel** `e` = 10 mm, `de` = −25 mm/s. Beide Eingänge liegen genau in der Mitte zwischen zwei Stützpunkten, also zählen vier Regeln zu je 25 %:

$$
u = 0.25 \cdot (-4) + 0.25 \cdot 0 + 0.25 \cdot 0 + 0.25 \cdot 3 = -0.25°
$$

Der Ball ist fast richtig unterwegs, der Regler greift kaum ein.

#### Code für deine Schiene

Vorzeichen wie in `main.py`: `e = x_set - Distanz`, und ein positiver Winkel rollt den Ball vom Sensor weg.

```python
E_PEAKS = (-60, -20, 0, 20, 60)         # mm
V_PEAKS = (-150, -50, 0, 50, 150)       # mm/s
RULES = (                               # ° Zeile: e, Spalte: de
    (-16, -13, -9, -5, 0),
    (-15, -7, -3, 0, 9),
    (-12, -4, 0, 4, 12),
    (-9, 0, 3, 7, 15),
    (0, 5, 9, 13, 16),
)

def weights(x, peaks):
    x = max(peaks[0], min(peaks[-1], x))
    w = [0.0]*len(peaks)
    for k in range(len(peaks) - 1):
        if x <= peaks[k + 1]:
            w[k + 1] = (x - peaks[k])/(peaks[k + 1] - peaks[k])
            w[k] = 1 - w[k + 1]
            return w
    return w

def regler(e, de, dt):
    we = weights(e, E_PEAKS)
    wv = weights(de, V_PEAKS)
    u = 0.0
    for i in range(5):
        for j in range(5):
            u += we[i]*wv[j]*RULES[i][j]
    return u
```

#### Einstellen

Du änderst nicht Faktoren, sondern einzelne Regeln. Das Plotter-Bild zeigt dir, in welcher Situation der Ball falsch reagiert.

| Beobachtung                  | Massnahme                                                 |
| ---------------------------- | --------------------------------------------------------- |
| Ball pendelt langsam und weit um das Ziel | Winkel bei `e` = 0 und `de` = ±50 vergrössern |
| Ball klebt knapp neben Ziel  | Winkel bei `e` = ±20 und `de` = 0 erhöhen                 |
| Schnelles Zittern in Zielnähe | Winkel bei `e` = 0 und `de` = ±50 verkleinern            |
| Ball langsam aus der Ferne   | Winkel bei `e` = ±60 und `de` = 0 erhöhen                 |
| Feinheit nahe am Ziel fehlt  | Stützpunkt bei ±8 mm ergänzen, Tabelle um Zeilen erweitern |

Halte die Tabelle **punktsymmetrisch**: Das Feld (−20, −50) ist das Negative von (+20, +50). So kannst du jede Änderung gleich gespiegelt eintragen.

#### Stärken und Schwächen

| Stärken                                        | Schwächen                                             |
| ---------------------------------------------- | ----------------------------------------------------- |
| Kein Modell nötig                              | Viele Zahlen in der Tabelle                           |
| Regeln lassen sich lesen und erklären          | Kein systematischer Weg zu guten Regeln               |
| Gezielte Änderung für genau eine Situation     | Bei grösserer Tabelle wächst der Rechenaufwand schnell |
| Wenig Rechenzeit bei 5×5                       | Kein I-Anteil: `OFFSET` muss stimmen            |

#### Was die Simulation zeigt

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| **Normal** | 20/20 | 1.6 s | 20/20 | 1.3 s | 1.0 mm | 21 °/s | 0.0 |
| **A +33 %** | 0/20 | – | 0/20 | – | 13.8 mm | 67 °/s | 0.0 |
| **A −25 %** | 20/20 | 1.3 s | 20/20 | 0.8 s | 0.6 mm | 19 °/s | 0.0 |
| **Schwer** | 20/20 | 1.5 s | 20/20 | 0.7 s | 3.3 mm | 27 °/s | 0.0 |

Das Modell schätzt nur grob ab, ob Vorzeichen und Logik stimmen. **Ein Test an der echten Schiene steht noch aus.** Die guten Werte im Fall «Normal» zeigen nicht, dass Fuzzy allgemein besser ist als PID. Der Fall «A +33 %» zeigt die Schwäche: Reagiert der Ball deutlich stärker als angenommen, schwingt die Tabelle dauerhaft. Dann verkleinerst du die Einträge, statt den Code zu ändern. Die Rechenzeit ist mit 3 ms angenommen, nicht gemessen.

#### Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. Servo noch aus: nur `print(e, de, regler(e, de, dt))`, Ball von Hand verschieben, Vorzeichen prüfen.
3. Alle Tabellenwerte halbieren, Servo aktivieren, Ball in Zielnähe legen.
4. Tabellenwerte schrittweise erhöhen, bis der Ball flott und ohne Schwingen ankommt.

#### Verwandtschaft mit PID

Hier ist die Nähe am grössten. Rechnerisch ist diese Tabelle ein **nichtlinearer PD-Regler**, denn sie bildet genau `e` und `de` auf einen Winkel ab. Die Tabellenwerte in der Nähe des Ziels entsprechen sogar grob $K_p \approx 0.15$ und $K_d \approx 0.08$. Neu ist die Denkweise: Du beschreibst Situationen in Worten, statt Faktoren zu tunen, und kannst jede Situation einzeln behandeln.

### 6.4 Golfer-Regelung (Impuls-Regelung)

Der Regler regelt nicht laufend. Er wartet, bis der Ball ruht, misst die Distanz zum Ziel und gibt einen berechneten Stoss. Dann schaut er, wo der Ball liegt, und lernt daraus für den nächsten Schlag.

#### Die Idee

Beim Putten korrigierst du den Ball nicht unterwegs. Du schätzt die Distanz, schlägst und schaust, wo er liegen bleibt. War der Schlag zu kurz, schlägst du beim nächsten Mal stärker. Genau so arbeitet dieser Regler.

#### So funktioniert es

Ein Schlag hat zwei gleich lange Phasen: Die Schiene kippt mit Winkel $U$ für die Zeit $T$ in Richtung Ziel, danach genauso lang zurück, um den Ball wieder abzubremsen. Der Ball legt dabei diesen Weg zurück:

$$
s = A \cdot U \cdot T^2 \qquad \Rightarrow \qquad T = \sqrt{\frac{|e|}{A \cdot U}}
$$

**Beispiel:** `e` = 100 mm, $A$ = 60, $U$ = 8°: $T = \sqrt{100 / 480} \approx 0.46$ s, also 0.46 s kippen und 0.46 s zurück.

Ablauf:

1. **Warten:** Schiene waagrecht, bis der Ball 0.3 s ruht.
2. **Prüfen:** Im Zielbereich? Dann bleibt alles, wie es ist.
3. **Schlag:** $T$ berechnen und den Schlag ausführen.
4. **Lernen:** Nach dem Schlag misst der Regler den wahren Weg und korrigiert seine Schätzung von $A$:

$$
A_{\text{neu}} = 0.7 \cdot A_{\text{alt}} + 0.3 \cdot \frac{s_{\text{gemessen}}}{U \cdot T^2}
$$

5. **Bremsen:** Rollt der Ball nach dem Schlag noch stark, folgt ein kurzer Gegenstoss.

#### Code für deine Schiene

Vorzeichen wie in `main.py`: `e = x_set - Distanz`, und ein positiver Winkel rollt den Ball vom Sensor weg.

```python
from math import sqrt

U = 8                       # °    Kippwinkel beim Schlag
V_REST, T_REST = 20, 0.3    # mm/s, s  Ball gilt als ruhend
V_BRAKE = 60                # mm/s schneller: Gegenstoss
T_QUIET = 0.3               # s    nach Stoss warten, bis der Filter stimmt
TOL = 6                     # mm   Ziel erreicht
T_MIN, T_MAX = 0.08, 0.8    # s    Grenzen für einen Schlag

class Golfer:
    def __init__(self):
        self.A = 60.0       # Startwert, wird nachgelernt
        self.state = "wait"
        self.rest = self.quiet = self.timer = 0.0
        self.T1 = self.T2 = self.e_start = 0.0
        self.dir = 1
        self.fresh = False

    def update(self, e, de, dt):
        if self.state == "wait":
            self.quiet -= dt
            if self.quiet > 0:
                return 0
            if abs(de) > V_BRAKE:           # rollt noch: bremsen
                self.dir = 1 if de > 0 else -1
                self.T1 = min(abs(de)/(self.A*U), T_MAX)
                self.T2 = 0.0
                self.timer, self.state, self.fresh = 0.0, "pulse", False
            else:
                self.rest = self.rest + dt if abs(de) < V_REST else 0.0
                if self.rest < T_REST:
                    return 0
                if self.fresh:              # Schlag auswerten, A nachlernen
                    moved = (self.e_start - e)*self.dir
                    A_new = moved/(U*self.T1*self.T1)
                    self.A = max(20, min(200, 0.7*self.A + 0.3*A_new))
                    self.fresh = False
                if abs(e) < TOL:
                    return 0
                self.dir = 1 if e > 0 else -1   # neuer Schlag
                self.T1 = max(T_MIN, min(T_MAX, sqrt(abs(e)/(self.A*U))))
                self.T2 = self.T1
                self.e_start, self.timer, self.state = e, 0.0, "pulse"
                self.fresh = True
        self.timer += dt
        if self.timer < self.T1:
            return self.dir*U
        if self.timer < self.T1 + self.T2:
            return -self.dir*U
        self.state, self.rest, self.quiet = "wait", 0.0, T_QUIET
        return 0

golfer = Golfer()

def regler(e, de, dt):
    return golfer.update(e, de, dt)
```

**Einbau:** `measure()`, `setAngle()` und die Initialisierung bleiben aus `main.py`. Die Regelschleife am Ende wird ersetzt durch:

```python
while True:
    e_raw, dt = measure()
    u = regler(e, de, dt)
    setAngle(u)
    print("%.1f, %.1f, %.0f" % (e, u, golfer.A))
```

Das dritte Plotter-Signal zeigt dir, wie der Regler $A$ nachlernt.

#### Einstellen

| Beobachtung                       | Massnahme                                        |
| --------------------------------- | ------------------------------------------------ |
| Schlag bewegt den Ball kaum       | `U` erhöhen, muss klar über dem toten Bereich liegen |
| Ball schiesst oft über das Ziel   | `U` senken, `T_MAX` senken                       |
| Regler startet nie einen Schlag   | `V_REST` erhöhen, liegt vermutlich unter dem Rauschen |
| Ball rollt nach dem Schlag davon  | `test_offset.py` neu laufen lassen, Schiene ist schief |

Wichtig: `V_REST` muss über dem Rauschen von `de` liegen. In `main.py` steht dazu `V_TOL = 15` als «knapp über Rauschen».

#### Stärken und Schwächen

| Stärken                                      | Schwächen                                           |
| -------------------------------------------- | --------------------------------------------------- |
| Haftreibung ist Teil der Methode, kein Störfaktor | Langsam: pro Schlag rund 1–2 s                 |
| Lernt $A$ selbst, keine Messung nötig        | Eine schiefe Schiene (`OFFSET`) lässt den Ball davonrollen |
| Servo ist die meiste Zeit in Ruhe            | Reagiert während des Schlags nicht auf Störungen    |
| Gut sichtbar und gut erklärbar               | Kurze Schläge gehen in der Servo-Verzögerung unter  |

#### Was die Simulation zeigt

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| **Normal** | 13/20 | 3.1 s | 9/20 | 5.2 s | 2.9 mm | 13 °/s | 0.1 |
| **A +33 %** | 9/20 | 3.7 s | 7/20 | 3.2 s | 8.5 mm | 16 °/s | 0.7 |
| **A −25 %** | 15/20 | 8.0 s | 12/20 | 4.9 s | 3.2 mm | 15 °/s | 0.0 |
| **Schwer** | 0/20 | – | 3/20 | 4.4 s | 63.3 mm | 14 °/s | 0.1 |

Das Modell schätzt nur grob ab, ob Vorzeichen und Logik stimmen. **Ein Test an der echten Schiene steht noch aus.** Im Fall «Schwer» liegt die Schiene 0.5° schief und der Ball rollt nach dem Schlag davon. Die Methode steht und fällt also mit gutem `OFFSET`. Sie blieb in der Simulation klar unzuverlässiger als MPC, Bang-Bang und Fuzzy.

#### Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen. Ohne das hat diese Methode keine Chance.
2. Ball rund 100 mm neben das Ziel legen und einen einzelnen Schlag beobachten.
3. Prüfen, ob der Ball nach dem Schlag wirklich stehen bleibt. Wenn nicht: `V_BRAKE` senken.
4. Im Plotter verfolgen, ob sich $A$ über mehrere Schläge beruhigt.

#### Verwandtschaft mit PID

Keine. Es gibt keinen laufenden Vergleich von Fehler und Winkel. Der Regler handelt nur selten und in fertigen Schlägen und passt zwischen den Schlägen sein Modell an. Fachlich liegt die Methode nahe bei *Iterative Learning Control* und *Event-triggered Control*.

### 6.5 Imitations-Regelung

Du steuerst die Schiene ein paar Minuten von Hand mit einem Potentiometer. Der Pico merkt sich, was du in welcher Situation getan hast, und macht es danach allein nach. Fachlich heisst das *Imitation Learning* oder *Behavioral Cloning*.

#### Die Idee

Ein Lehrling schaut einer erfahrenen Person bei der Arbeit zu und merkt sich: «In dieser Lage macht sie jenes.» Später braucht er keine Formel, sondern nur die Erinnerung. Hier ist die Erinnerung eine Tabelle.

#### So funktioniert es

**Phase 1: Vormachen.** Du drehst am Poti und hältst den Ball am Ziel. Alle 30 ms speichert der Pico die Situation (`e`, `de`) und deinen Winkel `u`. Er teilt die Situationen in Felder auf und summiert pro Feld:

| Grösse | Feldgrenzen                                      | Felder |
| ------ | ------------------------------------------------ | ------ |
| `e`    | −100, −50, −20, −8, +8, +20, +50, +100 mm        | 9      |
| `de`   | −120, −40, −12, +12, +40, +120 mm/s              | 7      |

Das ergibt 63 Felder. Felder sind nahe am Ziel kleiner, weil es dort auf Feinheit ankommt.

**Phase 2: Tabelle bauen.** Jedes Feld bekommt den Mittelwert deiner Winkel:

$$
R_{ij} = \frac{1}{n_{ij}} \sum u_{ij}
$$

Felder ohne Daten übernehmen den Wert des nächstgelegenen Felds mit Daten.

**Phase 3: Nachmachen.** Der Pico schlägt in der Tabelle nach. Es gibt keine Physik und keine Regel, nur deine Erfahrung als Zahlen.

#### Hardware

Du brauchst zusätzlich ein Potentiometer. `ADC(26)` entspricht GP26. Der Schleifer kommt an GP26, die beiden anderen Anschlüsse an 3V3 und GND. Prüfe die Belegung deines Boards oder Shields.

#### Code für deine Schiene

Vorzeichen wie in `main.py`: `e = x_set - Distanz`, und ein positiver Winkel rollt den Ball vom Sensor weg.

```python
E_EDGES = (-100, -50, -20, -8, 8, 20, 50, 100)  # mm
V_EDGES = (-120, -40, -12, 12, 40, 120)         # mm/s
ROWS, COLS = len(E_EDGES) + 1, len(V_EDGES) + 1
summe = [[0.0]*COLS for _ in range(ROWS)]
anzahl = [[0]*COLS for _ in range(ROWS)]
table = None

def zelle(x, edges):
    for k, edge in enumerate(edges):
        if x < edge:
            return k
    return len(edges)

def merke(e, de, u):
    i, j = zelle(e, E_EDGES), zelle(de, V_EDGES)
    summe[i][j] += u
    anzahl[i][j] += 1

def baue_tabelle():
    t = [[0.0]*COLS for _ in range(ROWS)]
    for i in range(ROWS):
        for j in range(COLS):
            best = None             # leere Felder: nächstes Feld mit Daten
            for a in range(ROWS):
                for b in range(COLS):
                    d = abs(a - i) + abs(b - j)
                    if anzahl[a][b] and (best is None or d < best[0]):
                        best = (d, summe[a][b]/anzahl[a][b])
            t[i][j] = best[1]
    return t

def regler(e, de, dt):
    return table[zelle(e, E_EDGES)][zelle(de, V_EDGES)]
```

**Vormachen:** Die Regelschleife am Ende von `main.py` wird ersetzt durch diesen Teil. Er läuft 5 Minuten und speichert dann die Tabelle.

```python
import json
from machine import ADC

poti = ADC(26)
U_MAX = 15          # °  beim Poti-Vollausschlag

t_start = ticks_ms()
while ticks_diff(ticks_ms(), t_start) < 5*60*1000:
    e_raw, dt = measure()
    u = (poti.read_u16()/65535 - 0.5)*2*U_MAX
    setAngle(u)
    merke(e, de, u)

with open("tabelle.json", "w") as f:
    json.dump(baue_tabelle(), f)
print("Tabelle gespeichert")
```

**Nachmachen:** Danach ersetzt du diesen Teil durch:

```python
import json

with open("tabelle.json") as f:
    table = json.load(f)

while True:
    e_raw, dt = measure()
    u = regler(e, de, dt)
    setAngle(u)
    print("%.1f, %.1f" % (e, u))
```

#### So machst du gut vor

- Lege den Ball an viele verschiedene Stellen, auch an die Schienenenden. Sonst bleiben diese Felder leer.
- Stosse den Ball auch mal an. Auch das Abfangen schneller Bälle muss in die Tabelle.
- Beginne mit kleinem `U_MAX`. Mit langsamerem Ball kannst du die Schiene besser führen.
- Ruhig bleiben. Hektische Korrekturen landen genauso in der Tabelle.

#### Einstellen

| Beobachtung                         | Massnahme                                  |
| ----------------------------------- | ------------------------------------------ |
| Ball rollt in seltenen Lagen davon  | Mehr vormachen, genau diese Lagen üben     |
| Ball pendelt nahe am Ziel           | Sauberer vormachen, feinere Felder nahe null |
| Winkel springt hörbar               | Mit `weights()` aus [Fuzzy-Regelung.md](Fuzzy-Regelung.md) zwischen Feldern mischen |

Mit dieser Mischung wäre die Tabelle eine **Fuzzy-Tabelle, die aus deinen Daten gelernt ist**.

#### Stärken und Schwächen

| Stärken                                        | Schwächen                                      |
| ---------------------------------------------- | ---------------------------------------------- |
| Kein Modell und keine Reglertheorie nötig      | Du musst die Aufgabe selbst können             |
| Starke Demo: Der Pico macht es wie du          | Kann in ungeübten Lagen versagen               |
| Mittelwert glättet zufälliges Zittern der Hand | Macht auch deine systematischen Fehler nach    |
| Tabelle passt in sehr wenig Speicher           | Poti und Lernphase nötig                       |

#### Was die Simulation zeigt

Statt dir sitzt im Modell ein synthetischer «Mensch»: Er reagiert 150 ms verspätet, führt die Hand träge und zittert leicht. Er lernt 4 Minuten lang mit der Tabelle, danach übernimmt die Tabelle allein.

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| Synth. Mensch, Normal | 8/20 | 7.3 s | 0/20 | – | 7.0 mm | 20 °/s | 0.0 |
| **Normal** | 18/20 | 5.2 s | 15/20 | 4.9 s | 3.2 mm | 11 °/s | 0.0 |
| **A +33 %** | 20/20 | 3.6 s | 19/20 | 3.4 s | 3.6 mm | 10 °/s | 0.0 |
| **A −25 %** | 13/20 | 6.9 s | 6/20 | 4.5 s | 4.2 mm | 12 °/s | 0.0 |
| **Schwer** | 19/20 | 6.6 s | 18/20 | 4.1 s | 3.0 mm | 13 °/s | 0.0 |

**Die Zahlen sagen wenig über deine Hand aus.** Der synthetische Mensch ist nur eine Formel und ein Test an der echten Schiene steht noch aus. 10 von 63 Feldern blieben im Training leer.

Im Fall «Normal» schneidet die Tabelle besser ab als ihr Lehrer, vermutlich weil der Mittelwert das Handzittern herausrechnet. Das gilt nicht immer: Läuft die Schiene träger als beim Vormachen (Fall «A −25 %»), wird die Tabelle deutlich schlechter. Lehre deshalb an genau der Schiene, die der Regler später führt.

Ob du die Schiene von Hand überhaupt stabil hältst, zeigt erst der Versuch. Teste das, bevor du Zeit in den Rest steckst.

#### Erster Test an der Schiene

1. Nur das Poti an `setAngle()` anschliessen und 2 Minuten üben. Gelingt das nicht, `U_MAX` senken. Bleibt es zu schwierig, passt die Methode nicht.
2. Vormachen, 5 Minuten.
3. Tabelle laden, Ball in Zielnähe legen, zuschauen.
4. Ball anstossen und schauen, ob die Tabelle fängt.

#### Verwandtschaft mit PID

Keine in der Entstehung. Niemand wählt Faktoren oder Regeln, die Tabelle entsteht aus Beobachtung. Das Resultat ist aber wieder eine Abbildung von `e` und `de` auf einen Winkel. Das ist Physik: Jeder Regler für diesen Ball braucht diese zwei Grössen.

### 6.6 Q-Learning-Regelung

Niemand sagt dem Regler, wie er kippen soll. Er probiert in einer Simulation tausende Male aus, bekommt Belohnung und Strafe und lernt daraus eine Tabelle. Q-Learning ist eine Methode des *Reinforcement Learning* (bestärkendes Lernen).

#### Die Idee

Ein Hund lernt Sitz, ohne dass ihm jemand die Muskeln erklärt: Macht er das Richtige, gibt es ein Leckerli. Mit der Zeit merkt er sich, welche Handlung in welcher Situation am meisten bringt. Der Q-Learning-Regler arbeitet genauso, nur gibt es statt Leckerli eine Punktzahl.

#### So funktioniert es

**Zustand:** Position und Geschwindigkeit, in Felder eingeteilt (11 × 9 = 99 Zustände).

**Aktion:** einer von 7 Winkeln: −10°, −5°, −2°, 0°, 2°, 5°, 10°.

**Q-Tabelle:** Für jeden Zustand und jede Aktion steht eine Zahl: «Wie viel Punkte erwarte ich langfristig, wenn ich hier diese Aktion wähle?» Am Anfang stehen überall Nullen.

**Lernregel:** Nach jedem Schritt wird der Wert nachgezogen:

$$
Q(s,a) \leftarrow Q(s,a) + \alpha \left[ r + \gamma \max_{a'} Q(s',a') - Q(s,a) \right]
$$

| Symbol     | Bedeutung                                                 | Wert           |
| ---------- | --------------------------------------------------------- | -------------- |
| $r$        | Punkte für diesen Schritt                                 | siehe unten    |
| $\alpha$   | Lernrate: wie stark neue Erfahrung zählt                  | 0.23 → 0.03    |
| $\gamma$   | Wie wichtig die Zukunft ist                               | 0.97           |
| $\epsilon$ | Anteil zufälliger Aktionen zum Ausprobieren               | 0.32 → 0.02    |

**Belohnung:** Hier steckt die eigentliche Arbeit, denn der Regler lernt genau das, was hier steht:

$$
r = -\frac{|e|}{100} - 0.1 \cdot \frac{|\dot e|}{100} - 0.03 \cdot |u| + \text{Bonus im Ziel} - \text{Strafe am Anschlag}
$$

**Warum am PC?** Ein Training hat 6000 Läufe zu 12 s, auf der echten Schiene wären das 20 Stunden Servo-Dauerbetrieb. Den Ball zurücklegen müsstest du nicht, die Schiene kann ihn selbst zurückrollen wie in `test_offset.py`. Darum lernt der Regler in einem Modell am PC, und nur das Ergebnis kommt auf den Pico.

#### Training am PC

Normales Python am Laptop, nicht auf dem Pico. Es trainiert fünfmal und gibt die beste Tabelle aus, denn das Ergebnis hängt stark vom Zufall ab. Dauert rund zwei Minuten.

```python
import random

A, DEAD, ROLL = 60.0, 1.5, 0.2      # Modell: an deine Schiene anpassen
LENGTH, SENSOR, X_SET = 400.0, 30.0, 200
ALPHA, BETA = 0.3, 0.35             # gleich wie main.py
Q_E = (-100, -50, -25, -10, -4, 4, 10, 25, 50, 100)     # mm
Q_V = (-150, -60, -20, -6, 6, 20, 60, 150)              # mm/s
Q_U = (-10, -5, -2, 0, 2, 5, 10)                        # °
SKIP = 3            # eine neue Aktion nur jeden 3. Durchgang
GAMMA, PEN, BONUS = 0.97, 0.03, 1

def zelle(x, edges):
    for k, edge in enumerate(edges):
        if x < edge:
            return k
    return len(edges)

class Schiene:
    def __init__(self):
        self.p, self.v = random.uniform(20, 380), random.uniform(-60, 60)
        self.th = self.u = 0.0
        self.hit = False

    def rollen(self, T, dt=0.003):
        for _ in range(round(T/dt)):
            self.th += (self.u - self.th)/0.04*dt       # Servo-Verzögerung
            if self.v == 0:
                if abs(self.th) > DEAD/2:               # Haftreibung
                    self.v = A*(self.th - ROLL*(1 if self.th > 0 else -1))*dt
            else:
                vn = self.v + A*(self.th - ROLL*(1 if self.v > 0 else -1))*dt
                if vn*self.v < 0 and abs(self.th) < DEAD/2:
                    vn = 0.0
                self.v = vn
            self.p += self.v*dt
            if not 0 <= self.p <= LENGTH:
                self.p, self.v, self.hit = max(0.0, min(LENGTH, self.p)), 0.0, True

    def messen(self):
        self.rollen(0.015)
        e = X_SET - min(self.p + SENSOR + random.gauss(0, 2), 400)
        self.rollen(0.015)
        return e

def episode(q, eps, lr):
    """Ein Lauf von 12 s. Gibt zurück, ob der Ball am Schluss am Ziel liegt."""
    w = Schiene()
    e, de, t = w.messen(), 0.0, 0.0
    s = (zelle(e, Q_E), zelle(de, Q_V))
    while t < 12:
        c = q[s[0]][s[1]]
        a = random.randrange(len(Q_U)) if random.random() < eps else c.index(max(c))
        w.u, w.hit = Q_U[a], False
        for _ in range(SKIP):
            e_raw = w.messen()
            e_old, t = e, t + 0.03
            e = ALPHA*e_raw + (1 - ALPHA)*(e + de*0.03)
            de = BETA*(e - e_old)/0.03 + (1 - BETA)*de
        r = -abs(e)/100 - 0.1*abs(de)/100 - PEN*abs(Q_U[a])
        if abs(e) < 7.5 and abs(de) < 15:
            r += BONUS
        if w.hit:
            r -= 5
        s2 = (zelle(e, Q_E), zelle(de, Q_V))
        c[a] += lr*(r + GAMMA*max(q[s2[0]][s2[1]]) - c[a])
        # Spiegelbild ist physikalisch gleich: doppelt lernen
        m, m2 = q[len(Q_E) - s[0]][len(Q_V) - s[1]], q[len(Q_E) - s2[0]][len(Q_V) - s2[1]]
        ma = len(Q_U) - 1 - a
        m[ma] += lr*(r + GAMMA*max(m2) - m[ma])
        s = s2
    return abs(e) < 7.5

def trainiere(episoden=6000):
    q = [[[0.0]*len(Q_U) for _ in range(len(Q_V) + 1)] for _ in range(len(Q_E) + 1)]
    for ep in range(episoden):
        frac = ep/episoden
        episode(q, 0.3*(1 - frac) + 0.02, 0.2*(1 - frac) + 0.03)
    return q

beste, beste_note = None, -1
for seed in (1, 2, 3, 7, 11):
    random.seed(seed)
    q = trainiere()
    note = sum(episode(q, 0, 0) for _ in range(50))     # Test ohne Zufall, ohne Lernen
    print("# Seed", seed, "am Ziel:", note, "von 50")
    if note > beste_note:
        beste, beste_note = q, note

for zeile in beste:
    print(tuple(Q_U[c.index(max(c))] for c in zeile), ",", sep="")
```

#### Code für den Pico

Auf dem Pico läuft nur noch Nachschlagen. Die gelernte Tabelle unten stammt aus dem Training oben mit $A$ = 60. Bei anderem $A$ passt sie nicht: Trainiere neu und kopiere die Ausgabe hinein.

```python
Q_E = (-100, -50, -25, -10, -4, 4, 10, 25, 50, 100)     # mm
Q_V = (-150, -60, -20, -6, 6, 20, 60, 150)              # mm/s
SKIP = 3
BEST = (
    # Seed 1 am Ziel: 45 von 50
    # Seed 2 am Ziel: 28 von 50
    # Seed 3 am Ziel: 33 von 50
    # Seed 7 am Ziel: 24 von 50
    # Seed 11 am Ziel: 40 von 50
    (-10, -5, -5, -5, -10, -5, -5, 0, 0),
    (-5, -10, -2, -5, -2, -10, -5, 0, 5),
    (2, 0, -10, 0, -2, 5, 2, 5, 5),
    (-5, -10, -2, -5, 0, 0, 0, 10, 0),
    (5, 0, -5, 0, -2, 5, 0, 10, 0),
    (-5, 10, -2, 0, 0, 0, 2, -10, 5),
    (0, -10, 0, -5, 2, 0, 5, 0, -5),
    (0, -10, 0, 0, 0, 5, 2, 10, 5),
    (-5, -5, -2, -5, 2, 0, 10, 0, -2),
    (-5, 0, 5, 10, 2, 5, 2, 10, 5),
    (0, 0, 5, 5, 10, 5, 5, 5, 10),
)
n, u_alt = 0, 0

def zelle(x, edges):
    for k, edge in enumerate(edges):
        if x < edge:
            return k
    return len(edges)

def regler(e, de, dt):
    global n, u_alt
    if n % SKIP == 0:
        u_alt = BEST[zelle(e, Q_E)][zelle(de, Q_V)]
    n += 1
    return u_alt
```

#### Einstellen

Du änderst nicht den Regler, sondern Belohnung und Modell, und trainierst neu.

| Beobachtung                    | Massnahme                                                    |
| ------------------------------ | ------------------------------------------------------------ |
| Ball bleibt knapp neben Ziel   | Feinere Felder nahe null, `BONUS` vorsichtig erhöhen         |
| Servo zittert                  | `PEN` erhöhen, `SKIP` erhöhen                                |
| Regler ist zu faul und kippt kaum | `PEN` senken                                              |
| Läuft im Modell, aber nicht echt | $A$, `DEAD` und `ROLL` im Modell auf gemessene Werte setzen |

**Vorsicht beim Vergleichen:** Beurteile eine Änderung nie anhand eines einzelnen Trainings. Mit `BONUS = 3` und `PEN = 0.05` lag «Am Ziel» über dieselben fünf Trainings zwischen 0 und 11 von 20, Median 8. Das ist kein klarer Fortschritt gegenüber der Standardeinstellung mit Median 5.

Tipp für später: Variiere $A$ und `DEAD` bei jeder Episode zufällig. Der Fachbegriff dafür ist *Domain Randomization*. Der Regler wird dadurch unempfindlicher gegen Modellfehler.

#### Stärken und Schwächen

| Stärken                                          | Schwächen                                          |
| ------------------------------------------------ | -------------------------------------------------- |
| Niemand muss die Lösung vorgeben                 | Ergebnis schwankt stark von Training zu Training   |
| Auf dem Pico extrem wenig Aufwand und Speicher   | Grobe Tabelle: feste Winkel, Sprünge               |
| Lernt auch ungewohnte Strategien                 | Modellfehler landen direkt in der Tabelle          |
| Starker Einstieg in Machine Learning             | Belohnung tunen ist heikel und langwierig          |

#### Was die Simulation zeigt

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| **Normal** | 15/20 | 4.8 s | 14/20 | 5.4 s | 1.9 mm | 24 °/s | 0.0 |
| **A +33 %** | 5/20 | 9.1 s | 5/20 | 6.1 s | 11.1 mm | 46 °/s | 0.0 |
| **A −25 %** | 18/20 | 7.6 s | 17/20 | 4.2 s | 2.2 mm | 19 °/s | 0.0 |
| **Schwer** | 7/20 | 8.8 s | 2/20 | 3.8 s | 5.4 mm | 28 °/s | 0.0 |

**Das ist die Tabelle, die der Test im Skript aus fünf Trainings wählte, nicht der Normalfall.** Im Fall «Normal» erreichten die fünf Tabellen 4, 4, 5, 15 und 17 von 20 Läufen. Der Skript-Test wählte die mit 15. Rechne also immer mit mehreren Trainings. Die Auswahl geschieht im Modell und ist deshalb etwas zu optimistisch.

Auch diese Zahlen stammen aus dem Modell, **ein Test an der echten Schiene steht noch aus**. Dort dürfte es eher schlechter laufen, weil die Tabelle ihr Modell für die Wirklichkeit hält.

#### Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. $A$ messen wie in [Vorhersage-Regelung-MPC.md](Vorhersage-Regelung-MPC.md), neu trainieren.
3. Tabelle auf den Pico, Ball in Zielnähe legen, zuschauen.
4. Weicht das Verhalten stark ab, das Modell im Training an die echte Schiene angleichen.

#### Verwandtschaft mit PID

Keine in der Herkunft: Hier bestimmt ein Lernverfahren die Tabelle, weder Faktoren noch Regeln stammen von dir. Wie jeder Regler hier bildet aber auch sie `e` und `de` auf einen Winkel ab.

### 6.7 Gebogene Schiene: Regelung durch Mechanik

Statt die Regelung zu verbessern, änderst du die Form: Die Schiene wird zur Mulde. Der Ball rollt von selbst zum tiefsten Punkt, und Kippen verschiebt diesen Punkt. Es braucht fast keinen Regler mehr.

#### Die Idee

Eine Murmel in einer Salatschüssel findet den tiefsten Punkt ohne Elektronik. Auf der geraden Schiene ist der Ball instabil, denn er rollt davon, sobald sie nicht genau waagrecht liegt. In der Mulde ist der Ball **von Natur aus stabil**: Wird er gestört, rollt er zurück. Die Regelung steckt in der Geometrie.

#### So funktioniert es

Die Schiene sei ein Kreisbogen mit Radius $R$. Kippst du sie um den Winkel $\varphi$, wandert der Tiefpunkt:

$$
x_{\text{Tief}} = R \cdot \varphi \quad (\varphi \text{ in Radiant})
$$

Der Sollwert ist also direkt ein Winkel. Dieser Ablauf heisst **Steuerung** statt Regelung: Kein Sensor ist nötig, um den Ball hinzubringen.

Der Ball pendelt wie ein Pendel um den Tiefpunkt, mit dieser Periode:

$$
T = 2\pi \sqrt{\frac{R}{\tfrac{5}{7} g}}
$$

Es bleiben zwei Probleme:

**Der Ball pendelt lange.** Rollreibung dämpft schwach. Die Lösung ist elegant und heisst **Input Shaping** oder *Posicast*: Kippe in zwei Stufen, zuerst die halbe Änderung und nach $T/2$ den Rest. Die zweite Stufe löscht das Pendeln der ersten genau aus, der Ball kommt direkt zur Ruhe.

**Die Haftreibung bestimmt die Genauigkeit.** Der Ball bleibt überall dort liegen, wo die Steigung unter der Haftgrenze liegt. Bei halbem totem Bereich $\theta_H$ ergibt das einen Fehler bis $\pm R \cdot \theta_H$.

Radius ist damit der entscheidende Kompromiss (Schiene 400 mm, $\theta_H$ = 0.75°):

| Radius $R$ | Periode $T$ | Kippen für ±100 mm | Mögliche Abweichung | Durchhang der Mitte |
| ---------- | ----------- | ------------------ | ------------------- | ------------------- |
| 500 mm     | 1.68 s      | ±11.5°             | ±6.5 mm             | 40 mm               |
| 1000 mm    | 2.38 s      | ±5.7°              | ±13 mm              | 20 mm               |
| 2000 mm    | 3.36 s      | ±2.9°              | ±26 mm              | 10 mm               |

Kleiner Radius heisst: genauer und schneller, aber stärker gebogen. Für den Sollbereich von ±7.5 mm aus `main.py` passt nur ungefähr $R$ = 500 mm.

#### Bauen

- **Schiene:** Alu-Profil biegen, Karton oder Holz in Bogenform als Unterlage, oder einen Bogen 3D-drucken.
- **Sensor:** Der Sensor muss den Ball weiterhin sehen. Bei 40 mm Durchhang kann die ansteigende Schiene hinten selbst in den Messkegel ragen und falsche Werte liefern. Setze ihn tief und prüfe es vor dem Umbau mit einer provisorischen Mulde.
- **Hebel:** $\varphi$ ist der Winkel der **Schiene**, nicht des Servos. Bestimme das Verhältnis `RATIO`: Servo um 10° drehen und die Schienenneigung messen.
- **Radius bestimmen:** Ball anstossen, 5 Perioden stoppen, dann $R = \tfrac{5}{7} g \cdot (T/2\pi)^2$.

#### Code für deine Schiene

Es braucht nur die Funktion, die den Winkel in zwei Stufen setzt. Der Sensor dient bloss zur Kontrolle.

```python
from math import degrees
from utime import sleep_ms

R = 500             # mm  Radius, gemessen
T = 1.68            # s   Periode, gemessen
RATIO = 1.0         # Grad Servo pro Grad Schiene, gemessen
phi = 0.0           # °   aktueller Schienenwinkel

def fahre_zu(x):
    """x in mm: Soll-Position relativ zum Tiefpunkt der waagrechten Schiene.
    Blockiert T/2. Ball muss vorher ruhen."""
    global phi
    neu = degrees(x/R)
    setAngle((phi + (neu - phi)/2)*RATIO)   # halbe Änderung
    sleep_ms(int(T/2*1000))
    setAngle(neu*RATIO)                     # zweite Hälfte
    phi = neu
```

Positives `x` bedeutet wie in `main.py` eine Bewegung vom Sensor weg. Demo mit wechselnden Zielen:

```python
while True:
    for x in (-60, 60, 0):
        fahre_zu(x)
        sleep_ms(3000)
        print("Soll %d, Distanz %d" % (x, tof.ping()))
```

#### Stärken und Schwächen

| Stärken                                        | Schwächen                                        |
| ---------------------------------------------- | ------------------------------------------------ |
| Fast kein Code, kein Tuning von Reglern        | Umbau der Hardware                               |
| Stabil auch bei Stromausfall oder Programmfehler | Genauigkeit durch Haftreibung begrenzt          |
| Ball kehrt nach Störung von selbst zurück      | Sensor könnte die Schiene statt des Balls sehen  |
| Auch ohne Sensor lauffähig                     | Pendeln nach Störung klingt nur langsam ab       |
| Überraschender Ansatz, gut für eine Demo       | **Trifft vermutlich nicht den Sinn einer Regelungsaufgabe** |

Falls das Projekt eine Regelung verlangt, kläre die Methode zuerst mit der Betreuung. Sie zeigt aber gut, warum die gerade Schiene überhaupt schwierig ist.

#### Was die Simulation zeigt

Modell: Ball auf Kreisbogen, Haftreibung mit totem Bereich 1.5°, Rollreibung 0.1°, Kippen um 3° Schiene ohne äussere Störung. Es zeigt die Position nach 25 s und die Zeit, bis der Ball dauerhaft innerhalb ±7.5 mm um den idealen Tiefpunkt liegt.

| Radius  | Ziel     | In einem Schritt            | In zwei Stufen ($T/2$)       |
| ------- | -------- | --------------------------- | ---------------------------- |
| 500 mm  | 26.2 mm  | Ende 20.9 mm, nach 8.6 s    | Ende 24.4 mm, nach 0.6 s     |
| 1000 mm | 52.4 mm  | Ende 41.9 mm, nie im Bereich | Ende 48.9 mm, nach 1.0 s    |
| 2000 mm | 104.7 mm | Ende 83.8 mm, nie im Bereich | Ende 97.7 mm, nach 1.6 s    |

Bei $R$ = 500 mm kommt der Ball mit zwei Stufen rund 14-mal schneller zur Ruhe und näher ans Ziel. Bei grösserem Radius erreicht der einstufige Ablauf den Bereich gar nicht.

**Empfindlichkeit:** Ist $T$ um 10 % falsch, hält $R$ = 500 mm den Bereich noch (Ende 22.5 mm), $R$ = 2000 mm nicht mehr. Bei 20 % fällt auch $R$ = 1000 mm heraus. Miss $T$ also sauber.

Das Modell schätzt nur grob ab, ob die Physik aufgeht. **Es wurde nichts gebaut und nichts an echter Hardware getestet.**

#### Erster Test an der Schiene

1. Vor dem Umbau: Biege ein Stück Karton als Mulde und lege den Ball hinein. Bleibt er genau genug liegen?
2. Sensor provisorisch montieren: Sieht er den Ball auch am Muldenende, und misst er ohne Ball nicht die Schiene?
3. Erst dann die Schiene bauen, $R$, $T$ und `RATIO` messen.

#### Verwandtschaft mit PID

Keine. Es läuft keine Rückführung, also auch kein Regelkreis. Willst du die Reibungsabweichung loswerden, kannst du nach dem Stillstand messen und $\varphi$ um $\Delta x / R$ nachstellen. Das ist dann wieder eine langsame Rückführung, im Wesen ein kleiner I-Anteil.

## 7. Simulationscode

**Einrichten:** Beide Blöcke als `sim_methods.py` und `check_docs.py` im selben Ordner speichern. Nicht auf dem Pico. In `check_docs.py` muss `DOCS` auf den Ordner `Ball_on_beam` zeigen.

```
python check_docs.py            # prüft Beispiele und Tabellen, unter 1 Minute
python check_docs.py --write    # schreibt Tabellen in die MDs neu
python check_docs.py --q-seeds  # zusätzlich Q-Learning-Vergleich, rund 5 Minuten
```

**Neue Methode testen:** MD mit einem ` ```python `-Block, der `def regler(e, de, dt)` enthält, dazu eine `report(...)`-Zeile in `check_docs.py`. Die Ergebnistabelle ersetzt `report` beim ersten `| Fall | Am Ziel`-Abschnitt im MD, die Zeile muss also existieren.

### 7.1 sim_methods.py

```python
# PC simulation for the ball-on-beam controller docs. Not for the Pico.
# Plant follows Scripts/test_offset_sim.py (stiction), plus rolling friction, servo lag and
# sensor noise. Holt() and PIDMain mirror Scripts/main.py: update them when main.py changes.
# ponytail: simple ball model, does not replace a test on the hardware.
import math, statistics

LENGTH = 400.0      # mm   rail length (assumed, not measured)
SENSOR = 30.0       # mm   sensor distance at p = 0
X_SET = 200         # mm   setpoint
X_MAX = 400         # mm   main.py: at or above this a reading counts as invalid
ALPHA, BETA = 0.3, 0.35


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def sign(x):
    return (x > 0) - (x < 0)


class World:
    def __init__(self, p0, rng, A=60.0, dead=1.5, roll=0.2, level=0.0, noise=2.0,
                 outlier=0.0, tau=0.04, rate=400.0, dt=0.001):
        self.p, self.v, self.th, self.u, self.t = p0, 0.0, 0.0, 0.0, 0.0
        self.rng, self.A, self.dead, self.roll, self.level = rng, A, dead, roll, level
        self.noise, self.outlier, self.tau, self.rate, self.dt = noise, outlier, tau, rate, dt
        self.hits = 0
        self.at_end = False

    def advance(self, T):
        for _ in range(int(round(T/self.dt))):
            dt = self.dt
            step = clamp((self.u - self.th)/self.tau*dt, -self.rate*dt, self.rate*dt)
            self.th += step
            ang = self.th - self.level
            if self.v == 0.0:
                if abs(ang) > self.dead/2:
                    self.v = self.A*(ang - sign(ang)*self.roll)*dt
            else:
                vn = self.v + self.A*(ang - sign(self.v)*self.roll)*dt
                if vn*self.v < 0 and abs(ang) < self.dead/2:
                    vn = 0.0
                self.v = vn
            self.p += self.v*dt
            if not 0 <= self.p <= LENGTH:
                self.p = clamp(self.p, 0.0, LENGTH)
                self.v = 0.0
                if not self.at_end:     # count arrivals, not pressing against the stop
                    self.hits += 1
                self.at_end = True
            elif 0 < self.p < LENGTH:
                self.at_end = False
        self.t += T

    def ping(self):
        self.advance(0.015)
        d = 8190 if self.rng.random() < self.outlier else self.p + SENSOR + self.rng.gauss(0, self.noise)
        self.advance(0.015)
        return d

    def e_true(self):
        return X_SET - (self.p + SENSOR)


class Holt:
    """measure() from main.py: an invalid reading keeps e on its trend."""

    def __init__(self, x):
        self.e, self.de = X_SET - min(x, X_MAX), 0.0

    def update(self, x, dt):
        e_raw = X_SET - min(x, X_MAX)
        e_old = self.e
        self.e = ALPHA*(e_raw if x < X_MAX else self.e) + (1 - ALPHA)*(self.e + self.de*dt)
        self.de = BETA*(self.e - e_old)/dt + (1 - BETA)*self.de
        return self.e, self.de


class PIDMain:
    """Inner and steady loop of Scripts/main.py, for comparison."""
    latency = 0.0

    def __init__(self):
        self.i = 0.0
        self.hold = False
        self.t_hold = 0.0
        self.steady = False

    def __call__(self, e, de, dt):
        TOL, V_TOL, T_STEADY = 7.5, 15, 3.0
        if self.steady:
            if abs(e) < 3*TOL:
                return 0.0
            self.steady = False
            self.t_hold = 0.0
        self.hold = abs(e) < TOL and abs(de) < (2*V_TOL if self.hold else V_TOL)
        if not self.hold:
            self.t_hold = 0.0
        else:
            self.t_hold += dt
            if self.t_hold > T_STEADY:
                self.steady = True
                return 0.0
        if self.hold:
            p = d = 0.0
        else:
            p = (0.10 + 0.05*min(abs(e)/50, 1))*e
            if abs(e) > TOL:
                p += 2 if e > 0 else -2
            self.i = clamp(self.i + 0.02*e*dt, -10, 10)
            d = 0.08*de
        return p + self.i + d


class Human:
    """Stand-in for a person on a potentiometer: slow reaction, inexact hand."""
    latency = 0.0

    def __init__(self, rng, delay=0.15):
        self.rng, self.n = rng, max(1, int(delay/0.03))
        self.buf = []
        self.hand = 0.0

    def __call__(self, e, de, dt):
        self.buf.append((e, de))
        e_d, de_d = self.buf.pop(0) if len(self.buf) > self.n else (0.0, 0.0)
        target = 0.06*e_d + 0.06*de_d
        if abs(e_d) < 5 and abs(de_d) < 12:
            target = 0.0
        target = clamp(target, -15, 15) + self.rng.gauss(0, 0.3)
        self.hand += 0.5*(target - self.hand)       # hand is not instant
        return self.hand


def run(ctrl, p0, seed, T_end=20.0, push_t=12.0, push_v=120.0, **world):
    import random
    rng = random.Random(seed)
    w = World(p0, rng, **world)
    f = Holt(w.ping())
    t_prev = w.t
    log = []
    pushed = False
    prev_u = 0.0
    travel = 0.0
    while w.t < T_end:
        if not pushed and w.t >= push_t:
            w.v += push_v
            pushed = True
        x = w.ping()
        dt = w.t - t_prev
        t_prev = w.t
        e, de = f.update(x, dt)
        if ctrl.latency:
            w.advance(ctrl.latency)
        u = ctrl(e, de, dt)
        w.u = clamp(u, -30, 30)
        travel += abs(w.u - prev_u)
        prev_u = w.u
        log.append((w.t, w.e_true(), w.v, w.u))
    return log, travel/w.t, w.hits


def metrics(log, travel, hits, push_t=12.0, band=7.5):
    before = [r for r in log if r[0] < push_t]
    after = [r for r in log if r[0] >= push_t]
    out1 = [r[0] for r in before if abs(r[1]) > band]
    settle = out1[-1] if out1 else 0.0
    settled = abs(before[-1][1]) <= band and settle < push_t - 1.0
    out2 = [r[0] for r in after if abs(r[1]) > band]
    recover = (out2[-1] - push_t) if out2 else 0.0
    recovered = abs(after[-1][1]) <= band and (not out2 or out2[-1] < log[-1][0] - 1.0)
    last = [abs(r[1]) for r in before[-100:]]
    return dict(settle=settle if settled else None, recover=recover if recovered else None,
                err=statistics.mean(last), travel=travel, hits=hits)


CASES = {
    "nominal": dict(),
    "A+33%": dict(A=80.0),
    "A-25%": dict(A=45.0),
    "schwer": dict(A=45.0, dead=3.0, roll=0.3, level=0.5, noise=3.0),
}


def curved(R, tilt_deg, shaped, roll=0.1, dead=1.5, T=25.0, dt=0.001, period_err=0.0):
    """Ball on arc of radius R (mm). Returns end position, target, last time outside
    +-7.5 mm, period. shaped: half the tilt first, the rest after T/2."""
    A = 122.0       # rail degrees, not servo degrees
    omega = math.sqrt(A*57.2958/R)
    period = 2*math.pi/omega
    x, v, t = 0.0, 0.0, 0.0
    target = R*math.radians(tilt_deg)
    last_out = 0.0
    while t < T:
        phi = tilt_deg if (not shaped or t >= period/2*(1 + period_err)) else tilt_deg/2
        ang = phi - math.degrees(x/R)
        if v == 0.0:
            if abs(ang) > dead/2:
                v = A*(ang - sign(ang)*roll)*dt
        else:
            vn = v + A*(ang - sign(v)*roll)*dt
            if vn*v < 0 and abs(ang) < dead/2:
                vn = 0.0
            v = vn
        x += v*dt
        t += dt
        if abs(x - target) > 7.5:
            last_out = t
    return x, target, last_out, period
```

### 7.2 check_docs.py

```python
# Runs the code blocks from the Ball_on_beam docs through the plant in sim_methods.py
# and checks that the result tables in the docs match.
#   python check_docs.py            check, fails if a table is out of date (under 1 min)
#   python check_docs.py --write    rewrite the result tables in the docs
#   python check_docs.py --q-seeds  also rerun the Q-learning seed comparison (about 5 min)
import contextlib, io, os, random, re, statistics, sys, time
from sim_methods import World, Holt, PIDMain, Human, run, metrics, CASES, X_SET, clamp, curved

DOCS = r"C:/Users/jan/OneDrive - ZHAW/Semester_Unterlagen/HS26/Physical_Computing/Ball_on_beam"
WRITE = "--write" in sys.argv
sys.stdout.reconfigure(encoding="utf-8")


def read(name):
    return open(os.path.join(DOCS, name), encoding="utf-8").read()


def block_with(text, needle):
    return next(b for b in re.findall(r"```python\n(.*?)```", text, re.S) if needle in b)


class Wrap:
    def __init__(self, f, latency):
        self.f, self.latency = f, latency

    def __call__(self, e, de, dt):
        return self.f(e, de, dt)


def controller(code, latency=0.0, setup=None):
    def make():
        ns = {}
        exec(code, ns)
        if setup:
            setup(ns)
        return Wrap(ns["regler"], latency)
    return make


def evaluate(make, **world):
    ms = []
    for p0 in (40, 100, 300, 360):
        for seed in range(5):
            log, tr, hits = run(make(), p0, seed, **world)
            ms.append(metrics(log, tr, hits))
    ok = [m["settle"] for m in ms if m["settle"] is not None]
    rec = [m["recover"] for m in ms if m["recover"] is not None]
    med = lambda xs: statistics.median(xs) if xs else None
    return dict(ok=len(ok), t=med(ok), rec=len(rec), t_rec=med(rec),
                err=med([m["err"] for m in ms]), servo=med([m["travel"] for m in ms]),
                hits=statistics.mean(m["hits"] for m in ms))


LABEL = {"nominal": "Normal", "A+33%": "A +33 %", "A-25%": "A −25 %", "schwer": "Schwer"}
HEADER = ("| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |\n"
          "| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |\n")


def row(name, r):
    t = "–" if r["t"] is None else "%.1f s" % r["t"]
    tr = "–" if r["t_rec"] is None else "%.1f s" % r["t_rec"]
    return "| %s | %d/20 | %s | %d/20 | %s | %.1f mm | %.0f °/s | %.1f |" % (
        name, r["ok"], t, r["rec"], tr, r["err"], r["servo"], r["hits"])


pid_row = row("PID aus `main.py`, Normal", evaluate(PIDMain, **CASES["nominal"]))
stale = []


def report(doc, make, extra=()):
    rows = [pid_row] + [row(n, r) for n, r in extra]
    rows += [row("**%s**" % LABEL[tag], evaluate(make, **w)) for tag, w in CASES.items()]
    table = HEADER + "\n".join(rows) + "\n"
    print("\n== " + doc + "\n" + "\n".join(rows))
    text = read(doc)
    new, n = re.subn(r"\| Fall \| Am Ziel .*?\n\n", lambda m: table + "\n", text, count=1, flags=re.S)
    assert n == 1, doc
    if new != text:
        stale.append(doc)
        if WRITE:
            open(os.path.join(DOCS, doc), "w", encoding="utf-8", newline="\n").write(new)


# Worked examples in the docs ------------------------------------------------
mpc_doc = read("Vorhersage-Regelung-MPC.md")
mpc = block_with(mpc_doc, "def regler")
ns = {}
exec(mpc, ns)
assert ns["regler"](30, 0, 0.03) == 2
for u, j in ((0, 4500), (2, 3600), (4, 4862), (8, 13867)):
    a = -ns["A"]*u
    v = a*ns["TIMES"][-1]
    cost = sum((30 + 0.5*a*t*t)**2 for t in ns["TIMES"]) + ns["W_V"]*v*v + ns["W_U"]*u*u
    assert round(cost) == j, (u, cost)

fz = read("Fuzzy-Regelung.md")
fuzzy = block_with(fz, "def regler")
ns = {}
exec(fuzzy, ns)
assert abs(ns["regler"](10, -25, .03) + 0.25) < 1e-9
assert abs(ns["regler"](20, 0, .03) - 3) < 1e-9
assert ns["regler"](0, 0, .03) == 0

w = World(0.0, random.Random(1), noise=0)
w.u = 5
while not 30.0 + 200 < w.ping() < 1000:
    pass
print("A-Messung im Modell (wahr 60): %.1f" % (2*200/(w.t*w.t*5)))

# Why A = 60: the tuned PID against the beam gain ----------------------------
print("PID gegen A:", {A: evaluate(PIDMain, A=float(A))["ok"] for A in (30, 60, 90, 122)})
print("PID mit 0.5 % Fehlmessungen:", evaluate(PIDMain, outlier=0.005))
print("MPC schwer, U_KICK 3/4/5:", [evaluate(controller(mpc.replace("U_KICK = 3 ", "U_KICK = %d " % k), latency=0.015),
                                             **CASES["schwer"])["ok"] for k in (3, 4, 5)])

# Robustness: Bang-Bang angle and the other controllers, count at target of 20 ---
bb = block_with(read("Bang-Bang-Regelung.md"), "def regler")
assert "U = 3 " in bb
WORLDS = list(CASES.values()) + [dict(A=45.0, dead=4.0, roll=0.3, level=0.5, noise=3.0), dict(A=122.0)]
VARIANTS = [("%s Grad" % u, controller(bb.replace("U = 3 ", "U = %s " % u))) for u in (2, 2.5, 3, 4, 10)]
VARIANTS += [("10 fern 4 nah", controller(bb.replace("    a = A*U*SAFETY\n", "    U = 10 if abs(e) > 30 else 4\n    a = A*U*SAFETY\n"))),
             ("MPC", controller(mpc, latency=0.015)), ("Fuzzy", controller(fuzzy, latency=0.003)), ("PID", PIDMain)]
print("Normal, A+33, A-25, Schwer, tot 4 Grad, A=122:")
for name, make in VARIANTS:
    print(" ", name, [evaluate(make, **w)["ok"] for w in WORLDS], flush=True)

# Noise of de for a resting ball
w = World(170.0, random.Random(1))
f = Holt(w.ping())
t_prev, vals = w.t, []
for i in range(3000):
    x = w.ping()
    dt = w.t - t_prev
    t_prev = w.t
    vals.append(f.update(x, dt)[1])
print("Rauschen de bei ruhendem Ball: %.1f mm/s, Spitze %.1f" % (statistics.stdev(vals[100:]), max(abs(v) for v in vals[100:])))

# Curved rail numbers --------------------------------------------------------
for R in (500, 1000, 2000):
    for shaped in (False, True):
        x, target, settle, period = curved(R, 3.0, shaped)
        print("R=%4d zweistufig=%-5s Ziel %5.1f Ende %5.1f im Bereich ab %5.1f s Periode %.2f s" % (R, shaped, target, x, settle, period))
    for err in (0.1, 0.2, -0.2):
        x, target, settle, period = curved(R, 3.0, True, period_err=err)
        print("   T falsch um %+.0f%%: Ende %5.1f im Bereich ab %5.1f s" % (err*100, x, settle))

# Controllers ----------------------------------------------------------------
report("Vorhersage-Regelung-MPC.md", controller(mpc, latency=0.015))
report("Bang-Bang-Regelung.md", controller(block_with(read("Bang-Bang-Regelung.md"), "def regler")))
report("Fuzzy-Regelung.md", controller(fuzzy, latency=0.003))
report("Golfer-Impuls-Regelung.md", controller(block_with(read("Golfer-Impuls-Regelung.md"), "def regler")))

# Imitation: synthetic person teaches the table for 4 minutes ------------------
imit = block_with(read("Imitations-Regelung.md"), "def regler")
base = {}
exec(imit, base)
rng = random.Random(5)
human = Human(rng)
t = 0.0
while t < 240:
    wd = World(rng.uniform(20, 380), rng)
    f = Holt(wd.ping())
    t_prev = wd.t
    while wd.t < 20:
        x = wd.ping()
        dt = wd.t - t_prev
        t_prev = wd.t
        e, de = f.update(x, dt)
        u = human(e, de, dt)
        wd.u = clamp(u, -30, 30)
        base["merke"](e, de, u)
    t += wd.t
tab = base["baue_tabelle"]()
print("\nImitation: leere Felder", sum(1 for r in base["anzahl"] for c in r if not c), "von 63")
human_row = ("Synth. Mensch, Normal", evaluate(lambda: Human(random.Random(3)), **CASES["nominal"]))
report("Imitations-Regelung.md", controller(imit, setup=lambda ns: ns.update(table=tab)), extra=[human_row])
print("Synth. Mensch A-25%:", evaluate(lambda: Human(random.Random(3)), **CASES["A-25%"])["ok"])

# Q-learning: the table printed in the doc -------------------------------------
q_doc = read("Q-Learning-Regelung.md")
pico = block_with(q_doc, "def regler")
report("Q-Learning-Regelung.md", controller(pico))

if "--q-seeds" in sys.argv:
    train = block_with(q_doc, "def trainiere")
    for label, rep in (("Standard", None), ("BONUS=3 PEN=0.05", ("GAMMA, PEN, BONUS = 0.97, 0.03, 1", "GAMMA, PEN, BONUS = 0.97, 0.05, 3"))):
        oks = []
        for seed in (1, 2, 3, 7, 11):
            code = train.replace("for seed in (1, 2, 3, 7, 11):", "for seed in (%d,):" % seed)
            if rep:
                assert rep[0] in code
                code = code.replace(*rep)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                exec(code, {})
            best = "\n".join(l for l in buf.getvalue().splitlines() if l.startswith("("))
            alt = re.sub(r"BEST = \(\n.*?\n\)\n", lambda m: "BEST = (\n" + best + "\n)\n", pico, flags=re.S)
            oks.append(evaluate(controller(alt), **CASES["nominal"])["ok"])
            print(label, "Seed", seed, "am Ziel", oks[-1], flush=True)
        print(label, "sortiert", sorted(oks), "Median", statistics.median(oks))

if stale and not WRITE:
    sys.exit("Tabellen veraltet: " + ", ".join(stale))
print("\nOK" if not stale else "\nTabellen neu geschrieben: " + ", ".join(stale))
```
