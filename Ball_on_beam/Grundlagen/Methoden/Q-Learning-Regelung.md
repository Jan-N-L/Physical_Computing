# Q-Learning-Regelung

Niemand sagt dem Regler, wie er kippen soll. Er probiert in einer Simulation tausende Male aus, bekommt Belohnung und Strafe und lernt daraus eine Tabelle. Q-Learning ist eine Methode des *Reinforcement Learning* (bestärkendes Lernen).

## Die Idee

Ein Hund lernt Sitz, ohne dass ihm jemand die Muskeln erklärt: Macht er das Richtige, gibt es ein Leckerli. Mit der Zeit merkt er sich, welche Handlung in welcher Situation am meisten bringt. Der Q-Learning-Regler arbeitet genauso, nur gibt es statt Leckerli eine Punktzahl.

## So funktioniert es

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

## Training am PC

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

## Code für den Pico

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

**Einbau:** `measure()`, `setAngle()` und die Initialisierung bleiben aus `main.py`. Die Regelschleife am Ende wird ersetzt durch:

```python
while True:
    e_raw, dt = measure()
    u = regler(e, de, dt)
    setAngle(u)
    print("%.1f, %.1f" % (e, u))
```

## Einstellen

Du änderst nicht den Regler, sondern Belohnung und Modell, und trainierst neu.

| Beobachtung                    | Massnahme                                                    |
| ------------------------------ | ------------------------------------------------------------ |
| Ball bleibt knapp neben Ziel   | Feinere Felder nahe null, `BONUS` vorsichtig erhöhen         |
| Servo zittert                  | `PEN` erhöhen, `SKIP` erhöhen                                |
| Regler ist zu faul und kippt kaum | `PEN` senken                                              |
| Läuft im Modell, aber nicht echt | $A$, `DEAD` und `ROLL` im Modell auf gemessene Werte setzen |

**Vorsicht beim Vergleichen:** Beurteile eine Änderung nie anhand eines einzelnen Trainings. Mit `BONUS = 3` und `PEN = 0.05` lag «Am Ziel» über dieselben fünf Trainings zwischen 0 und 11 von 20, Median 8. Das ist kein klarer Fortschritt gegenüber der Standardeinstellung mit Median 5.

Tipp für später: Variiere $A$ und `DEAD` bei jeder Episode zufällig. Der Fachbegriff dafür ist *Domain Randomization*. Der Regler wird dadurch unempfindlicher gegen Modellfehler.

## Stärken und Schwächen

| Stärken                                          | Schwächen                                          |
| ------------------------------------------------ | -------------------------------------------------- |
| Niemand muss die Lösung vorgeben                 | Ergebnis schwankt stark von Training zu Training   |
| Auf dem Pico extrem wenig Aufwand und Speicher   | Grobe Tabelle: feste Winkel, Sprünge               |
| Lernt auch ungewohnte Strategien                 | Modellfehler landen direkt in der Tabelle          |
| Starker Einstieg in Machine Learning             | Belohnung tunen ist heikel und langwierig          |

## Was die Simulation zeigt

**Test:** PC-Simulation, 20 Läufe aus 4 Startpositionen mit je 5 Zufallsfolgen. Nach 12 s bekommt der Ball einen Stoss von 120 mm/s. «Am Ziel» heisst: nach 12 s seit mindestens 1 s innerhalb ±7.5 mm. Zeiten sind Mediane der erfolgreichen Läufe. Restfehler ist die mittlere Abweichung in den 3 s vor dem Stoss, Servo-Weg die gesamte Winkeländerung pro Sekunde.

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| PID aus `main.py`, Normal | 16/20 | 6.6 s | 19/20 | 3.0 s | 4.0 mm | 27 °/s | 0.0 |
| **Normal** | 15/20 | 4.8 s | 14/20 | 5.4 s | 1.9 mm | 24 °/s | 0.0 |
| **A +33 %** | 5/20 | 9.1 s | 5/20 | 6.1 s | 11.1 mm | 46 °/s | 0.0 |
| **A −25 %** | 18/20 | 7.6 s | 17/20 | 4.2 s | 2.2 mm | 19 °/s | 0.0 |
| **Schwer** | 7/20 | 8.8 s | 2/20 | 3.8 s | 5.4 mm | 28 °/s | 0.0 |

**Modell:** 30 ms pro Schleife, Servo-Verzögerung 40 ms, Sensorrauschen 2 mm, Holt-Filter wie in `main.py`, toter Bereich 1.5°, Rollreibung 0.2°.

**Fälle:** Im Fall «Normal» reagiert der Ball mit $A$ = 60 mm/s² pro Grad, so wie der Regler annimmt. «A +33 %» heisst real $A$ = 80, «A −25 %» real $A$ = 45. «Schwer» ist $A$ = 45, toter Bereich 3°, Rollreibung 0.3°, Schiene 0.5° schief und Rauschen 3 mm.

**Warum $A$ = 60 statt der theoretischen 122?** Dein PID aus `main.py` schwingt im Modell ab $A$ ≈ 90 stark, läuft aber bei 30 bis 60 sauber. Falls er bei dir stabil läuft, liegt dein echtes $A$ wohl deutlich unter 122. Dein PID ist auf die echte Schiene abgestimmt, die Methode hier auf das Modell. Der PID-Vergleich ist darum nur eine grobe Einordnung.

**Das ist die Tabelle, die der Test im Skript aus fünf Trainings wählte, nicht der Normalfall.** Im Fall «Normal» erreichten die fünf Tabellen 4, 4, 5, 15 und 17 von 20 Läufen. Der Skript-Test wählte die mit 15. Rechne also immer mit mehreren Trainings. Die Auswahl geschieht im Modell und ist deshalb etwas zu optimistisch.

Auch diese Zahlen stammen aus dem Modell, **ein Test an der echten Schiene steht noch aus**. Dort dürfte es eher schlechter laufen, weil die Tabelle ihr Modell für die Wirklichkeit hält.

## Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. $A$ messen wie in [Vorhersage-Regelung-MPC.md](Vorhersage-Regelung-MPC.md), neu trainieren.
3. Tabelle auf den Pico, Ball in Zielnähe legen, zuschauen.
4. Weicht das Verhalten stark ab, das Modell im Training an die echte Schiene angleichen.

## Verwandtschaft mit PID

Keine in der Herkunft: Hier bestimmt ein Lernverfahren die Tabelle, weder Faktoren noch Regeln stammen von dir. Wie jeder Regler hier bildet aber auch sie `e` und `de` auf einen Winkel ab.
