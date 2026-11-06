# Golfer-Regelung (Impuls-Regelung)

Der Regler regelt nicht laufend. Er wartet, bis der Ball ruht, misst die Distanz zum Ziel und gibt einen berechneten Stoss. Dann schaut er, wo der Ball liegt, und lernt daraus für den nächsten Schlag.

## Die Idee

Beim Putten korrigierst du den Ball nicht unterwegs. Du schätzt die Distanz, schlägst und schaust, wo er liegen bleibt. War der Schlag zu kurz, schlägst du beim nächsten Mal stärker. Genau so arbeitet dieser Regler.

## So funktioniert es

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

## Code für deine Schiene

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

## Einstellen

| Beobachtung                       | Massnahme                                        |
| --------------------------------- | ------------------------------------------------ |
| Schlag bewegt den Ball kaum       | `U` erhöhen, muss klar über dem toten Bereich liegen |
| Ball schiesst oft über das Ziel   | `U` senken, `T_MAX` senken                       |
| Regler startet nie einen Schlag   | `V_REST` erhöhen, liegt vermutlich unter dem Rauschen |
| Ball rollt nach dem Schlag davon  | `test_offset.py` neu laufen lassen, Schiene ist schief |

Wichtig: `V_REST` muss über dem Rauschen von `de` liegen. In `main.py` steht dazu `V_TOL = 15` als «knapp über Rauschen».

## Stärken und Schwächen

| Stärken                                      | Schwächen                                           |
| -------------------------------------------- | --------------------------------------------------- |
| Haftreibung ist Teil der Methode, kein Störfaktor | Langsam: pro Schlag rund 1–2 s                 |
| Lernt $A$ selbst, keine Messung nötig        | Eine schiefe Schiene (`OFFSET`) lässt den Ball davonrollen |
| Servo ist die meiste Zeit in Ruhe            | Reagiert während des Schlags nicht auf Störungen    |
| Gut sichtbar und gut erklärbar               | Kurze Schläge gehen in der Servo-Verzögerung unter  |

## Was die Simulation zeigt

**Test:** PC-Simulation, 20 Läufe aus 4 Startpositionen mit je 5 Zufallsfolgen. Nach 12 s bekommt der Ball einen Stoss von 120 mm/s. «Am Ziel» heisst: nach 12 s seit mindestens 1 s innerhalb ±7.5 mm. Zeiten sind Mediane der erfolgreichen Läufe. Restfehler ist die mittlere Abweichung in den 3 s vor dem Stoss, Servo-Weg die gesamte Winkeländerung pro Sekunde.

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| PID aus `main.py`, Normal | 16/20 | 6.6 s | 19/20 | 3.0 s | 4.0 mm | 27 °/s | 0.0 |
| **Normal** | 13/20 | 3.1 s | 9/20 | 5.2 s | 2.9 mm | 13 °/s | 0.1 |
| **A +33 %** | 9/20 | 3.7 s | 7/20 | 3.2 s | 8.5 mm | 16 °/s | 0.7 |
| **A −25 %** | 15/20 | 8.0 s | 12/20 | 4.9 s | 3.2 mm | 15 °/s | 0.0 |
| **Schwer** | 0/20 | – | 3/20 | 4.4 s | 63.3 mm | 14 °/s | 0.1 |

**Modell:** 30 ms pro Schleife, Servo-Verzögerung 40 ms, Sensorrauschen 2 mm, Holt-Filter wie in `main.py`, toter Bereich 1.5°, Rollreibung 0.2°.

**Fälle:** Im Fall «Normal» reagiert der Ball mit $A$ = 60 mm/s² pro Grad, so wie der Regler annimmt. «A +33 %» heisst real $A$ = 80, «A −25 %» real $A$ = 45. «Schwer» ist $A$ = 45, toter Bereich 3°, Rollreibung 0.3°, Schiene 0.5° schief und Rauschen 3 mm.

**Warum $A$ = 60 statt der theoretischen 122?** Dein PID aus `main.py` schwingt im Modell ab $A$ ≈ 90 stark, läuft aber bei 30 bis 60 sauber. Falls er bei dir stabil läuft, liegt dein echtes $A$ wohl deutlich unter 122. Dein PID ist auf die echte Schiene abgestimmt, die Methode hier auf das Modell. Der PID-Vergleich ist darum nur eine grobe Einordnung.

Das Modell schätzt nur grob ab, ob Vorzeichen und Logik stimmen. **Ein Test an der echten Schiene steht noch aus.** Im Fall «Schwer» liegt die Schiene 0.5° schief und der Ball rollt nach dem Schlag davon. Die Methode steht und fällt also mit gutem `OFFSET`. Sie blieb in der Simulation klar unzuverlässiger als MPC, Bang-Bang und Fuzzy.

## Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen. Ohne das hat diese Methode keine Chance.
2. Ball rund 100 mm neben das Ziel legen und einen einzelnen Schlag beobachten.
3. Prüfen, ob der Ball nach dem Schlag wirklich stehen bleibt. Wenn nicht: `V_BRAKE` senken.
4. Im Plotter verfolgen, ob sich $A$ über mehrere Schläge beruhigt.

## Verwandtschaft mit PID

Keine. Es gibt keinen laufenden Vergleich von Fehler und Winkel. Der Regler handelt nur selten und in fertigen Schlägen und passt zwischen den Schlägen sein Modell an. Fachlich liegt die Methode nahe bei *Iterative Learning Control* und *Event-triggered Control*.
