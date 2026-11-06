# Fuzzy-Regelung

Du schreibst dein Wissen als Regeln in Worten auf, etwa «Ball weit weg und ruhig: stark kippen». Der Regler prüft, wie stark jede Regel gerade zutrifft, und mischt die Antworten.

## Die Idee

Beim Duschen rechnest du nicht. Du denkst: «etwas zu kalt, also ein wenig wärmer». «Etwas» und «ein wenig» sind unscharf, englisch *fuzzy*. Fuzzy-Regelung macht solche Aussagen rechenbar. 22 °C ist dann zum Beispiel 70 % «kühl» und 30 % «angenehm», statt hart in eine Schublade zu fallen.

## So funktioniert es

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

## Code für deine Schiene

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

**Einbau:** `measure()`, `setAngle()` und die Initialisierung bleiben aus `main.py`. Die Regelschleife am Ende wird ersetzt durch:

```python
while True:
    e_raw, dt = measure()
    u = regler(e, de, dt)
    setAngle(u)
    print("%.1f, %.1f" % (e, u))
```

## Einstellen

Du änderst nicht Faktoren, sondern einzelne Regeln. Das Plotter-Bild zeigt dir, in welcher Situation der Ball falsch reagiert.

| Beobachtung                  | Massnahme                                                 |
| ---------------------------- | --------------------------------------------------------- |
| Ball pendelt langsam und weit um das Ziel | Winkel bei `e` = 0 und `de` = ±50 vergrössern |
| Ball klebt knapp neben Ziel  | Winkel bei `e` = ±20 und `de` = 0 erhöhen                 |
| Schnelles Zittern in Zielnähe | Winkel bei `e` = 0 und `de` = ±50 verkleinern            |
| Ball langsam aus der Ferne   | Winkel bei `e` = ±60 und `de` = 0 erhöhen                 |
| Feinheit nahe am Ziel fehlt  | Stützpunkt bei ±8 mm ergänzen, Tabelle um Zeilen erweitern |

Halte die Tabelle **punktsymmetrisch**: Das Feld (−20, −50) ist das Negative von (+20, +50). So kannst du jede Änderung gleich gespiegelt eintragen.

## Stärken und Schwächen

| Stärken                                        | Schwächen                                             |
| ---------------------------------------------- | ----------------------------------------------------- |
| Kein Modell nötig                              | Viele Zahlen in der Tabelle                           |
| Regeln lassen sich lesen und erklären          | Kein systematischer Weg zu guten Regeln               |
| Gezielte Änderung für genau eine Situation     | Bei grösserer Tabelle wächst der Rechenaufwand schnell |
| Wenig Rechenzeit bei 5×5                       | Kein I-Anteil: `OFFSET` muss stimmen            |

## Was die Simulation zeigt

**Test:** PC-Simulation, 20 Läufe aus 4 Startpositionen mit je 5 Zufallsfolgen. Nach 12 s bekommt der Ball einen Stoss von 120 mm/s. «Am Ziel» heisst: nach 12 s seit mindestens 1 s innerhalb ±7.5 mm. Zeiten sind Mediane der erfolgreichen Läufe. Restfehler ist die mittlere Abweichung in den 3 s vor dem Stoss, Servo-Weg die gesamte Winkeländerung pro Sekunde.

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| PID aus `main.py`, Normal | 16/20 | 6.6 s | 19/20 | 3.0 s | 4.0 mm | 27 °/s | 0.0 |
| **Normal** | 20/20 | 1.6 s | 20/20 | 1.3 s | 1.0 mm | 21 °/s | 0.0 |
| **A +33 %** | 0/20 | – | 0/20 | – | 13.8 mm | 67 °/s | 0.0 |
| **A −25 %** | 20/20 | 1.3 s | 20/20 | 0.8 s | 0.6 mm | 19 °/s | 0.0 |
| **Schwer** | 20/20 | 1.5 s | 20/20 | 0.7 s | 3.3 mm | 27 °/s | 0.0 |

**Modell:** 30 ms pro Schleife, Servo-Verzögerung 40 ms, Sensorrauschen 2 mm, Holt-Filter wie in `main.py`, toter Bereich 1.5°, Rollreibung 0.2°.

**Fälle:** Im Fall «Normal» reagiert der Ball mit $A$ = 60 mm/s² pro Grad, so wie der Regler annimmt. «A +33 %» heisst real $A$ = 80, «A −25 %» real $A$ = 45. «Schwer» ist $A$ = 45, toter Bereich 3°, Rollreibung 0.3°, Schiene 0.5° schief und Rauschen 3 mm.

**Warum $A$ = 60 statt der theoretischen 122?** Dein PID aus `main.py` schwingt im Modell ab $A$ ≈ 90 stark, läuft aber bei 30 bis 60 sauber. Falls er bei dir stabil läuft, liegt dein echtes $A$ wohl deutlich unter 122. Dein PID ist auf die echte Schiene abgestimmt, die Methode hier auf das Modell. Der PID-Vergleich ist darum nur eine grobe Einordnung.

Das Modell schätzt nur grob ab, ob Vorzeichen und Logik stimmen. **Ein Test an der echten Schiene steht noch aus.** Die guten Werte im Fall «Normal» zeigen nicht, dass Fuzzy allgemein besser ist als PID. Der Fall «A +33 %» zeigt die Schwäche: Reagiert der Ball deutlich stärker als angenommen, schwingt die Tabelle dauerhaft. Dann verkleinerst du die Einträge, statt den Code zu ändern. Die Rechenzeit ist mit 3 ms angenommen, nicht gemessen.

## Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. Servo noch aus: nur `print(e, de, regler(e, de, dt))`, Ball von Hand verschieben, Vorzeichen prüfen.
3. Alle Tabellenwerte halbieren, Servo aktivieren, Ball in Zielnähe legen.
4. Tabellenwerte schrittweise erhöhen, bis der Ball flott und ohne Schwingen ankommt.

## Verwandtschaft mit PID

Hier ist die Nähe am grössten. Rechnerisch ist diese Tabelle ein **nichtlinearer PD-Regler**, denn sie bildet genau `e` und `de` auf einen Winkel ab. Die Tabellenwerte in der Nähe des Ziels entsprechen sogar grob $K_p \approx 0.15$ und $K_d \approx 0.08$. Neu ist die Denkweise: Du beschreibst Situationen in Worten, statt Faktoren zu tunen, und kannst jede Situation einzeln behandeln.
