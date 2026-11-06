# Vorhersage-Regelung (MPC)

Der Pico spielt vor jedem Schritt mehrere Neigungswinkel im Kopf durch und wählt den Winkel, der den Ball laut Physik am besten ans Ziel bringt. MPC steht für *Model Predictive Control*. Hier steht eine stark vereinfachte Version.

## Die Idee

Beim Autofahren fragst du nicht nur, wie weit du gerade von der Spurmitte entfernt bist. Du schätzt, wo du in zwei Sekunden bist, wenn du so weiterlenkst, und korrigierst schon jetzt. Genau das macht dieser Regler: Er schaut nicht auf den Fehler, sondern auf die vorhergesagte Zukunft.

## So funktioniert es

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

## Code für deine Schiene

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

**Einbau:** `measure()`, `setAngle()` und die Initialisierung bleiben aus `main.py`. Die Regelschleife am Ende wird ersetzt durch:

```python
while True:
    e_raw, dt = measure()
    u = regler(e, de, dt)
    setAngle(u)
    print("%.1f, %.1f" % (e, u))
```

## A messen

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

## Einstellen

| Beobachtung                       | Massnahme                               |
| --------------------------------- | --------------------------------------- |
| Ball schwingt um das Ziel         | `W_V` erhöhen, `A` erhöhen              |
| Ball kommt nur langsam ans Ziel   | `W_V` senken                            |
| Winkel springt nervös hin und her | `W_U` erhöhen, grössere Winkel streichen |
| Ball klebt knapp neben dem Ziel   | `U_KICK` erhöhen                        |
| Ball wird vom Kick weggestossen   | `U_KICK` senken, `T_STUCK` erhöhen      |

Miss auch die Rechenzeit: `ticks_us()` vor und nach `regler()`. Zählt sie mehr als rund 15 ms, nimm weniger Winkel oder weniger Zeitpunkte. Als `U_KICK` passt ungefähr der halbe «tote Bereich» aus `test_offset.py` plus 1°.

## Stärken und Schwächen

| Stärken                                          | Schwächen                                          |
| ------------------------------------------------ | -------------------------------------------------- |
| Faktoren folgen aus Physik statt aus Ausprobieren | Braucht ein gemessenes $A$                         |
| Begrenzte Winkel sind von Anfang an eingebaut    | Rechenaufwand auf dem Pico                         |
| Ruhiger Servo                                    | Modell kennt keine Reibung, darum der Kick         |
| Leicht erweiterbar: Hindernis, Zielbahn          | Kein I-Anteil: `OFFSET` muss stimmen         |

## Was die Simulation zeigt

**Test:** PC-Simulation, 20 Läufe aus 4 Startpositionen mit je 5 Zufallsfolgen. Nach 12 s bekommt der Ball einen Stoss von 120 mm/s. «Am Ziel» heisst: nach 12 s seit mindestens 1 s innerhalb ±7.5 mm. Zeiten sind Mediane der erfolgreichen Läufe. Restfehler ist die mittlere Abweichung in den 3 s vor dem Stoss, Servo-Weg die gesamte Winkeländerung pro Sekunde.

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| PID aus `main.py`, Normal | 16/20 | 6.6 s | 19/20 | 3.0 s | 4.0 mm | 27 °/s | 0.0 |
| **Normal** | 20/20 | 2.1 s | 20/20 | 1.0 s | 1.2 mm | 5 °/s | 0.0 |
| **A +33 %** | 18/20 | 2.8 s | 16/20 | 4.0 s | 2.7 mm | 14 °/s | 0.0 |
| **A −25 %** | 20/20 | 1.9 s | 20/20 | 1.8 s | 1.9 mm | 4 °/s | 0.0 |
| **Schwer** | 8/20 | 3.6 s | 18/20 | 1.1 s | 7.4 mm | 9 °/s | 0.0 |

**Modell:** 30 ms pro Schleife, Servo-Verzögerung 40 ms, Sensorrauschen 2 mm, Holt-Filter wie in `main.py`, toter Bereich 1.5°, Rollreibung 0.2°.

**Fälle:** Im Fall «Normal» reagiert der Ball mit $A$ = 60 mm/s² pro Grad, so wie der Regler annimmt. «A +33 %» heisst real $A$ = 80, «A −25 %» real $A$ = 45. «Schwer» ist $A$ = 45, toter Bereich 3°, Rollreibung 0.3°, Schiene 0.5° schief und Rauschen 3 mm.

**Warum $A$ = 60 statt der theoretischen 122?** Dein PID aus `main.py` schwingt im Modell ab $A$ ≈ 90 stark, läuft aber bei 30 bis 60 sauber. Falls er bei dir stabil läuft, liegt dein echtes $A$ wohl deutlich unter 122. Dein PID ist auf die echte Schiene abgestimmt, die Methode hier auf das Modell. Der PID-Vergleich ist darum nur eine grobe Einordnung.

Im Fall «Schwer» bleibt der Ball oft knapp am Rand des Sollbereichs (Restfehler 7.4 mm). `U_KICK = 4` hob den Wert im Modell nur auf 11/20.

Das Modell schätzt nur grob ab, ob Vorzeichen und Logik stimmen. **Ein Test an der echten Schiene steht noch aus.** Rechenzeit ist mit 15 ms angenommen, nicht gemessen.

## Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. $A$ messen und eintragen.
3. Nur `print("%.1f, %.1f" % (e, regler(e, de, dt)))` laufen lassen, Servo noch aus: Ball von Hand verschieben, Vorzeichen prüfen. Bei `e > 0` muss der Winkel positiv sein.
4. Servo aktivieren, Ball in Zielnähe legen, danach weiter weg.

## Verwandtschaft mit PID

Es gibt keine Kp-, Ki- und Kd-Werte und kein Fehlerintegral. Ganz ohne Verwandtschaft ist die Methode aber nicht: Mit unbegrenzten Winkeln ergäbe die Optimierung rechnerisch $u = k_1 e + k_2 \dot e$, also eine PD-Form. Der echte Unterschied liegt im Entwurf und bei grossen Fehlern: Die Faktoren entstehen aus Physik und Gewichten, und die Kandidatenliste begrenzt das Verhalten ausdrücklich.
