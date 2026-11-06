# Bang-Bang-Regelung mit Schaltkurve

Die Schiene kennt nur drei Zustände: kippen, gegenkippen, waagrecht. Eine einfache Formel aus der Physik entscheidet, wann gebremst wird.

## Die Idee

Du fährst auf ein Rotlicht zu. Am schnellsten bist du, wenn du Vollgas gibst und im letzten möglichen Moment voll bremst. Dafür musst du nur eines wissen: Wie lang ist mein Bremsweg? Solange die Strecke zum Ziel länger ist als der Bremsweg, gibst du Gas. Sonst bremst du.

## So funktioniert es

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

## Code für deine Schiene

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

**Einbau:** `measure()`, `setAngle()` und die Initialisierung bleiben aus `main.py`. Die Regelschleife am Ende wird ersetzt durch:

```python
while True:
    e_raw, dt = measure()
    u = regler(e, de, dt)
    setAngle(u)
    print("%.1f, %.1f" % (e, u))
```

## Einstellen

| Beobachtung                    | Massnahme                                 |
| ------------------------------ | ----------------------------------------- |
| Ball pendelt um das Ziel       | `U` senken, danach `SAFETY` senken oder `T_LEAD` erhöhen |
| Ball kommt nur langsam ans Ziel | `U` in kleinen Schritten erhöhen         |
| Ball bleibt liegen, obwohl die Schiene kippt | `U` erhöhen: muss über halbem totem Bereich aus `test_offset.py` plus Schieflage liegen |
| Zittern um das Ziel            | `TOL` und `V_TOL` erhöhen                 |

$A$ misst du wie in [Vorhersage-Regelung-MPC.md](Vorhersage-Regelung-MPC.md) beschrieben.

## Stärken und Schwächen

| Stärken                                 | Schwächen                                     |
| --------------------------------------- | --------------------------------------------- |
| Sehr wenig Code, kaum Rechenzeit        | Servo wechselt dauernd die Richtung: Verschleiss und Lärm |
| Physik gut sichtbar: Bremsweg als Formel | `U` muss zwischen Haftgrenze und Pendeln liegen |
| Im Modell schnell und robust gegen Reibung | Immer gleich starkes Kippen, kein sanftes Anfahren |
| Haftreibung stört kaum, solange `U` gross genug ist | Reagiert der Ball viel stärker als angenommen, wird es unruhig |

## Was die Simulation zeigt

**Test:** PC-Simulation, 20 Läufe aus 4 Startpositionen mit je 5 Zufallsfolgen. Nach 12 s bekommt der Ball einen Stoss von 120 mm/s. «Am Ziel» heisst: nach 12 s seit mindestens 1 s innerhalb ±7.5 mm. Zeiten sind Mediane der erfolgreichen Läufe. Restfehler ist die mittlere Abweichung in den 3 s vor dem Stoss, Servo-Weg die gesamte Winkeländerung pro Sekunde.

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| PID aus `main.py`, Normal | 16/20 | 6.6 s | 19/20 | 3.0 s | 4.0 mm | 27 °/s | 0.0 |
| **Normal** | 20/20 | 1.9 s | 20/20 | 1.7 s | 1.4 mm | 21 °/s | 0.0 |
| **A +33 %** | 12/20 | 4.0 s | 19/20 | 1.6 s | 2.3 mm | 21 °/s | 0.0 |
| **A −25 %** | 20/20 | 2.0 s | 20/20 | 2.0 s | 1.5 mm | 18 °/s | 0.0 |
| **Schwer** | 19/20 | 7.1 s | 19/20 | 2.7 s | 3.6 mm | 26 °/s | 0.0 |

**Modell:** 30 ms pro Schleife, Servo-Verzögerung 40 ms, Sensorrauschen 2 mm, Holt-Filter wie in `main.py`, toter Bereich 1.5°, Rollreibung 0.2°.

**Fälle:** Im Fall «Normal» reagiert der Ball mit $A$ = 60 mm/s² pro Grad, so wie der Regler annimmt. «A +33 %» heisst real $A$ = 80, «A −25 %» real $A$ = 45. «Schwer» ist $A$ = 45, toter Bereich 3°, Rollreibung 0.3°, Schiene 0.5° schief und Rauschen 3 mm.

**Warum $A$ = 60 statt der theoretischen 122?** Dein PID aus `main.py` schwingt im Modell ab $A$ ≈ 90 stark, läuft aber bei 30 bis 60 sauber. Falls er bei dir stabil läuft, liegt dein echtes $A$ wohl deutlich unter 122. Dein PID ist auf die echte Schiene abgestimmt, die Methode hier auf das Modell. Der PID-Vergleich ist darum nur eine grobe Einordnung.

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

## Erster Test an der Schiene

1. `test_offset.py` laufen lassen, `OFFSET` eintragen.
2. $A$ messen und eintragen.
3. Mit `U = 2` beginnen und erhöhen, bis der Ball zuverlässig losrollt. Schiene und Servo gut festhalten.
4. Ball nah am Ziel starten, danach weiter weg.

## Verwandtschaft mit PID

Bang-Bang ist kaum verwandt mit PID: Der Winkel wächst nicht mit dem Fehler, sondern springt zwischen festen Werten. Es gibt keine Verstärkung, sondern nur die Frage «Gas oder Bremse?». Wer die Schaltgrösse auf die Form $s = e + \lambda \dot e$ vereinfacht, landet bei der **Sliding-Mode-Regelung**, einem verwandten eigenen Forschungsgebiet.
