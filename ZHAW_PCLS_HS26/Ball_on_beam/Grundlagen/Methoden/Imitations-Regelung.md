# Imitations-Regelung

Du steuerst die Schiene ein paar Minuten von Hand mit einem Potentiometer. Der Pico merkt sich, was du in welcher Situation getan hast, und macht es danach allein nach. Fachlich heisst das *Imitation Learning* oder *Behavioral Cloning*.

## Die Idee

Ein Lehrling schaut einer erfahrenen Person bei der Arbeit zu und merkt sich: «In dieser Lage macht sie jenes.» Später braucht er keine Formel, sondern nur die Erinnerung. Hier ist die Erinnerung eine Tabelle.

## So funktioniert es

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

## Hardware

Du brauchst zusätzlich ein Potentiometer. `ADC(26)` entspricht GP26. Der Schleifer kommt an GP26, die beiden anderen Anschlüsse an 3V3 und GND. Prüfe die Belegung deines Boards oder Shields.

## Code für deine Schiene

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

## So machst du gut vor

- Lege den Ball an viele verschiedene Stellen, auch an die Schienenenden. Sonst bleiben diese Felder leer.
- Stosse den Ball auch mal an. Auch das Abfangen schneller Bälle muss in die Tabelle.
- Beginne mit kleinem `U_MAX`. Mit langsamerem Ball kannst du die Schiene besser führen.
- Ruhig bleiben. Hektische Korrekturen landen genauso in der Tabelle.

## Einstellen

| Beobachtung                         | Massnahme                                  |
| ----------------------------------- | ------------------------------------------ |
| Ball rollt in seltenen Lagen davon  | Mehr vormachen, genau diese Lagen üben     |
| Ball pendelt nahe am Ziel           | Sauberer vormachen, feinere Felder nahe null |
| Winkel springt hörbar               | Mit `weights()` aus [Fuzzy-Regelung.md](Fuzzy-Regelung.md) zwischen Feldern mischen |

Mit dieser Mischung wäre die Tabelle eine **Fuzzy-Tabelle, die aus deinen Daten gelernt ist**.

## Stärken und Schwächen

| Stärken                                        | Schwächen                                      |
| ---------------------------------------------- | ---------------------------------------------- |
| Kein Modell und keine Reglertheorie nötig      | Du musst die Aufgabe selbst können             |
| Starke Demo: Der Pico macht es wie du          | Kann in ungeübten Lagen versagen               |
| Mittelwert glättet zufälliges Zittern der Hand | Macht auch deine systematischen Fehler nach    |
| Tabelle passt in sehr wenig Speicher           | Poti und Lernphase nötig                       |

## Was die Simulation zeigt

Statt dir sitzt im Modell ein synthetischer «Mensch»: Er reagiert 150 ms verspätet, führt die Hand träge und zittert leicht. Er lernt 4 Minuten lang mit der Tabelle, danach übernimmt die Tabelle allein.

**Test:** PC-Simulation, 20 Läufe aus 4 Startpositionen mit je 5 Zufallsfolgen. Nach 12 s bekommt der Ball einen Stoss von 120 mm/s. «Am Ziel» heisst: nach 12 s seit mindestens 1 s innerhalb ±7.5 mm. Zeiten sind Mediane der erfolgreichen Läufe. Restfehler ist die mittlere Abweichung in den 3 s vor dem Stoss, Servo-Weg die gesamte Winkeländerung pro Sekunde.

| Fall | Am Ziel | Zeit | Nach Stoss wieder am Ziel | Zeit | Restfehler | Servo-Weg | Anschläge pro Lauf |
| ---- | ------- | ---- | ------------------------- | ---- | ---------- | --------- | ------------------ |
| PID aus `main.py`, Normal | 16/20 | 6.6 s | 19/20 | 3.0 s | 4.0 mm | 27 °/s | 0.0 |
| Synth. Mensch, Normal | 8/20 | 7.3 s | 0/20 | – | 7.0 mm | 20 °/s | 0.0 |
| **Normal** | 18/20 | 5.2 s | 15/20 | 4.9 s | 3.2 mm | 11 °/s | 0.0 |
| **A +33 %** | 20/20 | 3.6 s | 19/20 | 3.4 s | 3.6 mm | 10 °/s | 0.0 |
| **A −25 %** | 13/20 | 6.9 s | 6/20 | 4.5 s | 4.2 mm | 12 °/s | 0.0 |
| **Schwer** | 19/20 | 6.6 s | 18/20 | 4.1 s | 3.0 mm | 13 °/s | 0.0 |

**Modell:** 30 ms pro Schleife, Servo-Verzögerung 40 ms, Sensorrauschen 2 mm, Holt-Filter wie in `main.py`, toter Bereich 1.5°, Rollreibung 0.2°.

**Fälle:** Im Fall «Normal» reagiert der Ball mit $A$ = 60 mm/s² pro Grad, so wie der Regler annimmt. «A +33 %» heisst real $A$ = 80, «A −25 %» real $A$ = 45. «Schwer» ist $A$ = 45, toter Bereich 3°, Rollreibung 0.3°, Schiene 0.5° schief und Rauschen 3 mm.

**Warum $A$ = 60 statt der theoretischen 122?** Dein PID aus `main.py` schwingt im Modell ab $A$ ≈ 90 stark, läuft aber bei 30 bis 60 sauber. Falls er bei dir stabil läuft, liegt dein echtes $A$ wohl deutlich unter 122. Dein PID ist auf die echte Schiene abgestimmt, die Methode hier auf das Modell. Der PID-Vergleich ist darum nur eine grobe Einordnung.

**Die Zahlen sagen wenig über deine Hand aus.** Der synthetische Mensch ist nur eine Formel und ein Test an der echten Schiene steht noch aus. 10 von 63 Feldern blieben im Training leer.

Im Fall «Normal» schneidet die Tabelle besser ab als ihr Lehrer, vermutlich weil der Mittelwert das Handzittern herausrechnet. Das gilt nicht immer: Läuft die Schiene träger als beim Vormachen (Fall «A −25 %»), wird die Tabelle deutlich schlechter. Lehre deshalb an genau der Schiene, die der Regler später führt.

Ob du die Schiene von Hand überhaupt stabil hältst, zeigt erst der Versuch. Teste das, bevor du Zeit in den Rest steckst.

## Erster Test an der Schiene

1. Nur das Poti an `setAngle()` anschliessen und 2 Minuten üben. Gelingt das nicht, `U_MAX` senken. Bleibt es zu schwierig, passt die Methode nicht.
2. Vormachen, 5 Minuten.
3. Tabelle laden, Ball in Zielnähe legen, zuschauen.
4. Ball anstossen und schauen, ob die Tabelle fängt.

## Verwandtschaft mit PID

Keine in der Entstehung. Niemand wählt Faktoren oder Regeln, die Tabelle entsteht aus Beobachtung. Das Resultat ist aber wieder eine Abbildung von `e` und `de` auf einen Winkel. Das ist Physik: Jeder Regler für diesen Ball braucht diese zwei Grössen.
