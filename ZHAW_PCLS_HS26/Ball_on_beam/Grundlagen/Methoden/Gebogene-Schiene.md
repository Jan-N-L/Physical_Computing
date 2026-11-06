# Gebogene Schiene: Regelung durch Mechanik

Statt die Regelung zu verbessern, änderst du die Form: Die Schiene wird zur Mulde. Der Ball rollt von selbst zum tiefsten Punkt, und Kippen verschiebt diesen Punkt. Es braucht fast keinen Regler mehr.

## Die Idee

Eine Murmel in einer Salatschüssel findet den tiefsten Punkt ohne Elektronik. Auf der geraden Schiene ist der Ball instabil, denn er rollt davon, sobald sie nicht genau waagrecht liegt. In der Mulde ist der Ball **von Natur aus stabil**: Wird er gestört, rollt er zurück. Die Regelung steckt in der Geometrie.

## So funktioniert es

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

## Bauen

- **Schiene:** Alu-Profil biegen, Karton oder Holz in Bogenform als Unterlage, oder einen Bogen 3D-drucken.
- **Sensor:** Der Sensor muss den Ball weiterhin sehen. Bei 40 mm Durchhang kann die ansteigende Schiene hinten selbst in den Messkegel ragen und falsche Werte liefern. Setze ihn tief und prüfe es vor dem Umbau mit einer provisorischen Mulde.
- **Hebel:** $\varphi$ ist der Winkel der **Schiene**, nicht des Servos. Bestimme das Verhältnis `RATIO`: Servo um 10° drehen und die Schienenneigung messen.
- **Radius bestimmen:** Ball anstossen, 5 Perioden stoppen, dann $R = \tfrac{5}{7} g \cdot (T/2\pi)^2$.

## Code für deine Schiene

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

## Stärken und Schwächen

| Stärken                                        | Schwächen                                        |
| ---------------------------------------------- | ------------------------------------------------ |
| Fast kein Code, kein Tuning von Reglern        | Umbau der Hardware                               |
| Stabil auch bei Stromausfall oder Programmfehler | Genauigkeit durch Haftreibung begrenzt          |
| Ball kehrt nach Störung von selbst zurück      | Sensor könnte die Schiene statt des Balls sehen  |
| Auch ohne Sensor lauffähig                     | Pendeln nach Störung klingt nur langsam ab       |
| Überraschender Ansatz, gut für eine Demo       | **Trifft vermutlich nicht den Sinn einer Regelungsaufgabe** |

Falls das Projekt eine Regelung verlangt, kläre die Methode zuerst mit der Betreuung. Sie zeigt aber gut, warum die gerade Schiene überhaupt schwierig ist.

## Was die Simulation zeigt

Modell: Ball auf Kreisbogen, Haftreibung mit totem Bereich 1.5°, Rollreibung 0.1°, Kippen um 3° Schiene ohne äussere Störung. Es zeigt die Position nach 25 s und die Zeit, bis der Ball dauerhaft innerhalb ±7.5 mm um den idealen Tiefpunkt liegt.

| Radius  | Ziel     | In einem Schritt            | In zwei Stufen ($T/2$)       |
| ------- | -------- | --------------------------- | ---------------------------- |
| 500 mm  | 26.2 mm  | Ende 20.9 mm, nach 8.6 s    | Ende 24.4 mm, nach 0.6 s     |
| 1000 mm | 52.4 mm  | Ende 41.9 mm, nie im Bereich | Ende 48.9 mm, nach 1.0 s    |
| 2000 mm | 104.7 mm | Ende 83.8 mm, nie im Bereich | Ende 97.7 mm, nach 1.6 s    |

Bei $R$ = 500 mm kommt der Ball mit zwei Stufen rund 14-mal schneller zur Ruhe und näher ans Ziel. Bei grösserem Radius erreicht der einstufige Ablauf den Bereich gar nicht.

**Empfindlichkeit:** Ist $T$ um 10 % falsch, hält $R$ = 500 mm den Bereich noch (Ende 22.5 mm), $R$ = 2000 mm nicht mehr. Bei 20 % fällt auch $R$ = 1000 mm heraus. Miss $T$ also sauber.

Das Modell schätzt nur grob ab, ob die Physik aufgeht. **Es wurde nichts gebaut und nichts an echter Hardware getestet.**

## Erster Test an der Schiene

1. Vor dem Umbau: Biege ein Stück Karton als Mulde und lege den Ball hinein. Bleibt er genau genug liegen?
2. Sensor provisorisch montieren: Sieht er den Ball auch am Muldenende, und misst er ohne Ball nicht die Schiene?
3. Erst dann die Schiene bauen, $R$, $T$ und `RATIO` messen.

## Verwandtschaft mit PID

Keine. Es läuft keine Rückführung, also auch kein Regelkreis. Willst du die Reibungsabweichung loswerden, kannst du nach dem Stillstand messen und $\varphi$ um $\Delta x / R$ nachstellen. Das ist dann wieder eine langsame Rückführung, im Wesen ein kleiner I-Anteil.
