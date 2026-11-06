# Kreative Ansätze für Ball on Beam

**Das sind ungetestete Überlegungen.** Nichts davon ist gebaut oder simuliert. Die Zahlen sind Überschlagsrechnungen mit angenommenen Werten, die Einschätzungen sind meine Meinung, Fachbegriffe und historische Angaben stammen aus dem Gedächtnis und sind nicht nachgeprüft. Jede Idee nennt darum einen kleinen Versuch, der sie früh widerlegen kann.

Die anderen Dokumente tauschen den Regler aus. Hier steht eine andere Frage im Zentrum: **Was, wenn der Regler gar nicht das Hauptproblem ist?**

## Wo die Ideen ansetzen

Die Simulation der Regler deutet auf drei Schwachstellen hin. Sie sind nur im Modell beobachtet.

| Schwachstelle | Was im Modell passiert |
| ------------- | ---------------------- |
| **Verzögerung** | Sensor, Filter und Servo reagieren spät. Wahrscheinlich schwingen PID und Fuzzy deshalb, sobald der Ball stärker reagiert als angenommen. |
| **Haftreibung** | Der Ball bleibt knapp neben dem Ziel liegen. Dagegen arbeiten heute `FC` in `main.py` und der Kick im MPC. |
| **Unbekannte Schiene** | $A$, Schiefstand und Unebenheiten sind nur ungefähr bekannt, und die Ergebnisse hängen stark davon ab. |

## Überblick

| Nr. | Idee | Setzt an bei | Aufwand | Potenzial |
| --- | ---- | ------------ | ------- | --------- |
| 1 | Rastkerbe: die digitale Schiene | Haftreibung | sehr klein | hoch |
| 2 | Rüttelmotor gegen Haftreibung | Haftreibung | klein | hoch |
| 3 | Absichtlich pendeln | Haftreibung | sehr klein | mittel |
| 4 | Ball als Schleifer eines Potentiometers | Verzögerung | mittel | sehr hoch |
| 5 | Der Regler hört den Ball | Verzögerung, Haftreibung | klein | mittel |
| 6 | Beobachter schätzt die Störung live | Unbekannte Schiene | mittel | hoch |
| 7 | Wirbelstrom als eingebaute Dämpfung | Verzögerung | mittel | mittel, riskant |
| 8 | Landkarte der Schiene | Unbekannte Schiene | klein | hängt von der Schiene ab |
| 9 | Der Pico lernt seine eigene Physik | Unbekannte Schiene, Haftreibung | mittel | hoch |
| 10 | Die Schiene optimiert sich selbst | Unbekannte Schiene | mittel | hoch |
| 11 | Der Regler als Messgerät | neuer Zweck | klein bis gross | hoch als Projektidee |
| 12 | Motorik-Trainer mit Stützrädern | neuer Zweck | mittel | hoch als Projektidee |
| 13 | Ball-Telefon | neuer Zweck | mittel | mittel, starke Demo |
| 14 | Wellen-Schiene | alles zugleich | gross | unklar, sehr originell |

**Meine Empfehlung:**

1. **Heute:** Nr. 1 mit zwei Stücken Klebeband. Das dauert Minuten und zeigt, wie viel simple Mechanik bringt.
2. **Grösster Hebel:** Nr. 4. Sie verbessert die Messung, und davon profitiert jeder Regler.
3. **Ohne neue Hardware:** Nr. 10 oder 9. Beide sehen beeindruckend aus und passen sich der echten Schiene an statt einem Modell.

---

## A. Reibung nutzen statt bekämpfen

### 1. Rastkerbe: die digitale Schiene

**Kerngedanke:** Eine winzige Vertiefung im Sollpunkt fängt den Ball. Der Regler muss ihn nicht mehr millimetergenau parkieren, sondern nur langsam genug ankommen lassen. Aus einer stetigen Aufgabe wird eine fast digitale: «zur nächsten Kerbe».

**Theorie:** Ein rollender Ball trägt die Energie $\tfrac{7}{10} m v^2$. Aus einer Kerbe der Tiefe $h$ kommt er nur heraus, wenn diese Energie $m g h$ übersteigt:

$$
v_{\max} = \sqrt{\tfrac{10}{7} g h}
$$

Bei $h$ = 0.1 mm sind das rund 37 mm/s, bei 0.5 mm rund 84 mm/s. Schon eine Klebeband-Dicke fängt also einen langsamen Ball.

Halten ist noch stärker: Um aus einer Stufe zu rollen, braucht ein Ball mit Radius $r$ ungefähr $\tan\alpha \approx \sqrt{2h/r}$. Bei $r$ = 10 mm und $h$ = 0.1 mm wären das rund 8° Schienenneigung. Innerhalb dieses Winkels bleibt der Ball liegen, auch wenn die Schiene etwas schief steht.

**Haken:** Sollwerte gibt es nur noch an den Kerben. Zum Weiterrollen braucht es einen kräftigen Stoss. Vermutlich gilt das auch nicht mehr als «Regelung» im Sinn der Aufgabe, also vorher klären.

**Kleinster Versuch:** Zwei Streifen Klebeband mit 4 mm Lücke quer auf die Schiene kleben, Sollwert in die Lücke legen, `main.py` starten. Rollt der Ball darüber hinweg, wird er eingefangen?

### 2. Rüttelmotor gegen Haftreibung

**Kerngedanke:** Nicht gegen die Reibung anregeln, sondern sie wegschütteln. Ein kleiner Vibrationsmotor wie im Handy hält den Ball in Bewegung, sodass er auf kleinste Neigungen reagiert. In der Industrie heisst das Prinzip *Dither*, es ist etwa bei Ventilen bekannt.

**Theorie:** Die Haftgrenze entspricht hier rund $122 \cdot 0.75 \approx 92$ mm/s², also etwa 1 % der Erdbeschleunigung. Eine Schwingung mit Amplitude $s$ und Frequenz $f$ beschleunigt mit $(2\pi f)^2 s$. Bei 150 Hz reicht dafür rechnerisch schon ein Weg von rund 0.1 µm. Der tote Bereich könnte also stark schrumpfen. Das Verhalten wird linearer, und damit dürften alle Regler besser laufen.

**Clever einsetzen:** Motor nur einschalten, wenn der Ball neben dem Ziel klebt. Rüttelt er dauernd, rollt der Ball bei jeder kleinen Schieflage davon.

**Haken:** Echte Reibung ist mehr als eine Schwelle, etwa Dellen in der Schiene. Vibration kann auch Sensorwerte stören oder Schrauben lösen. Der Motor braucht einen Transistor und eine Freilaufdiode, weil der Pico-Pin ihn nicht direkt treiben kann.

**Kleinster Versuch:** Handy mit Vibration auf die Schiene legen oder einen Motor festkleben. Schiene von Hand sehr langsam neigen: Rollt der Ball jetzt deutlich früher los?

### 3. Absichtlich pendeln

**Kerngedanke:** Haftreibung wirkt nur auf einen ruhenden Ball. Also lässt man ihn nie ruhen. Der Ball pendelt immer ein wenig um das Ziel, und im Mittel liegt er genau richtig. Was bei anderen Reglern als Fehler gilt, wird hier zur Methode.

**Theorie:** Kippt die Schiene mit Periode $T$ zwischen $\pm U$, beträgt die Schwingweite:

$$
x_{\text{Spitze-Spitze}} = \frac{A \, U \, T^2}{16}
$$

Mit $A$ = 60, $U$ = 3°, $T$ = 1 s sind das rund 11 mm, also $\pm$5.6 mm. Das läge innerhalb des Sollbereichs von $\pm$7.5 mm. Beim Umkehren ist die Neigung mit 3° über der Haftgrenze, der Ball bleibt also nicht hängen.

Umsetzen lässt es sich mit dem Bang-Bang-Regler ohne Ziel-Totzone: `u = U if e + T_LEAD*de > 0 else -U`. Die Periode stellt sich dabei von selbst ein und hängt von `T_LEAD` und der Verzögerung ab.

**Haken:** Der Servo arbeitet dauernd und macht Lärm. Der Ball steht nie, das sieht nicht nach «gehalten» aus. Die Amplitude lässt sich nur indirekt einstellen.

**Kleinster Versuch:** `Bang-Bang-Regelung.md` mit `TOL = 0`, also ohne Ruhezone. Im Plotter Amplitude und Mittelwert ablesen.

---

## B. Besser messen statt besser rechnen

### 4. Ball als Schleifer eines Potentiometers

**Kerngedanke:** Die Schiene wird selbst zum Sensor. Eine Metallkugel rollt über zwei leitende Bahnen, eine davon mit Widerstand. Die Kugel greift wie der Schleifer eines Potentiometers eine Spannung ab, die direkt der Position entspricht. Soweit ich weiss, arbeiten auch kommerzielle Lehrgeräte für Ball on Beam so.

**Warum das der grösste Hebel sein könnte:**

| Grösse | ToF heute (Annahme) | Schleifer (Schätzung) |
| ------ | ------------------- | --------------------- |
| Messrate | rund 33 Hz | einige hundert Hz |
| Auflösung | rund 1 mm, Rauschen 2–3 mm | 0.1 mm pro ADC-Schritt bei 12 Bit |
| Filterbedarf | stark, darum Verzögerung | gering |
| Fehlmessung | Wert 8190 | Kontakt offen, erkennbar |

Weniger Verzögerung bedeutet mehr Spielraum für jeden Regler. Vielleicht ist dann selbst dein bestehender PID schon deutlich besser, weil `de` sauberer wird.

**Theorie zum Aufbau:** Ende der Widerstandsbahn an 3V3 und GND, Gesamtwiderstand rund 100 Ω bis 10 kΩ. Dann fliessen höchstens 33 mA und der ADC liest stabil. Die zweite, gut leitende Bahn geht an den ADC, dazu rund 1 MΩ nach GND. Hebt der Ball ab, sinkt der Wert gegen 0 und das Programm erkennt den offenen Kontakt.

**Haken:** Die Kugel muss aus Metall sein. Kontakte oxidieren, die Spur nutzt sich ab und muss gleichmässig sein. Kontaktprellen erzeugt Spitzen, darum Median aus drei Messungen. Der Servo bleibt bei 50 Hz und behält seine eigene Verzögerung.

**Kleinster Versuch:** Bleistift-Grafit oder leitfähige Folie als Widerstandsbahn, Kupferband als zweite Bahn, Stahlkugel. Nur `print(adc.read_u16())` und Kugel von Hand rollen: Ist der Verlauf linear und glatt?

### 5. Der Regler hört den Ball

**Kerngedanke:** Ein rollender Ball macht Geräusche, ein ruhender nicht. Ein Piezo-Scheibchen an der Schiene wird zum Sensor für «rollt» oder «steht», ohne Umweg über eine Ableitung. Das sieht kein anderer Regler in diesem Projekt.

**Warum wertvoll:** Heute braucht der Regler mehrere Messungen, bis `de` unter `V_TOL` fällt. Das Geräusch könnte das fast sofort zeigen. Der Golfer und jede Kick-Logik wüssten sofort, wann der Ball klebt. Die Lautstärke dürfte ausserdem grob mit dem Tempo wachsen.

**Theorie zum Aufbau:** Piezo über 1 MΩ belasten und mit Schutzdioden sichern, denn **Spannungsspitzen können den ADC-Eingang beschädigen**. Das Signal über eine kurze Zeit abtasten und den Effektivwert bilden, oder mit Diode und Kondensator vorher glätten.

**Haken:** Der Servo ist vermutlich lauter als der Ball. Hören geht darum oft nur, während der Servo stillsteht. Die Lautstärke hängt von Schiene und Kugel ab, und die Richtung fehlt.

**Kleinster Versuch:** Piezo gesichert am ADC, Werte im Plotter ansehen. Drei Zustände vergleichen: Ball steht, Ball rollt, Servo bewegt sich.

### 6. Beobachter schätzt die Störung live

**Kerngedanke:** Der Holt-Filter weiss nicht, was der Servo gerade tut. Ein **Beobachter** rechnet dagegen ständig ein kleines Modell mit. Er vergleicht es mit der Messung und erklärt jede Abweichung als unbekannte Störung $f$. Schiefstand, Reibung und falsches $A$ landen alle in dieser einen Zahl. Fachlich heisst das *Extended State Observer*, der Regler dazu ADRC (*Active Disturbance Rejection Control*).

**Theorie:** Mit dem Modell $\ddot e = -A u + f$ und der Beobachter-Bandbreite $\omega$ in rad/s:

```python
err = e_raw - e_hat
e_hat += dt*(de_hat + 3*w*err)
de_hat += dt*(-A*u + f_hat + 3*w*w*err)
f_hat += dt*(w*w*w*err)
u = (K*e_hat + D*de_hat + f_hat)/A
```

Der Winkel `u` hebt die geschätzte Störung direkt auf. Sinnvolle Startwerte wären etwa $\omega$ = 10, $K$ = 9, $D$ = 6.

**Warum wertvoll:** `f_hat` lässt sich anzeigen: Der Regler sagt dir laufend, wie schief die Schiene gerade steht. Du könntest `OFFSET` sparen und den Tisch mitten im Betrieb kippen. Weil das Modell den Servo kennt, hinkt die Schätzung von `de` weniger nach. Das ist auch die Grundlage für Nr. 11.

**Haken:** Ganz ohne PID-Verwandtschaft geht es nicht: Die lineare Form wirkt rechnerisch wie ein PID mit Filter, und `f_hat` übernimmt die Rolle des I-Anteils. Das bringt dasselbe Risiko wie Wind-up: Während der Ball klebt, wächst `f_hat`, und beim Losrollen schiesst er über.

**Kleinster Versuch:** Beobachter nur mitrechnen lassen, ohne ihn für die Regelung zu nutzen. Läuft dein PID, kippst du den Tisch und schaust, ob `f_hat` dem Kippwinkel folgt.

---

## C. Physik übernimmt die Dämpfung

### 7. Wirbelstrom als eingebaute Dämpfung

**Kerngedanke:** Der D-Anteil soll den Ball bremsen, wenn er schnell ist. Die Physik kann das auch: Eine Neodym-Kugel in einer Schiene aus Aluminium oder Kupfer erzeugt Wirbelströme, und die bremsen proportional zum Tempo. Die Dämpfung braucht dann weder Sensor noch Ableitung, sie hat kein Rauschen und keine Verzögerung.

**Theorie:** Mit Bremswert $c$ und einem reinen P-Regler $u = k e$ gilt:

$$
\ddot e + c \, \dot e + A k \, e = 0
$$

Keine Überschwinger gibt es ab $k = c^2/(4A)$. Mit angenommen $c$ = 5 /s und $A$ = 60 wären das $k \approx 0.10$ °/mm, ähnlich deinem `KP_FINE`. Dafür begrenzt sich das Tempo auf $v = A u / c$: Bei 5° sind das 60 mm/s, für 200 mm also gut 3 s.

**Haken:**

- **Sicherheit:** Starke Magnete können Finger quetschen, splittern und Implantate wie Herzschrittmacher stören. Nicht verschlucken lassen.
- Wie stark die Bremse in einer offenen Schiene ist, ist völlig offen. Eindrücklich ist der Effekt vor allem im geschlossenen Rohr.
- Die Kugel zieht Schrauben und Servo-Teile an. Der Sensor kann an der glänzenden Kugel schlecht messen.

**Kleinster Versuch:** Neodym-Kugel gegenüber Stahlkugel gleicher Grösse: Beide über dasselbe Stück Aluminium-Profil rollen und die Zeit stoppen. Ohne deutlichen Unterschied ist die Idee erledigt. Die Variante ohne Magnet wäre eine Rinne mit Öl.

---

## D. Die Schiene lernt sich selbst

### 8. Landkarte der Schiene

**Kerngedanke:** `OFFSET` nimmt an, die Schiene sei überall gleich. Echte Schienen sind verzogen und haben Dellen. Der Pico vermisst darum Stelle für Stelle die lokale Neigung und Reibung und legt daraus eine Karte an:

$$
u_{\text{gesamt}} = u_{\text{Regler}} + \text{Karte}(x)
$$

**Theorie:** `test_offset.py` findet schon heute Waagrecht und toten Bereich. Dasselbe Prinzip lässt sich je Abschnitt von rund 20 mm anwenden: Winkel «rollt weg» und «rollt zurück» suchen. Die Mitte ergibt die Karte, die halbe Differenz die lokale Kick-Stärke. Bei 15 Abschnitten und rund 1 Minute pro Abschnitt wären das 15 Minuten, vollautomatisch.

**Haken:** Die Karte gilt nur, solange nichts verschoben wird. Der Ball muss für jede Stelle sauber hingebracht werden, was einen funktionierenden Regler voraussetzt.

**Kleinster Versuch:** Ball an drei Stellen legen und jeweils den Winkel suchen, ab dem er rollt. Unterscheidet sich die Mitte um weniger als rund 0.5°, ist die Schiene gleichmässig und die Idee bringt wenig. Dann weisst du das aber auch sicher.

### 9. Der Pico lernt seine eigene Physik

**Kerngedanke:** Q-Learning lernt direkt, was zu tun ist, und braucht dafür Tausende Läufe. Viel weniger Daten braucht eine einfachere Frage: **«Was passiert, wenn ich bei diesem Tempo diesen Winkel einstelle?»** Der Pico lernt im Betrieb eine kleine Tabelle der Physik und nutzt sie im MPC statt der Formel. Fachlich: modellbasiertes *Reinforcement Learning*.

**Theorie:** 7 Tempostufen mal 9 Winkel ergeben 63 Zahlen. Jeder Eintrag merkt sich die mittlere Tempoänderung pro Schleife:

```python
dv[i][j] += 0.05*(de - de_alt - dv[i][j])
```

Am Anfang steht in jedem Feld die Physik: $-A \cdot u \cdot \Delta t$. Damit läuft er ab der ersten Sekunde und wird dann genauer. Bei kleinem Winkel und ruhendem Ball lernt die Tabelle von selbst fast 0, also die Haftreibung, die die heutige MPC-Formel nicht kennt.

**Zur Datenmenge:** Das Rauschen von `de` liegt im Modell bei rund 10 mm/s, das Signal bei 1.8 mm/s pro Grad und Schleife. Für $\pm$0.5 mm/s braucht es grob $(10/0.5)^2 = 400$ Werte, also rund 12 s je Feld.

**Haken:** Winkel, die nie vorkommen, lernt die Tabelle nie. Die Wirkung eines Winkels kommt verzögert, darum gehört zu jedem Tempo der Winkel von rund drei Schleifen zuvor. Vorhersage plus Tabellensuche kosten Rechenzeit.

**Kleinster Versuch:** Während dein PID läuft, nur die Tabelle füllen und nach 5 Minuten ausgeben. Allein das ist wertvoll: Du erhältst **$A$ und den toten Bereich aus dem normalen Betrieb**, ohne extra Messung.

### 10. Die Schiene optimiert sich selbst

**Kerngedanke:** Die Schiene kann den Ball selbst zurückrollen, das macht `test_offset.py` schon. Niemand muss also danebenstehen. Der Pico probiert leicht veränderte Parameter aus, misst die Leistung und verbessert sich an der echten Schiene, ganz ohne Modell.

**Theorie:** Das Verfahren **SPSA** (*Simultaneous Perturbation Stochastic Approximation*) braucht pro Schritt nur zwei Versuche, egal wie viele Parameter es gibt. Es verändert alle Parameter $\theta$ zugleich zufällig um $\pm c$:

$$
g_i = \frac{J(\theta + c\Delta) - J(\theta - c\Delta)}{2 c \Delta_i}, \qquad \theta \leftarrow \theta - a \, g
$$

$J$ ist die Note eines Versuchs, zum Beispiel mittlerer Fehler plus Servo-Weg. Ein Versuch dauert mit Rückrollen rund 15 s, ein Schritt also 30 s, das sind rund 120 Schritte pro Stunde.

**Warum wertvoll:** Das passt zu jedem Regler, auch zu deinem PID: `kd`, `ki`, MPC-Gewichte oder gar die Gewichte eines kleinen neuronalen Netzes. Es umgeht das grösste Problem aller Simulationen, nämlich dass das Modell falsch sein kann.

**Haken:**

- Die Noten sind verrauscht und der Servo läuft dauernd. Das kostet Verschleiss und Wärme.
- Grenzen für alle Parameter setzen und die beste Einstellung laufend auf den Flash schreiben. Bleibt der Sensor blind oder klemmt etwas, muss das Programm stoppen.
- **Die ersten Läufe beaufsichtigen.** Für Betrieb über Nacht die Servo-Temperatur vorher prüfen.

**Kleinster Versuch:** Nur `kd` in 20 Minuten unter Aufsicht optimieren: Landet es in der Nähe deines von Hand gefundenen Werts 0.08?

---

## E. Gleiche Hardware, anderer Zweck

### 11. Der Regler als Messgerät

**Kerngedanke:** Ein Regler weiss viel über die Welt: Der Winkel, den er braucht, um den Ball zu halten, ist eine Messung. Aus dem Regelungsprojekt wird ein Instrument. Vier Varianten, von einfach bis schwierig:

| Instrument | Theorie | Haken |
| ---------- | ------- | ----- |
| **Hohl oder voll?** | Vollkugel: $a = \tfrac{5}{7} g \sin\theta$, Hohlkugel: $a = \tfrac{3}{5} g \sin\theta$. Unterschied rund 19 %, die Masse spielt keine Rolle. Mit dem Messskript aus dem MPC-Dokument erkennbar, ohne die Kugel zu öffnen. | Kugeln gleicher Grösse nötig, Rille verändert die Faktoren etwas. |
| **Rollreibungs-Prüfstand** | Anrollen und Ausrollen vergleichen. Die Differenz ist die Rollreibung, ganz ohne Kraftsensor. Für Vergleich von Oberflächen oder Kugeln. | Nur relative Aussage. |
| **Neigungsmesser** | Hält der Regler den Ball, entspricht der mittlere Winkel der Tischneigung. Mit Nr. 6 läuft das gleich mit. | Haftreibung begrenzt die Auflösung, Nr. 2 hilft. |
| **Viskosimeter** | Kugel im schrägen Rohr mit Flüssigkeit: Das Tempo sinkt mit der Viskosität. Der Regler hält das Tempo konstant, und der nötige Winkel misst die Viskosität. Das Kugelfall-Prinzip gibt es als echtes Messgerät (Höppler). | Der ToF-Sensor sieht kaum durch Wasser. Stattdessen Stahlkugel mit Hall-Sensoren oder Lichtschranken. Eichung nötig. |

**Warum wertvoll:** Das Thema «Biomedizin» aus der Unterrichtsunterlage bekommt so ein greifbares Beispiel, und das Projekt hebt sich klar von anderen ab.

**Kleinster Versuch:** Hohl gegen voll. Es braucht nur zwei Kugeln und das vorhandene Messskript.

### 12. Motorik-Trainer mit Stützrädern

**Kerngedanke:** Ein Mensch hält den Ball per Poti, der Regler hilft dabei so viel wie nötig:

$$
u = \alpha \, u_{\text{Mensch}} + (1 - \alpha) \, u_{\text{Regler}}
$$

Läuft es gut, steigt $\alpha$, und der Mensch übernimmt mehr. Klappt es nicht, greift der Regler stärker ein. Die Rehabilitationsrobotik kennt das Prinzip als *Assist-as-needed*.

**Warum wertvoll:** Das Gerät misst nebenbei: Zeit im Sollbereich, Reaktionszeit nach einem Stoss, Zittern. Es knüpft an das Thema Biomedizin im Kurs an, ist für Dritte ein Spiel und passt gut zu den Daten der Imitations-Regelung.

**Haken:** Kein Medizinprodukt, die Messwerte taugen nur für Vergleiche und Demo. Mit stark helfendem Regler kann sich der Mensch ausgetrickst fühlen.

**Kleinster Versuch:** Poti plus Fuzzy-Regler mit fest $\alpha$ = 0.5. Fühlt es sich leichter an, und merkt man die Hilfe überhaupt?

### 13. Ball-Telefon

**Kerngedanke:** Zwei Schienen sind per WLAN verbunden. Stösst du den Ball auf Schiene A, rollt der Ball auf Schiene B mit. `pico/sw08/get_mac_address.py` zeigt, dass WLAN ohnehin im Kurs vorkommt.

**Theorie:** Der Sollwert von B ist die Position von A: `x_set_B = x_A`. Im zweiten Schritt läuft es in beide Richtungen, jede Seite folgt der anderen. Das Gebiet heisst *bilaterale Teleoperation*.

**Haken:** Es braucht zwei Aufbauten, also Mitstudierende. Beide Regler müssen schon gut laufen, und die Netzwerk-Verzögerung kommt dazu. In beiden Richtungen kann das System leicht aufschaukeln, darum zuerst nur in eine.

**Kleinster Versuch ohne zweite Schiene:** Der **Geisterball**. Position während eines Handstosses aufzeichnen und als Sollwert-Verlauf wieder abspielen. Damit kannst du den Ball auch zu Musik tanzen lassen.

---

## F. Ganz wild

### 14. Wellen-Schiene

**Kerngedanke:** Statt die ganze Schiene zu kippen, läuft eine Welle durch einen biegsamen Streifen, angetrieben von einer Nockenwelle. Der Ball sitzt im Wellental und wird mitgetragen wie Nahrung im Darm (*Peristaltik*) oder wie die Mutter auf einer Gewindestange. **Die Position folgt direkt aus dem Motorwinkel**, also im Idealfall ganz ohne Sensor und Regelung: ein Schrittmotor für Bälle.

**Theorie:** Bei Amplitude $h$ und Wellenlänge $\lambda$ ist die grösste Steigung $2\pi h / \lambda$. Mit $h$ = 3 mm und $\lambda$ = 100 mm sind das rund 11°, deutlich über der Haftgrenze. Die Mulde bewegt sich, der Ball folgt, solange die Welle nicht zu stark beschleunigt.

**Warum wertvoll:** Löst Instabilität, Verzögerung und Haftreibung auf einen Schlag und ist bestimmt ein Blickfang. Mit mehreren Wellen wären sogar mehrere Bälle unabhängig vom Regler transportierbar.

**Haken:** Aufwendige Mechanik mit Reibung und Lärm. Der Streifen ist schwer sauber zu führen, und das Projekt wird zum Mechanik-Projekt. Eine Regelungsaufgabe ist das nicht mehr.

**Kleinster Versuch:** Karton-Streifen über drei Bleistiften, die du von Hand im Takt anhebst: Lässt sich eine Kugel überhaupt so transportieren?

---

## Kombinationen

Einzeln können diese Ideen nützlich sein, zusammen vielleicht mehr:

| Kombination | Mögliche Wirkung |
| ----------- | ---------------- |
| Nr. 4 + beliebiger Regler | Weniger Verzögerung, also ruhigere Regelung ohne neues Regelungskonzept |
| Nr. 2 + MPC | Rütteln löscht die Haftreibung, die dem MPC im Fall «Schwer» zu schaffen macht |
| Nr. 9 + Nr. 10 | Pico lernt die Physik im Betrieb und stellt die Gewichte selbst ein |
| Nr. 6 + Nr. 11 | Regler und Messgerät in einem: `f_hat` zeigt die Tischneigung |
| Nr. 1 + Golfer | Kerbe fängt den Ball, der Schlag muss nur ungefähr stimmen |
