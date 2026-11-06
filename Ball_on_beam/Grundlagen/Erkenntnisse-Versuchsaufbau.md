# Erkenntnisse Versuchsaufbau Ball on Beam

Stand: 06.11.2026. Alle Werte wurden an diesem Aufbau gemessen, sofern nicht anders vermerkt.
Vermutungen sind als **Hypothese** gekennzeichnet.

---

## 1. Kurzfassung: die wichtigsten Zahlen

| Grösse | Wert |
|---|---|
| Mitte (Ziel) | **169.5 mm** real, Sensorwert ca. **206** |
| Toleranz ±10 mm | Sensorwert ca. **190 bis 222** |
| Weg der Schiene | 0 mm (Anschlag Sensor) bis 339 mm (Anschlag Klotz) |
| Waagrecht (Offset) | Servo **+1.9°** (Startwert, wird im Betrieb nachgelernt) |
| Übersetzung Servo → Schiene | ca. **0.14°** Schiene pro 1° Servo |
| Nutzbereich Schiene | ca. **±4°** (Servo ±30°) |
| Ballbeschleunigung | ca. **10 mm/s² pro 1° Servo** (nahe Waagrecht) |
| Rollreibung | ca. **25 mm/s²**, das entspricht ca. **2.5° Servo** |
| Messrate Sensor | ca. **34 Hz** |
| Rauschen Sensor in Ruhe | 1.4 bis 4 mm (steigt mit der Distanz) |
| Schnellstmöglich Ende → Mitte | ca. **1.6 s** (Abschätzung) |

---

## 2. Ziel und Anforderungen

- Der Ball startet irgendwo ausserhalb der Mitte, auch direkt beim Klotz.
- Er soll in die Mitte gebracht werden und dort ruhig liegen bleiben.
- **Toleranz:** ±10 mm um die Mitte.
- **«Zur Ruhe»:** durchgehend 1 s innerhalb von ±10 mm.
- **Fail:** länger als 30 s.
- **Endziel:** 3 bis 5 s, nachdem die besten Methoden kombiniert und optimiert worden sind.
- **Vorgehen:** pro Methode ein Agent, zuerst in der Simulation, dann am echten Aufbau.

---

## 3. Hardware

| Teil | Details |
|---|---|
| Mikrocontroller | Raspberry Pi Pico H, MicroPython, VS Code mit MicroPico |
| Sensor | VL53L0X (Grove, Time of Flight), I2C0: **SDA GP8, SCL GP9** |
| Servo | AMEWI AMX Racing 6221MG Digital an **GP16**, 50 Hz |
| Servo-Ansteuerung | Duty 1000 bis 9000 (`duty_u16`) für −90° bis +90°, begrenzt auf ±30° |
| Richtung | Positiver Servowinkel → Ball rollt **weg** vom Sensor |
| Schiene | zwei aufgeleimte Metallstäbe, nicht genormt, nicht überall parallel |
| Ball | normaler Ping-Pong-Ball, 40 mm, 2.7 g, laut Jan nicht perfekt rund |
| Zusatz | schwarzer Karton 5 × 5 cm am Klotz (muss bleiben, siehe Kapitel 4) |

**Achtung:** Die Pins weichen von der ZHAW-Vorlage ab (dort SDA GP0, SCL GP1, Servo GP15).

---

## 4. Sensor (VL53L0X)

### 4.1 Kennlinie: Sensorwert → echte Position

Der Sensor misst stark daneben. Der Fehler wächst mit der Distanz.
Gemessen mit `rauschtest.py`. Echte Position = Massstab, 0 = Ball am Anschlag beim Sensor, gemessen bis zur Ballvorderkante.

| Echte Position | Sensor ohne Karton | Sensor **mit Karton (gilt)** | Rauschen (std) |
|---|---|---|---|
| 0 mm (Anschlag) | 36 | **36** | 1.5 mm |
| 100 mm | 111 | **109** | 1.4 mm |
| 150 mm | 190 | **174** | 2.0 mm |
| 200 mm | 291 | **256** | 2.5 mm |
| 250 mm | 363 | **333** | 3.1 mm |
| 300 mm | 412 | **391** | 4.2 mm |
| 339 mm (Anschlag Klotz) | 446 | **419** | 3.6 mm |
| ohne Ball | 463 | **460 bis 475** | 3.8 mm |

- **Hypothese zur Ursache:** Der Sensor sieht einen Kegel von 25°. Bei grosser Distanz füllt der Ball nur einen kleinen Teil davon. Der Messwert ist ein Mischwert aus Ball und Hintergrund und wird Richtung Hintergrund (ca. 460 mm) gezogen. Alle Messpunkte passen zu dieser Erklärung.
- **Lösung:** Umrechnungstabelle in `Scripts/kennlinie.py`. Der Regler arbeitet nur noch mit echten mm.
- **Wichtig:** Die Tabelle gilt nur mit dem schwarzen Karton am Klotz. Wird am Aufbau etwas verändert, muss die Kennlinie neu gemessen werden.

### 4.2 Eigenschaften

- **Winkelunabhängig:** Ball fixiert auf 200 mm, Servo −5° bis +10°: Der Wert ändert sich nur um ±1 mm (`winkeltest.py`).
- **Wiederholbar:** Gleiche Stelle, zwei Läufe: 256.1 und 255.6.
- **Auflösung:** In der Mitte ändert sich der Sensorwert um ca. 1.6 pro echtem mm. Das Rauschen entspricht dort real nur ca. 1.6 mm. Nahe beim Sensor (0 bis 100 mm) ändert er sich nur um ca. 0.7 pro mm, dort ist die Messung gröber.
- **Messrate:** ca. 34 Hz mit `tof.ping()`.
- **Messzeit (Timing Budget):** Der Treiber `vl53l0x.py` übernimmt die Einstellung nicht. Alle Varianten liefen mit 34 Hz. Ist nicht nötig, das Rauschen ist klein genug.
- **Mindestdistanz:** Der Ball am Anschlag liefert 36, also über dem Minimum von 30 mm. Ein zusätzlicher Anschlag ist nicht nötig.
- **Ball weg erkennen:** Ohne Ball liefert der Sensor ca. 460 und mehr.

### 4.3 Falsche Werte nach grossen Servosprüngen (wichtig für alle Methoden)

- Nach grossen, schnellen Servosprüngen (ab ca. 25°) liefert der Sensor kurz falsche Werte. **Hypothese:** Die Schiene schwingt nach, und der Sensor sitzt auf der Schiene.
- In `waagrecht_v3.py` blieb dadurch der Ausreisserfilter («Sprung über 40 mm verwerfen») am ersten falschen Wert hängen. Danach hat er alle richtigen Werte verworfen.

**Regeln für jeden Regler:**
1. Ein Ausreisserfilter darf nie einrasten. Nach wenigen verworfenen Werten in Folge den neuen Wert akzeptieren.
2. Nach grossen Sprüngen ca. 100 bis 200 ms Messwerte mit Vorsicht behandeln.
3. Grosse Sprünge wenn möglich vermeiden oder als Rampe fahren.

---

## 5. Mechanik (Servo → Schiene)

Gemessen mit phyphox «Neigung» (Handy längs auf den Stäben, Bildschirm unten) und `neigung_trend.py` / `servo_bereich.py`.

### 5.1 Servobereich

| Servo | Schiene gegenüber waagrecht |
|---|---|
| −30° | −4.05° |
| −25° | −3.51° |
| −20° | −2.91° |
| −15° | −2.32° |
| −10° | −1.66° |
| −5° | −0.97° |
| 0° | −0.27° |
| +5° | +0.41° |
| +10° | +1.18° |
| +15° | +1.89° |
| +20° | +2.55° |
| +25° | +3.19° |
| +30° | +3.89° |

- **Kein Anschlag, kein Totpunkt** im ganzen Bereich ±30°.
- **Leicht nichtlinear:** Nahe der Mitte 0.14 bis 0.15° Schiene pro 1° Servo, gegen −30° nur noch 0.11°.
- **Spiel:** praktisch keins. Bei 0° von verschiedenen Seiten angefahren: Unterschied unter 0.03°.
- **Vorteil der kleinen Übersetzung:** Ein Fehler von 1° am Servo bewegt die Schiene nur um 0.14°.

### 5.2 Stäbe

- Die Stäbe sind nicht überall gleich gerade: Je nach Lage des Handys ergab sich 0.23° Unterschied.
- Laut Jan sind sie auch nicht überall parallel.

---

## 6. Ball und Reibung

Gemessen mit `waagrecht_test.py` (erster Lauf, nur gültige Durchgänge).

| Grösse | Wert |
|---|---|
| Ballbeschleunigung | ca. 10.1 mm/s² pro 1° Servo |
| Rollreibung | ca. 25 mm/s², das entspricht ca. 2.5° Servo bzw. 0.35° Schiene |
| Wiederholbarkeit (Servo +2°) | weg: −24.4 / −22.4 mm/s², hin: +25.8 / +28.3 mm/s² |

- **Haftreibung:** Der Ball klebt. Bei leichter Neigung rollt er nicht von selbst los, nach Antippen aber schon. Die Haftreibung ist grösser als die Rollreibung.
- **Folge für den Regler:** Damit der Ball weiterrollt, braucht es ca. 2.5° Servo zusätzlich in Bewegungsrichtung. Das passt zum Wert `FC = 2` in `main.py`.
- **Vorteil der Haftreibung:** Liegt der Ball einmal ruhig in der Mitte, bleibt er dort auch bei kleinen Abweichungen der Schiene liegen.
- **Physik:** Pro Grad Servo würde ein reibungsfrei gleitender Körper ca. 24 mm/s² beschleunigen. Der Ball erreicht ca. 42 % davon. Ein Teil geht in die Drehung des Balls (Hohlkugel auf zwei Stäben).
- **Unrunder Ball und Stäbe:** Bei weiteren Läufen lagen die Werte nicht mehr auf einer Geraden. Diese Störungen sind real und müssen vom Regler ausgeglichen werden.
- **Ball am Klotz:** braucht mehr als 3 s, bis er nach dem Anrollen ruhig liegt.
- **Schnellstmöglich:** Mit voller Neigung (ca. 0.28 m/s² Nutzbeschleunigung) braucht der Ball für 170 mm ca. 1.6 s. Das ist eine Abschätzung (voll beschleunigen, dann voll bremsen). Das Ziel von 3 bis 5 s ist damit realistisch.

---

## 7. Waagrecht (Offset)

| Methode | Ergebnis | Bemerkung |
|---|---|---|
| `test_offset.py` (Ball rollt an den Enden los) | +0.5° | misst vor allem die Haftreibung, nur an den Enden |
| Handy (phyphox, korrigiert um Handyfehler −0.38°) | +3.7° (2.9 bis 4.5°) | Handy wiegt ca. 70× mehr als der Ball, misst an anderer Stelle |
| `waagrecht_test.py`, Lauf 1 (rollender Ball, beide Richtungen) | **+1.9°** | gültige Durchgänge, misst in der Mitte |
| `waagrecht_test.py`, weitere Läufe | unbrauchbar | Skriptfehler, danach Störungen durch Ball und Stäbe |

**Entscheid:**
- Startwert **Servo +1.9°**. Keine weiteren Offsetmessungen.
- Es gibt kein einheitliches «Waagrecht». Die lokale Neigung hängt von der Position und von der Stellung des unrunden Balls ab.
- Der Regler muss den Rest selbst ausgleichen (I-Anteil oder Störgrössenschätzung) und den Offset im Betrieb nachlernen.
- `calib.json` auf dem Pico enthält eventuell noch den Fehlwert −0.35 und muss auf 1.9 gesetzt werden.

---

## 8. Was jede Methode berücksichtigen muss

1. **Echte mm verwenden:** Sensorwert immer über `kennlinie.py` umrechnen. Ziel = `MITTE_MM` (169.5 mm).
2. **Reibung kompensieren:** ca. 2.5° Servo zusätzlich in Bewegungsrichtung. Zum Losrollen braucht es eher mehr.
3. **Offset nicht als exakt annehmen:** Start mit 1.9°, Rest über I-Anteil oder Lernen ausgleichen.
4. **Ortsabhängige Störungen:** Stäbe und unrunder Ball erzeugen Störungen, die sich mit der Position und beim Rollen ändern.
5. **Sensor nach Servosprüngen:** siehe Kapitel 4.3. Filter darf nie einrasten.
6. **Stellbereich:** Servo ±30° ergibt nur ca. ±4° Schiene. Der Regler wird oft am Anschlag (±30°) arbeiten. Anti-Windup ist nötig.
7. **Abtastrate:** ca. 34 Hz. Mehr ist mit diesem Sensor nicht möglich.
8. **Nahe am Sensor ist die Messung gröber** (0 bis 100 mm).
9. **Haftreibung ausnutzen:** In der Mitte angekommen, darf der Regler ruhig werden (Totband). So pendelt der Ball nicht.
10. **Startposition beliebig:** Jede Methode muss von beiden Seiten und von jeder Position ausserhalb der Mitte funktionieren.

---

## 9. Parameter für die Simulation

| Grösse | Nennwert | Zufälliger Bereich |
|---|---|---|
| Schienenlänge (Ballvorderkante) | 0 bis 339 mm | fest |
| Servo → Schiene | Tabelle Kapitel 5.1 | fest |
| Offset | +1.9° Servo | ±1.5° Servo |
| Ballbeschleunigung | 10.1 mm/s² pro ° Servo | ±20 % |
| Rollreibung | 25 mm/s² | 15 bis 35 mm/s² |
| Haftreibung | grösser als Rollreibung | Ball bleibt bei kleiner Neigung liegen |
| Höhenprofil der Stäbe | ortsabhängige Zusatzneigung | ca. ±0.2° Schiene, zufällig pro Lauf |
| Unrunder Ball | periodische Störung abhängig vom Rollweg | zufällige Phase und Stärke pro Lauf |
| Servo | kleine Verzögerung | |
| Sensor | Kennlinie Kapitel 4.1, 34 Hz | Rauschen 1.5 bis 4 mm |
| Sensor nach Servosprung | falsche Werte | ca. 100 bis 200 ms |
| Startposition | | zufällig, beide Seiten |

Viele Durchläufe mit zufälligen Werten (Monte Carlo). Eine Methode gilt erst als gut, wenn sie über den ganzen Bereich besteht.

---

## 10. Kalibrierung beim Start (max. ca. 5 s, Ball bleibt drauf)

1. Servo auf den gelernten Waagrecht-Wert stellen (`calib.json`, Start 1.9°).
2. Voll Richtung Sensor kippen: Sensorwert muss ca. 36 sein.
3. Voll weg kippen: Sensorwert muss ca. 419 sein.
4. Weicht ein Endwert um mehr als ca. 10 ab: Warnung «Kennlinie neu messen».
5. Danach regeln in echten mm.
6. Im Betrieb: Servowinkel bei ruhigem Ball in der Mitte als neuen Offset in `calib.json` speichern.

- Ball weg oder Ball drauf wird automatisch über den Sprung im Sensorwert erkannt (ohne Ball ca. 460 und mehr).
- Am fernen Ende genug Zeit einplanen: Der Ball braucht dort mehr als 3 s, bis er ruhig liegt.

---

## 11. Skripte im Ordner `Scripts`

| Skript | Zweck | Status |
|---|---|---|
| `kennlinie.py` | Umrechnung Sensorwert ↔ echte mm | **in Gebrauch**, muss auf dem Pico sein |
| `rauschtest.py` | Rauschen und Kennlinie messen | erledigt, bei Umbau wiederholen |
| `winkeltest.py` | Prüft, ob der Sensorwert vom Kippwinkel abhängt | erledigt |
| `neigung.py` | Handywerte von Hand eingeben, Offset und Übersetzung rechnen | ersetzt durch `neigung_trend.py` |
| `neigung_trend.py` | Servowinkel nach Zeitplan für phyphox-Aufzeichnung | erledigt |
| `servo_bereich.py` | Servobereich ±30° langsam abfahren | erledigt |
| `waagrecht_test.py` | Offset mit rollendem Ball (alte Version) | nicht mehr verwenden |
| `waagrecht_v3.py` | Offset mit rollendem Ball (neue Version) | Filterfehler (Kapitel 4.3), nicht mehr nötig |
| `test_offset.py` | Offset über Losrollen (bestehend) | misst Haftreibung, nicht verwenden |
| `main.py` | bestehender Regler | Sollwert 200 (Sensorwert) = real 165.9 mm |

**Hinweis:** Dateien, die gerade in VS Code offen sind, kann VS Code beim Speichern mit seiner alten Version überschreiben. Neue Versionen darum unter neuem Namen ablegen oder den Tab vorher schliessen.

---

## 12. Offene Punkte

- Methoden-Markdowns fertigstellen (Jan).
- `calib.json` auf dem Pico auf Offset 1.9 setzen.
- Gemeinsame Simulation für alle Methoden bauen.
- Zugriff auf den Pico: Die Shell von Claude hat keinen USB-Zugriff. Testen am echten Aufbau läuft über VS Code / MicroPico durch Jan oder über Computer Use.

---

## Quellen

- ZHAW PCLS, PID-Regelung (Ball on Beam, Vorlage): https://github.zhaw.ch/pages/ADLS-Projekte-Labs/PCLS/PID-Regelung.html
- DFRobot Wiki, VL53L0X (Messbereich, 25° Sichtfeld, ±3 %): https://wiki.dfrobot.com/Gravity__VL53L0X_ToF_Laser_Range_Finder_SKU_SEN0245
- phyphox Wiki, Experiment «Inclination»: https://phyphox.org/wiki/index.php/Experiment:_Inclination
- phyphox Experiment-Datei `inclination.phyphox` (Formeln Kippwinkel): https://github.com/phyphox/phyphox-experiments/blob/master/inclination.phyphox
- Wikipedia, Table tennis (Ball 40 mm, 2.7 g): https://en.wikipedia.org/wiki/Table_tennis
