# Fuzzy-Regelung (`fuzzy`): Notizen Simulation 06.11.

## Idee in 3 Sätzen
Eine 9×9-Regeltabelle über Fehler e (mm) und Tempo v (mm/s) gibt den Grundwinkel aus, erzeugt aus einer Bremskurve («weit weg und ruhig: stark kippen», «nahe und schnell: bremsen»).
Dazu kommen Zusatzregeln in Worten: Haltewinkel lernen (langsamer I-Anteil), «Ball steckt: Stoss wachsen lassen», «Ball steckt trotz vollem Stoss: schaukeln», «Mitte und ruhig: Winkel einfrieren».
Alles ist Sugeno mit linearen Dreiecks-Zugehörigkeiten, pro Schritt sind nur 4 Tabellenregeln aktiv.

## Geändert gegenüber dem alten MD (und warum)
- Stützpunkte e ±10/30/80/170 mm, v ±15/50/120/250 mm/s (echte Schiene 339 mm, nicht 400).
- Tabelle nicht von Hand, sondern aus Bremskurve (kv 0.3 °/(mm/s), t_lin 0.72 s, Bremsbeschl. 110 mm/s², v_max 220 mm/s) plus **Steifigkeit kp0 = 0.6 °/mm bis ±30 mm**. Grund: Die Ballstörung (bis ±180 mm/s², Periode ca. 105 mm) wirkt wie eine Feder von bis zu 12 mm/s² pro mm; der alte Kp ≈ 0.15 ist dagegen zu weich. Ohne kp0: 154/200 statt 197/200 (Kontrolle seed0=5000).
- **Haltewinkel lernen** ki = 0.2 °/(mm·s) innerhalb ±40 mm, Bereich **±25°** (alt ±4°). Grund: Der nötige Haltewinkel in der Mitte ist wegen der Ballstörung oft 10 bis 28°. Ohne: 151/200.
- **Losbrech-Regel** wächst mit der Wartezeit (4° + 15°/s, bis 25°) statt bis 9°: Ball liegt sonst in Mulden 20 bis 40 mm neben der Mitte fest.
- **Schaukel-Regel** (neu): Ball länger als 1.7 s fern (> 8 mm) und langsam: rollt er zurück, kippt die Schiene mit (−10°), sonst voll Richtung Mitte (28°). Brachte 197 statt 195/200. Eine aktive Variante «zuerst Anlauf nehmen» (Parameter t_back > 0) war nicht besser (197/200, aber p90 6.9 s, Servo 90 °/s), darum aus.
- **Ruhezone**: friert den Ausgang ein (alt: Sprung auf Haltewert). Die Störung hängt nur von der Position ab, ein ruhender Ball bleibt bei festem Winkel liegen. In der Sim wirkt sie kaum (ohne: 197/200, Servo 84 statt 73 °/s); behalten wegen Servo-Ruhe am echten Aufbau (Vermutung: weniger Zittern).
- Rollreibungs-Regel fk_roll und Zusatzdämpfung kd0 sind im Code, aber 0: kd0 0.3 verschlechterte (p90 12 s, Tempo-Rauschen).
- Keine Fourier-Schätzung der Ballstörung (nicht geprüft, Zeitbudget).

## Endparameter (= Standardwerte in `pico/ctrl_fuzzy.py`)
kv 0.3, t_lin 0.72, a_brems 110, v_max 220, u_max 24, kp0 0.6 | fk_haft 4, fk_zeit 15, fk_haft_max 25, v_ruhig 15.6 | e_zone 3.5, e_zone_b 2.5, v_zone 15 | ki 0.2, e_i 40, i_max 25 | t_vor 0.04, du_max 150 °/s, AlphaBeta 0.5/0.15 | t_pump 1.7, u_back 10, t_pmax 6, e_pend 8, v_fest 40, t_back 0 | u_lim 28.
Gefunden: Handsuche (seed0=0, n=40/80), dann Koordinatensuche `tune.py` (seed0=0, n=80, Kosten = Mittel mit Fail 60 s + p90 + max/4), Kontrolle seed0=5000, n=200.

## Ergebnis
| Satz | Erfolg | T median | p90 | max | ≤5 s | ≤10 s | Servo |
|---|---|---|---|---|---|---|---|
| Abstimmung seed0=0, n=80 | 78/80 | 2.76 | 4.70 | 30 | 73 | 77 | 71 °/s |
| Kontrolle seed0=5000, n=200 | 197/200 | 2.94 | 5.90 | 30 | 170 | 193 | 73 °/s |
| **Validierung seed0=100000, n=200** | **194/200** | **2.89** | **5.46** | 30 | 175 | 193 | 74 °/s |

Validierung pro Start: sensor 48/50 (median 3.07 s), klotz 48/50 (3.00 s), nah_sensor 49/50 (3.12 s), nah_klotz 49/50 (2.18 s). Mess≠Wahr: 1.

## Schwächen
- Die 6 Validierungs-Fails haben fast alle k_servo ≤ 5.8 mm/s² pro ° (5 von 6) und meist hohe Reibung: Ball liegt bei vollem Winkel (±28°) in einer Mulde 15 bis 40 mm neben der Mitte oder am Anschlag fest, Schaukeln reicht nicht. Ein Lauf (sensor, K 7.0) pendelt langsam über einen Buckel in der Mitte (137 bis 195 mm).
- Arbeitet oft am Stellbereich-Rand (Haltewinkel 20 bis 28°). Am echten Aufbau ist das nur sinnvoll, wenn der Servo das dauerhaft verträgt.
- Viele Parameter (27), abgestimmt auf die Sim-Streuung; Tabelle ist kein echtes Expertenwissen mehr, sondern eine Bremskurve.
- Beobachtung Sim/bob_core (kein Fehler, aber wichtig): `vorbereiten()` mit kipp 20° bringt den Ball nicht immer ans Ende (Abstimm-Seed 48: Start «sensor», Ball blieb bei 208 mm). Echte «Endstarts» können darum mitten auf der Schiene beginnen.

## Rechenzeit
CPython: 1.8 µs pro step (zufällige x), 2.0 µs (Ball ruhig in Mitte), inkl. AlphaBeta, 20 000 Aufrufe, `time.perf_counter`. Hochgerechnet ×50 bis ×100: ca. 0.1 bis 0.2 ms auf dem Pico (Vermutung, nicht gemessen). Tabelle 81 floats, wird in `__init__` berechnet. Nur `bob_core` importiert, keine f-Strings, kein numpy.

## Vorschlag erster echter Test
1. Standardwerte, Start nah Klotz (in der Sim bester Fall). Log: x, u, `r.halt`, `r.t_fest`, `r.t_p`.
2. Darauf achten: Läuft `halt` gegen ±25° und bleibt dort, ist der Ball in einer Mulde; dann prüfen, ob die Schaukel-Regel anspringt (t_p ≥ 0) und ob sie den Ball heraus bringt.
3. Zittert der Servo in der Mitte stark: e_zone auf 5 mm, v_zone auf 20 mm/s erhöhen. Schwingt der Ball langsam um die Mitte (Periode 5 bis 10 s): ki auf 0.1 senken oder kp0 auf 0.8.
4. Sensorfehler nach Sprüngen: du_max 150 °/s bleibt unter 20° in 120 ms; nicht erhöhen.

## Kombination mit anderen Methoden
- Eine Störungsschätzung über x (Fourier sin/cos + Offset) könnte den Haltewinkel vorsteuern statt ihn langsam per I-Anteil zu suchen: Vermutung, dass das vor allem p90 und die Buckel-Fälle verbessert.
- Für die Anfahrt aus der Ferne könnte eine Schaltkurve (Bang-Bang) oder ein Golfer-Stoss die Tabelle ersetzen; die Fuzzy-Zusatzregeln (Haltewinkel, Schaukeln, Ruhezone) lassen sich unverändert an jeden Grundregler hängen.
