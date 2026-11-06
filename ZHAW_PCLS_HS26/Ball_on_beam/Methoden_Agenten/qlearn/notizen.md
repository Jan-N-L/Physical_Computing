# Q-Learning (qlearn), Stand 06.11., neue Sim mit unrundem Ball

## Idee
Zustand = |e| (9 Felder), Tempo v (11 Felder), Steckzeit (3 Stufen), gespiegelt (nur e >= 0 gespeichert, 297 Zustände).
Pro Zustand eine Aktion aus 9 Winkeln (±28, ±16, ±9, ±4, 0), neu gewählt nur jeden 3. Schritt, Ausgang mit 150 °/s Rampe.
Dazu U0 = +1.0° (Mitte des Offset-Bereichs) und ein I-Anteil nahe der Mitte (KI 0.6, |e| < 25 mm, ±10°).

## Ehrliches Hauptergebnis
**Das Q-Lernen hat in der neuen Sim nichts verbessert, sondern immer verschlechtert.** Die abgegebene Tabelle ist
die Warmstart-Tabelle (PD-Regel u = -0.6 e - 0.3 v - 10·Steckstufe, an den Feldmitten diskretisiert), also EPISODES=0.
Gemessen (sim.evaluate, n=80, seed0=0, alles mit KI 0.6 bzw. 0.4):

| Variante | Erfolg |
|---|---|
| Lernen ab Null, 16000 Ep., KI 0 / KI 0.4 (je Seed 1) | 2/80, 0/80 |
| Warmstart PD 0.4/0.2/8, dann 16000 Ep. (Seeds 1, 2) | 1/80, 4/80 |
| Warmstart, N0 60 (sehr träge), 8000 Ep. | 0/80 |
| Warmstart PD 0.6/0.3/10, Q0 -150, MIN_VISITS 20, 8000 Ep. (Seeds 3, 4) | 19/80, 2/80 |
| **Warmstart PD 0.6/0.3/10 ohne Lernen** | **56/80** |
| PD-Tabelle 0.4/0.2/8, 0.5/0.25/8 ohne Lernen | 47/80, 53/80 |

Auch in der eigenen schnellen Strecke (FastPlant) ist die gelernte Tabelle schlechter (13/60 vs. 43/60 Ruhe bis 15 s,
`diag_fast.py`). Es ist also kein Übertragungsfehler Sim zu FastPlant, sondern ein Lernproblem.
**Vermutung (nicht gemessen):** Der Zustand ist stark nicht-markovsch. Der unbekannte Ball-Drehwinkel (Störung ±15° Servo),
Offset und K pro Lauf sowie der momentane Servowinkel (Rampe 150 °/s, Aktionen bis 56° auseinander) fehlen im Zustand.
Tabellen-Q-Lernen mittelt dann über verdeckte Lagen und jagt Rauschen. Ein Q0 = -40 Startwert war zudem zu optimistisch:
selten besuchte Aktionen gewannen das greedy-Auslesen (Servo 90 °/s, erratisches Pendeln).

## Gegenüber altem MD geändert
Alte Zahlen (A = 60, 400 mm, linearer Sensor) verworfen. Spiegelung, SKIP 3 und mehrere Seeds beibehalten (Lehren Kap. 5).
Neu: Steckzeit im Zustand, Ausgangsrampe, U0, I-Anteil, Warmstart, FastPlant mit allen neuen Bereichen (K 5.5 bis 12,
Offset 0.4 bis 5.4, Störung 50 bis 180 mm/s², 92 bis 118 mm, Rollreibung 8 bis 30, Rollweg vorzeichenrichtig).

## Endparameter (in pico/ctrl_qlearn_tab.py)
E_EDGES (3,7,12,20,32,50,80,120), V_EDGES (±10,±25,±50,±90,±150), ST_EDGES (0.25, 0.8), U_SET oben, SKIP 3, RATE 150,
AlphaBeta 0.5/0.15, Steck: |v| < 12 und |e| > 6, U0 1.0, KI 0.6, I_ZONE 25, I_MAX 10. Tabelle 297 Bytes.

## Validierung (n=200, seed0=100000, einmal am Ende)
Erfolg 156/200 | T median 4.65 s, p90 30.00 s, max 30.00 s | <=5 s: 109, <=10 s: 143 | Servo 47 °/s | Mess!=Wahr 11

| Start | n | Erfolg | T_median | T_max |
|---|---|---|---|---|
| sensor | 50 | 35 | 5.78 | 30 |
| klotz | 50 | 40 | 5.03 | 30 |
| nah_sensor | 50 | 42 | 4.28 | 30 |
| nah_klotz | 50 | 39 | 3.44 | 30 |

## Schwächen
- 22 % Fail, p90 = 30 s. Kein Schätzer für die Ballstörung, nur I-Anteil an der Mitte und Steck-Kick.
- Grobe Tabelle: Mess!=Wahr 11/200, Ruhe-Fenster liegt oft knapp an ±10 mm.
- Der Lernteil (die eigentliche Methode) bringt in dieser Strecke nichts, siehe oben.

## Rechenzeit
CPython 0.86 µs pro step (20000 Aufrufe, perf_counter). Auf dem Pico geschätzt (×50 bis 100) unter 0.1 ms. Nur math-frei, bytes-Tabelle.

## Vorschlag erster echter Test
Lieber nicht als Erstes. Wenn doch: Standardwerte, Start nahe Klotz. Achten auf Pendeln zwischen ±16 und ±28 in der Mitte
(dann KI kleiner) und ob der I-Anteil am Rand ±10° hängen bleibt (Ball in Mulde neben der Mitte).

## Kombination
Q-Lernen eher als **Restkorrektur** auf einen guten Regler mit Störschätzer (Fourier sin/cos über x) sinnvoll:
Zustand = Fehler und Tempo, Aktion = kleine Korrektur ±3°, Basis liefert PID/MPC. Mit Störschätzer im Zustand
(z.B. Vorzeichen der geschätzten Störung) wäre der Zustand eher markovsch. Nicht getestet.

Dateien: tune.py (Training, `python3 tune.py train '<json>'`, `final`), diag_pdtab.py, diag_fast.py, logs/.
Reproduktion Endtabelle: `python3 tune.py train '{"SEEDS":[0],"KI":0.6,"PRIOR":[0.6,0.3,10],"EPISODES":0,"WRITE":1}'`.
