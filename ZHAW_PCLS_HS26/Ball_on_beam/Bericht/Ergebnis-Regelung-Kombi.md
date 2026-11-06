# Ball on Beam: Ergebnis Methodenvergleich und Kombination

> **Nachtrag 06.11.2026, nach dem Aufräumen:** Dieser Bericht ist der Stand vom Morgen (Aufträge 0001 bis 0017).
> Danach: Jan fand die Kombi live zu langsam und zu unruhig. Darum entstanden main_v3 und main_q, siehe
> `Bericht/Report_main_v3_main_q.md`. Spätere Läufe der Kombi: Auftrag 0020 4 von 4 (2.1, 9.1, 11.1, 2.4 s),
> als Aktion A4 in main_q 10 von 10 (Median 10.8 s). Insgesamt echt 30 von 30, aber mit wachsender Streuung.
> Pfade unten auf die neue Ordnerstruktur nachgeführt.

Stand 06.11.2026, morgens. Alle Zahlen «echt» stammen vom Aufbau, über die Brücke `Bruecke/pico_bridge.py` gemessen. Logs: `Messdaten/pico_logs/`.

## Kurzfassung

| | |
|---|---|
| **Bester Regler** (Stand Morgen) | `Pico_Programme/ctrl_kombi.py` (MPC-Kern, neu abgestimmt, mit Losbrech-Stoss) |
| **Echt** | **16 von 16** Läufen erfolgreich (Start abwechselnd am Sensor und am Klotz) |
| **Zeit bis «zur Ruhe»** | Median **3.5 s**, 13 von 16 unter 5 s, langsamster 16.1 s |
| **Ziel 3 bis 5 s** | im Median erreicht, aber nicht in jedem Lauf (3 von 16 über 5 s) |
| **Live-Programm** | `Pico_Programme/demo_kombi.py` |

Zeit = Beginn des Fensters, in dem der Ball 1 s durchgehend in ±10 mm um 169.5 mm liegt (Ballmitte ca. 19 cm). Danach wird 3 s weiter geprüft, ob er liegen bleibt.

## Die wichtigsten Befunde am Aufbau

1. **Der Ball hat eine schwere Seite.** Störung ca. ±90 bis ±160 mm/s², das sind 6 bis 15° Servo, periodisch über den Rollweg (ca. 100 mm pro Umdrehung). Daher kommen das Hängenbleiben in «Mulden» und das Pendeln in Ruhe. Mit dem Punkt seitlich gelegt wird es kleiner, verschwindet aber nicht. Beleg: `0004_scan.log`, `0005_scan_seitlich.log`, Modell-Fit rms 4 bis 5 mm.
2. **Der Servo wirkt stärker als angenommen:** ca. **13 bis 15 mm/s² pro ° Servo** im Regelbetrieb (Erkenntnisse: 10.1). Regression über 16 echte Läufe.
3. **Die Verzögerung ist grösser:** ca. **0.12 s** vom Befehl bis zur Wirkung (vorher angenommen ca. 0.05 s).
4. Befund 2 und 3 zusammen erklären, warum fast alle Regler am echten Aufbau pendeln: Sie waren im Simulator auf zu schwache und zu schnelle Strecke abgestimmt.
5. Die Bedingungen ändern sich von Durchgang zu Durchgang (vermutlich dreht der Ball beim Anschlag etwas). Darum gelten nur Vergleiche, die **abwechselnd im selben Auftrag** gelaufen sind.

## Methodenvergleich

Sim v2 = Simulator, auf den die Agenten abgestimmt haben (n = 200). Sim v3 = nachher an echten Läufen geeicht (n = 40). Echt = je 4 Läufe.

| Methode | Sim v2 Erfolg | Sim v2 Median | Sim v3 Erfolg | **Echt** |
|---|---|---|---|---|
| MPC (Vorhersage) | 193/200 | 2.7 s | 40/40 | **10 von 14** |
| Fuzzy | 194/200 | 2.9 s | 25/40 | 0 von 4 |
| Golfer-Impuls | 190/200 | 3.8 s | 1/40 | 0 von 4 |
| Bang-Bang | 186/200 | 5.2 s | 12/40 | 0 von 4 |
| Imitation | 182/200 | 3.7 s | 14/40 | 0 von 4 |
| PID | 180/200 | 7.3 s | 19/40 | 0 von 4 |
| Q-Learning | 156/200 | 4.7 s | 27/40 | 0 von 4 |
| **Kombination v1** | | | 149/150 (Median 2.3 s) | **16 von 16** |

Die Fail-Läufe der 6 anderen Methoden zeigen alle dasselbe Bild: Der Ball pendelt mit vollem Servoausschlag (±30°) um die Mitte. Sim v3 sagt dieses Verhalten richtig voraus.

## Was in der Kombination steckt

| Teil | Herkunft | Wirkung |
|---|---|---|
| Vorhersage über 0.45 s mit 7 Kandidaten-Winkeln, Bremsweg im Ziel | MPC | Kern |
| Online-Schätzung von Offset und Ballstörung (sin/cos über x) | MPC | grösster Gewinn gegen die schwere Seite |
| «Lernen beim Kleben»: bleibt der Ball liegen, wird die Störungsschätzung korrigiert | MPC | |
| Neue Parameter: K = 13, Bremsbeschl. 170, Servorampe 200 °/s, Beobachter 0.35, Lernrate 0.15 | Abstimmung auf Sim v3 | Sim, gleiche 60 Läufe: p90 von 14 auf 5.7 s |
| Losbrech-Stoss: steckt der Ball > 1.2 s mehr als 14 mm neben dem Ziel, wächst ein Zusatzwinkel | Golfer / Fuzzy | Sicherheitsnetz; in der Sim nie ausgelöst, echt nicht nachgewiesen |
| Ruhezone (Servo einfrieren) | Fuzzy | **getestet und verworfen**: Sim Median 4.3 statt 2.2 s |

Ehrliche Einordnung: Am echten Aufbau hat nur MPC funktioniert. Die «3 besten» hätten nichts gebracht, darum übernimmt die Kombination von den anderen nur einzelne Bausteine, die nachweislich helfen oder nicht schaden.

## Echte Läufe der Kombination (alle)

| Auftrag | Start | Zeiten (s) |
|---|---|---|
| 0013 | S, K, S, K, S, K | 4.6, 3.5, 4.5, 2.8, **16.1**, 2.3 |
| 0015 (A/B gegen MPC) | S, K, S, K | 3.4, 6.5, 1.6, 3.9 |
| 0016 (A/B gegen Variante v2) | S, K, S, K, S, K | 3.5, 2.5, 2.4, 1.3, 6.1, 3.6 |

S = Sensor-Ende, K = Klotz. Im selben Auftrag: reine MPC 3 von 4 (24.2 s, Fail, 1.3, 4.3), Variante v2 (Servorampe 140) 6 von 6, aber langsamer (Median 3.8 s).

Rechenzeit auf dem Pico: im Mittel 6.2 ms, höchstens 21 ms pro Schritt. Die Schleife (Sensor, 30 ms) hält das ein.

## Live-Vorführung

1. Diese Dateien auf den Pico laden (MicroPico: Upload): `vl53l0x.py`, `kennlinie.py`, `bob_core.py`, `bob.py`, `ctrl_mpc.py`, `ctrl_kombi.py`, `demo_kombi.py` (alle im Ordner `Pico_Programme`).
2. Ball mit dem Bleistiftpunkt waagrecht zu dir auf die Schiene legen.
3. `demo_kombi.py` öffnen, MicroPico **Run**. Ausgabe: `RUHE nach x.x s`.
4. Ball wegnehmen: Die Schiene geht waagrecht, das Programm wartet auf den nächsten Ball.

Wichtig: Vorher `pico_bridge.py` mit Ctrl+C beenden, sonst kann MicroPico den Port nicht öffnen.

`demo_kombi.py` einmal echt getestet (Auftrag 0017): Ball lag bei 192 mm, «RUHE nach 1.4 s», danach 25 s im Ziel gehalten.

Noch wenig getestet: Start an einer **von Hand gelegten** Zwischenposition (nur dieser eine Lauf). Die 16 Läufe oben starten alle an einem Ende.

## Schwächen und nächste Schritte

- 3 von 16 Läufen über 5 s, einer 16 s. Typisch: Der Ball steckt auf dem Weg in einer Mulde oder pendelt am Rand der Toleranz.
- Die Ballstörung hängt davon ab, wie der Ball liegt. Der Live-Test kann darum etwas anders ausfallen.
- Nächster sinnvoller Schritt: die gelernte Störungsschätzung von Lauf zu Lauf behalten (der Ball bleibt ja auf der Schiene), statt bei jedem Start bei null anzufangen. Nicht getestet.

## Dateien

| Ordner | Inhalt |
|---|---|
| `Pico_Programme/` | Endregler, Demo, alles für den Pico |
| `Simulation/` | Simulator v3, Identifikation (Fits), Abstimmungen |
| `Methoden_Agenten/` | Regler, Notizen, Abstimmung und Ergebnis jedes Methoden-Agenten, Auftrag an die Agenten |
| `Messskripte/schiene_messen.py` | Schienen-Karte und Scan (früher `karte.py`, `scan.py`) |
| `Bruecke/`, `Messdaten/pico_logs/` | Brücke PC ↔ Pico, alle Aufträge und Logs |

Jans Dateien (`Archiv/Jan_alt/main.py`, `Grundlagen/Methoden/`, `Grundlagen/Erkenntnisse-Versuchsaufbau.md`) sind inhaltlich unverändert.
