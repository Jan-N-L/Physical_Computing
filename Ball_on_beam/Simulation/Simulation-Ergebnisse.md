# Ball on Beam: Simulationsergebnisse und Identifikation (06.11.2026)

Gesichert aus dem temporären Arbeitsordner von Claude. Alles hier ist **Simulation oder Auswertung echter Logs**, die echten Läufe stehen in `Messdaten/pico_logs/`.

## 1. Simulator-Versionen

| Version | Wann | Grundlage | Wichtigste Werte |
|---|---|---|---|
| v1 (alt, Übergabe-MD) | 05.11. | Annahmen | A = 60 mm/s² pro ° Servo, Reibung 0.2°, Schiene 400 mm, Sensor linear. **Unbrauchbar** |
| v2 | 06.11. nachts | Erkenntnisse-MD + Scan 1 und 2 | K 5.5 bis 12 mm/s² pro °, Ballstörung ±50 bis 180 mm/s² mit 92 bis 118 mm Rollweg pro Umdrehung, Verzögerung ca. 0.05 s |
| **v3** | 06.11. morgens | Regression über 16 echte Läufe (0006 bis 0009) | **K 10 bis 16**, Verzögerung (Totzeit 0.05 bis 0.09 s + Servo 0.025 bis 0.05 s), Offset 1.9 ±3°, Störung ±30 bis 180 mm/s², 60 bis 120 mm, Rollreibung 15 bis 35 mm/s² |

Code: `Simulation/sim.py` (v3). Bewertung gleich wie am Pico (`Pico_Programme/bob_core.py`), aber mit der **wahren** Position.

## 2. Identifikation aus echten Logs

| Quelle | Methode | Ergebnis |
|---|---|---|
| `0004_scan.log` (Ball beliebig) | Fit Bewegungsgleichung auf 2 Abschnitte, rms 5.4 mm | Störung ±157 mm/s², 109 mm pro Umdrehung, Rollreibung 13 mm/s² |
| `0005_scan_seitlich.log` (Punkt seitlich) | Fit 3 Abschnitte, rms 4.2 mm | K 6.1 (an der unteren Grenze), waagrecht bei +3.0° relativ, Störung ±86 mm/s², 96 mm, Reibung 11 mm/s² |
| 0006 bis 0009 (16 Regelläufe) | Lineare Regression der Beschleunigung (Savitzky-Golay) | **K 12.4 bis 15.2 mm/s² pro ° Servo**, Totzeit ca. 0.09 s + Servo ca. 0.03 s, Reibung ca. 28 mm/s² |

Skript: `Simulation/identifikation.py` (zusammengelegt aus `fit5.py` und `ident.py`, Ausgabe identisch; Aufruf `python identifikation.py alle`). Die Rollweg-Länge war in der Regression schlecht bestimmt (rms flach von 55 bis 85 mm).

## 3. Agenten-Validierung im Simulator v2 (n = 200, seed0 = 100000)

| Methode | Erfolg | Median | p90 |
|---|---|---|---|
| Fuzzy | 194/200 | 2.89 s | 5.46 s |
| MPC | 193/200 | 2.74 s | 6.08 s |
| Golfer | 190/200 | 3.75 s | 9.36 s |
| Bang-Bang | 186/200 | 5.24 s | 19.26 s |
| Imitation | 182/200 | 3.65 s | 24.51 s |
| PID | 180/200 | 7.25 s | 28.91 s |
| Q-Learning | 156/200 | 4.65 s | 30.00 s |

Echt kamen alle ausser MPC auf 0 von 4: v2 war zu schwach und zu schnell. Details und Notizen: `Methoden_Agenten/<methode>/`.

## 4. Einheitlicher Vergleich im Simulator v3 (n = 100, seed0 = 300000, gleiche Läufe)

Datei: `Simulation/sim_v3_vergleich.csv` (mit Werten pro Startart), erzeugt von `Simulation/sim_v3_vergleich.py`. Nach dem Aufräumen mit der neuen Ordnerstruktur nachgerechnet: identisch. Fail zählt als 30 s.

| Regler | Erfolg | Median | p90 | ≤ 5 s | Servo °/s |
|---|---|---|---|---|---|
| Kombi v1 (= main_q A4) | 100 | 2.37 s | 5.14 s | 88 | 97 |
| MPC (Agent) | 99 | 3.85 s | 13.08 s | 65 | 91 |
| main_q A3 | 98 | 5.98 s | 14.32 s | 41 | 101 |
| main_v3 Standard (= main_q A0) | 97 | 6.42 s | 14.91 s | 30 | 59 |
| main_q A1 | 96 | 4.59 s | 10.30 s | 54 | 129 |
| main_q A2 | 94 | 5.17 s | 11.61 s | 48 | 64 |
| main_q A5 | 91 | 7.21 s | 28.02 s | 38 | 127 |
| main.py alt (1:1 nachgebaut) | 81 | 12.94 s | 30 s | 28 | 30 |
| Q-Learning (Agent) | 55 | 11.57 s | 30 s | 22 | 58 |
| Fuzzy (Agent) | 49 | 30 s | 30 s | 31 | 76 |
| PID (Agent) | 44 | 30 s | 30 s | 9 | 121 |
| Imitation (Agent) | 30 | 30 s | 30 s | 7 | 170 |
| Bang-Bang (Agent) | 28 | 30 s | 30 s | 5 | 60 |
| Golfer (Agent) | 6 | 30 s | 30 s | 0 | 80 |

Die Reihenfolge passt zu den echten Tests (Kombi und main_v3 schaffen echt alle Läufe, die Agenten-Methoden ausser MPC keinen). main_v3 hier mit `PROG = True` (aktueller Stand der Datei).

## 5. Abstimmungen im Simulator v3

| Was | Ergebnis | Skript |
|---|---|---|
| MPC-Parameter (Koordinatensuche, 60 Läufe) | Kosten 13.3 → 6.0; Kontrolle 100 Läufe: 100/100, Median 2.17 s, p90 5.09 s → wurde Kombi v1 | `Simulation/Abstimmung/search_mpc.py`, `mpc_best.json` |
| Kombi: Losbrech-Stoss | im Simulator nie ausgelöst, Ergebnis unverändert | |
| Kombi: Ruhezone (Fuzzy) | schlechter: Median 4.3 statt 2.2 s → verworfen | |
| main_v3 (Koordinatensuche, 60 Läufe) | Kontrolle 150 Läufe: 140/150, Median 7.3 s (main.py alt 116/150, 9.8 s) | `Simulation/Abstimmung/tune_v3.py`, `best_v3.json` |
| main_q Kandidaten (45 Zufallssätze) | A1, A2, A3 gewählt; Kontrolle 100 Läufe: A0 7.1 s, A1 5.1 s, A2 4.7 s, A3 6.3 s | `Simulation/Abstimmung/kand.py`, `kand.json` |
| PROG (Kriech-Erkennung) | im Simulator kein Unterschied | |
| A5 (Ziel −8 mm) | im Simulator schlechter (54/60, 9.2 s) | |

## 6. Dateien

- `Archiv/Sicherungen/temp_sicherung_2026-11-06.zip`: der ganze temporäre Arbeitsordner von Claude (Simulator, alle Regler, Agenten-Ordner mit Logs, Fits, Kopien der echten Logs).
- `Archiv/Sicherungen/weitere_temp_dateien_2026-11-06.zip`: frühere Sitzung (phyphox-Rohdaten, Probeläufe mit nachgebildeter Hardware) und Hilfsskripte der Agenten.
- `Simulation/Abstimmung/`: Abstimm-Skripte und Ergebnisse von MPC/Kombi, main_v3 und der main_q-Kandidaten. Die Pfade in diesen Skripten zeigen noch auf den damaligen Arbeitsordner von Claude.
