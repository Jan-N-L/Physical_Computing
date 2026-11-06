# Auftrag für Methoden-Agenten: Ball on Beam (Simulation)

Du stimmst EINE Regelungsmethode im gemeinsamen Simulator ab. Hardware-Tests macht der Hauptagent, nicht du.
Arbeite selbstständig bis zum Ergebnis. Sprache der Notizen: Deutsch, Schweizer Schreibweise (ss statt ß), keine Gedankenstriche.

## Ziel
Ball startet an einem Ende (0 mm beim Sensor oder 339 mm beim Klotz) oder irgendwo ausserhalb der Mitte und ruht.
Er muss in die Mitte (169.5 mm) und dort liegen bleiben.
- «Zur Ruhe» = durchgehend 1 s innerhalb ±10 mm. Gemeldete Zeit T = Beginn dieses Fensters.
- Danach 3 s weiter beobachten; verlässt er den Bereich, zählt das Fenster nicht.
- Fail: kein Ruhe-Fenster bis 30 s.
- Endziel des Projekts: 3 bis 5 s. Für dich: so schnell und so zuverlässig wie möglich. Zuverlässigkeit (Erfolg, T_p90, T_max) zählt mehr als der Median.

## Dateien (nicht verändern ausser deinen eigenen)
- `/home/claude/bob/pico/bob_core.py`: Kern (Messung mit Ausreisserfilter, Vorbereitung, Bewertung, Regler-Interface, AlphaBeta-Schätzer). Lies den Kopfkommentar.
- `/home/claude/bob/pico/kennlinie.py`: Sensorwert ↔ echte mm. bob_core rechnet schon um, dein Regler bekommt echte mm.
- `/home/claude/bob/sim/sim.py`: Simulator. `evaluate(make, n, seed0)` und `run_one(make, seed, start, keep_trace=True)`.
- `/home/claude/bob/docs/Erkenntnisse-Versuchsaufbau.md`: **Messungen am echten Aufbau. Massgebend.** Lies sie ganz.
- `/home/claude/bob/docs/<Methode>.md`: bisherige Beschreibung deiner Methode. **Achtung:** Die Zahlen darin stammen aus einem alten, falschen Modell (A = 60 mm/s² pro ° statt 10.1, Reibung 0.2° statt 2.5°, Schiene 400 statt 339 mm, linearer Sensor). Idee übernehmen, Parameter neu bestimmen.
- `/home/claude/bob/docs/Uebergabe-Claude-Regelung.md` Kap. 5 «Lernweg und Sackgassen»: Lehren aus dem alten Lauf.

Wenn du glaubst, der Simulator oder bob_core hat einen Fehler: NICHT ändern, sondern im Bericht melden (mit Beleg).

## NEU (06.11., echte Messungen über die Brücke): Ball mit schwerer Seite
Echte Tests (Logs in `/home/claude/bob/real/`: `0002_harness_demo.log` einfacher PD, `0003_karte.log`, `0004_scan.log`) zeigen:
- Der Ball hat einen Schwerpunkt neben der Mitte. Störbeschleunigung ca. **±160 mm/s² (≈ ±15° Servo!)**, sinusförmig über den Rollweg, **ca. 109 mm pro Umdrehung**. Fit auf den Scan: rms 5 mm.
- Folge: Ball bleibt in «Mulden» alle ca. 11 cm liegen und schaukelt dort (Periode ca. 2 s). Losrollen aus einer Mulde braucht 6 bis 19° Servo. Mit Schwung rollt er problemlos durch.
- Rollreibung laut Fit ca. 13 mm/s² (Sim jetzt 10 bis 30).
- Rollt der Ball ohne Rutschen, hängt die Störung nur von der Position ab (Phase pro Lauf unbekannt, Rollweg pro Umdrehung 100 bis 118 mm unbekannt). Ein Regler, der diese Störung online schätzt (z.B. 2 Fourier-Koeffizienten sin/cos über x, plus Offset) und vorsteuert, hat vermutlich einen grossen Vorteil. Bitte prüfen, nicht annehmen.
- Ein einfacher PD mit I-Anteil schafft in der neuen Sim nur ca. 20/40, am echten Aufbau 1/2 (Sensorstart Fail, Klotz 11.4 s).
- Zweiter Scan (Ball von Jan mit der schweren Seite seitlich gelegt, `0005_scan_seitlich.log`): Störung kleiner, aber noch da. Fit (rms 4.2 mm): K = 6.1 mm/s² pro ° Servo (!), waagrecht bei u = +3.0 (also absolut ca. 4.9°), Störung ±86 mm/s², 96 mm pro Umdrehung, Rollreibung 11 mm/s². Nahe beim Sensor (25 bis 50 mm) hielt er sogar gegen −15° Servo.
- Darum streut der Simulator jetzt breit: K 5.5 bis 12, Offset wahr 0.4 bis 5.4° absolut (Harness rechnet mit 1.9), Störung 50 bis 180 mm/s², 92 bis 118 mm, Rollreibung 8 bis 30.
- Der Simulator `sim.py` wurde entsprechend angepasst (Parameter in `Plant.__init__`). Er bildet das Hängenbleiben im Scan qualitativ nach.

Aus einem ersten, abgebrochenen Durchgang existieren evtl. Entwürfe von dir: `/home/claude/bob/pico/ctrl_<kurzname>.py` und `/home/claude/bob/methods/<kurzname>/`. Sie wurden mit dem alten Simulator (fast ohne Ballstörung) gemacht. Weiterverwenden erlaubt, aber neu abstimmen.

## Wichtige Physik (aus Erkenntnisse)
- Positives u → Ball rollt weg vom Sensor (x steigt). u ist relativ zum Offset 1.9° (wahrer Offset zufällig ±1.5°).
- Ca. 10 mm/s² pro ° Servo (±20 %), Rollreibung 15 bis 35 mm/s² (≈ 2.5° Servo), Haftreibung grösser.
- Stellbereich absolut ±30° Servo. Messung 34 Hz, nahe Sensor gröber, Fehlwerte nach grossen Servosprüngen (≥ 20° in 120 ms).
- Unrunder Ball (siehe oben, gross!) und krumme Stäbe (klein): ortsabhängige Störungen. Kein Regler darf einen exakten Offset voraussetzen.
- Echt gemessen: Schleife 30 ms, Rechenzeit des einfachen PD auf dem Pico im Mittel 0.8 ms.

## Was du ablieferst
1. `/home/claude/bob/pico/ctrl_<kurzname>.py` mit `class Regler` (`__init__(self, **param)` mit Standardwerten = beste Werte, `reset(x)`, `step(x, ok, dt)` → u in Grad).
   - **Muss auf MicroPython (Raspberry Pi Pico, RP2040, 264 kB RAM) laufen:** nur `math`, `bob_core`, `kennlinie`, Grundtypen. Kein numpy, keine dataclasses, kein typing, kein `random` nötig. Tabellen klein halten (z.B. `bytes`/`array`, < 20 kB). Grosse Tabellen als eigene Datei `ctrl_<kurzname>_tab.py` erlaubt.
   - Rechenzeit: MicroPython ist ca. 50 bis 100× langsamer als CPython. Ziel < 3 ms pro step auf dem Pico, also grob < 30 µs in CPython. Miss es (`time.perf_counter` über viele Aufrufe) und gib es an.
2. `/home/claude/bob/methods/<kurzname>/tune.py`: reproduzierbares Abstimm-Skript.
3. `/home/claude/bob/methods/<kurzname>/result.json`: `evaluate(..., n=200, seed0=100000)['stats']` deiner Endversion (nur `stats`, ohne `runs`).
4. `/home/claude/bob/methods/<kurzname>/notizen.md` (kurz, max. ca. 60 Zeilen): Idee in 3 Sätzen, was gegenüber dem alten MD geändert wurde und warum, Endparameter, Ergebnis Validierung (Tabelle inkl. pro_start), Schwächen, Rechenzeit, **Vorschlag für den ersten echten Test** (welche Parameter, worauf achten), was eine Kombination mit anderen Methoden bringen könnte.

## Regeln
- **Abstimmen nur mit seed0 < 100000.** Die Validierung `seed0=100000, n=200` erst ganz am Ende einmal, Zahlen ehrlich übernehmen. Nicht auf die Validierung nachjustieren.
- Rechenbudget: Die Maschine hat 2 Kerne für 7 Agenten. Kein multiprocessing. Ein `evaluate(n=40)` dauert ca. 1 s allein, mit Konkurrenz länger. Begrenze dich auf ca. 30 min Laufzeit.
- Jede Behauptung in den Notizen messen, nicht vermuten. Vermutungen als «Vermutung» kennzeichnen.
- Keine Hardware-Behauptungen: Alles ist Simulation.
- Schliesse mit einer kurzen Antwort (max. 15 Zeilen): Dateinamen, Validierungs-summary-Zeile, Rechenzeit, wichtigste Schwäche.
