# MPC (Vorhersage-Regelung) mit Störschätzung, Stand 06.11. (neue Sim mit unrundem Ball)

## Idee in 3 Sätzen
Ein Beobachter schätzt x, v und die ortsabhängige Störung d(x) = c0 + A·sin(2πx/λ) + B·cos(2πx/λ) (c0 = Offsetfehler, A/B = unrunder Ball) aus der Vorhersage-Innovation (LMS).
Pro Schritt werden 7 Winkel u = u_halt + du (u_halt = geschätztes Waagrecht am Ziel) über 3 × 0.15 s mit Reibung, Haftreibung und dieser Störung durchgerechnet; Kosten = Abstand zum Ziel + Haltepunkt bei Vollbremsung.
Klebt der Ball, obwohl das Modell Losrollen erwartet, wird d(x) an dieser Stelle Richtung Haftgrenze korrigiert («Haft-Lernen»).

## Geändert gegenüber altem MD und erstem Entwurf (alles gemessen, seed0 < 100000)
- Kandidaten relativ zum geschätzten Waagrecht, Modell mit Roll-/Haftreibung, Servo-Totzeit/-Trägheit, Vorausrechnen um tpre = 0.05 s, Anschläge. K = 10 statt A = 60.
- **Servo-Ratenbegrenzung 140 °/s**: ohne sie springt der Regler ±30° und löst laufend die Sensor-Fehlwerte (≥ 20° in 120 ms) aus. Gemessen (n=40, seed0=0): ohne Begrenzung 16 bis 21/40, mit 23/40.
- **Haft-Lernen** (gs=0.3): 23/40 → 37/40 (n=40, seed0=0). Grösster Einzelgewinn.
- Kleine Lerngewinne g0 = g1 = 0.003 statt 0.006/0.012 (n=60, seed0=2000: 60/60, p90 6.1 s, max 10 s gegenüber 52/60).
- Haltemodus (Totband) getestet: ehold = 6 schlechter als ehold = 0 (16 vs 21/40, frühe Version). Endversion ohne Haltemodus.

## Lohnt die Fourier-Schätzung? (geprüft, n=120, seed0=3000)
| Variante | Erfolg | T median | p90 | max |
|---|---|---|---|---|
| mit sin/cos (harm=1) | 118/120 | 2.51 s | 6.15 s | 30 |
| nur Offset c0 (harm=0) | 100/120 | 3.50 s | 30 | 30 |
Ja, deutlich. λ fest 105 mm: 98 und 112 gaben in einer frühen Version ähnliche Werte (55 und 54/60 vs 52/60), also unkritisch.

## Endparameter (Standardwerte in ctrl_mpc.py)
K=10, mu=15, mus=22, lam=105, du=(-30,-14,-6,0,6,14,30), h=0.15, nh=3, wt=2, abrk=120, tpre=0.05, tau=0.03, dead=0.012,
l1=0.45, l2=0.15, g0=0.003, g1=0.003, vb=25, gate=15, cmax=250, amax=220, rate_u=140, gs=0.3, nst=3, fs=0.7, ehold=0.
Empfindlich (n=120, seed0=3000): K=12 116/120 p90 7.9; g1=0.0015 117/120 p90 7.2; gs=0.5 117/120 p90 7.9. Zu grosse Lerngewinne (g1=0.02) klar schlechter (48/60).

## Validierung (n=200, seed0=100000, einmal am Ende)
Erfolg 193/200 | T median 2.74 s, p90 6.08 s, max 30.00 s | <=5 s: 163, <=10 s: 192 | Servo 86 °/s | Mess!=Wahr 0

| Start | n | Erfolg | T median | T max |
|---|---|---|---|---|
| sensor | 50 | 48 | 2.88 s | 30 |
| klotz | 50 | 48 | 2.69 s | 30 |
| nah_sensor | 50 | 48 | 2.68 s | 30 |
| nah_klotz | 50 | 49 | 3.01 s | 30 |

## Schwächen
- Fails (angeschaut: seed 3008, 3082) sind Läufe mit K ≈ 5.5 und Störung ≈ 174 mm/s²: Der Ball hängt in einer Mulde 10 bis 30 mm neben dem Ziel, Servo am Anschlag (K·28 < Störung + Haftreibung). Mit konstanten Kandidaten kann der Planer kein «Schaukeln» planen. Ein einfacher Schaukel-Ausweg (Gegenschwung nach Stillstand am Anschlag) wurde versucht, griff aber nie (Auslösebedingung zu eng, Zeit fehlte) und ist nicht drin.
- Servo-Weg 86 °/s: unruhig, Planer wechselt zwischen Nachbarkandidaten. Kein Totband.
- K, mu, mus sind fest; die Schätzung fängt Fehler nur über d(x) auf. Rollweg pro Umdrehung fest 105 mm.

## Rechenzeit
CPython 14 bis 28 µs pro step (timing.py, Last der Maschine schwankt), knapp unter dem Ziel 30 µs. Pro Schritt 7 × 3 sin plus 3 sin/cos/sqrt/atan2.
Vermutung: Pico 1.5 bis 3 ms (RP2040 ohne FPU, sin in Software). Vor dem ersten Test mit ticks_us messen; falls zu langsam: du auf 5 Kandidaten oder nh=2.

## Vorschlag erster echter Test
- Standardwerte. Zuerst Ball nah der Mitte, dann Sensorende, dann Klotz. Logge c0, A, B, uc (Attribute des Reglers) pro Schritt.
- Achten auf: (1) Rechenzeit < 3 ms; (2) Sensor-Fehlwerte nach Servosprüngen (sonst rate_u auf 110 senken); (3) ob √(A²+B²) gegen die Scan-Werte 86 bis 160 mm/s² läuft und c0 plausibel bleibt; (4) hängt der Ball neben dem Ziel mit Servo am Anschlag: dann ist das die bekannte Schwäche.
- Mit dem seitlich gelegten Ball (K ≈ 6) könnte K=7 besser passen (in Sim nicht geprüft).

## Kombination mit anderen Methoden
- Die Störschätzung (c0, A, B mit Haft-Lernen) ist unabhängig vom Planer und könnte als Vorsteuerung u_ff = -d(x)/K in PID/Fuzzy/Bang-Bang eingebaut werden (Vermutung, nicht gemessen).
- Schaukel-Strategie aus Golfer/Bang-Bang für die Mulden-Fails.
