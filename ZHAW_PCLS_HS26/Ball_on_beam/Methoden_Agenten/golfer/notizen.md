# Golfer-Impuls-Regelung (golfer), Stand 06.11., neue Sim mit schwerer Ballseite

## Idee
Ein Schlag kippt die Schiene Richtung Mitte, bis der Ball nach Plan (Bremswinkel × 0.45 plus Rollreibung) genau in der Mitte ausrollen würde. Danach bremst der Regler geführt: In jedem Schritt rechnet er die nötige Verzögerung neu (Nachkorrektur gegen falsches k, Reibung und Ballstörung). Nahe der Mitte (±30 mm, < 60 mm/s) übernimmt die Haltephase mit lernendem Haltewinkel, Feder, Dämpfung und Totband (Ball klebt in ±6 mm: Servo bleibt stehen); rollt der Ball über ±40 mm hinaus, folgt ein neuer Schlag.

## Geändert gegenüber dem alten MD und warum
- Kein Warten auf Ruhe vor dem Schlag und kein zeitgesteuerter Doppelpuls (U, T): Mit Störung ±50 bis 180 mm/s² ist ein offen gesteuerter Puls wertlos. Jetzt Schlag als Zustandsautomat mit geführtem Bremsen (Rückführung).
- Kein Nachlernen von A: Der Schlag korrigiert sich selbst, die Lerngrösse ist der Haltewinkel.
- Losbrechen: Klebt der Ball beim Stoss (< 20 mm/s), steigt der Stosswinkel um 40 °/s bis 30°. Ohne das blieb er in Mulden 30 mm neben der Mitte hängen (Spur Seed 16, Stoss −22° reichte nicht).
- Haltephase neu (fehlte im alten MD). Haltewinkel lernt nur bei < 30 mm/s.
- Rampe 300 °/s: < 20° in 120 ms ist damit nicht garantiert, aber 150 °/s war klar schlechter (40 Läufe: 9 bis 13 statt 16 bis 27 unter 5 s).
- Alte Zahlen (A = 60, U = 8°, V_REST 20) alle verworfen.

## Endparameter (Standardwerte in ctrl_golfer.py)
u_push 22, u_min 8, c_push 0.12 °/mm, u_brk 26, plan 0.45, k 8.5, mu 18, t_lag 0.08 s, rate 300 °/s,
e_hold 30, v_hold 60, e_out 40, kp 1.2 °/mm, kd 0.2 °s/mm, ki 0.8 °/(mm s), h_max 25, kick 0, e_db 6, v_stick 8,
v_learn 30, v_esc 20, r_esc 40, u_lim 30. Schätzer: Alpha-Beta 0.45/0.08 mit Ausreissertor 15 mm (max. 3 in Folge).

## Abstimmung (seed0 0 und 1000, n = 40/80)
kp/kd: höhere Dämpfung (kd 0.5 bis 0.8) war deutlich schlechter (n=40: 0 bis 18 Erfolge), vermutlich wegen Totzeit und verrauschtem Tempo. plan 0.45 und e_hold 30 auf beiden Seed-Sätzen am besten (74/80 und 72/80). Unterschiede zwischen den übrigen Kandidaten liegen im Rauschen (n=80: 67 bis 74).

## Validierung (seed0 = 100000, n = 200, einmalig)
Erfolg 190/200 | T median 3.75 s, p90 9.36 s, max 30.00 s | <=5 s: 141, <=10 s: 182 | Servo 57 °/s | Mess!=Wahr 5

| Start | n | Erfolg | T median |
|---|---|---|---|
| sensor | 50 | 48 | 4.04 s |
| klotz | 50 | 46 | 3.72 s |
| nah_sensor | 50 | 49 | 3.90 s |
| nah_klotz | 50 | 47 | 3.52 s |

## Nachkorrektur und Haltewinkel (gemessen, Abstimm-Seeds 0 bis 79)
- Nur 22 von 74 Erfolgen schaffen es mit einem Schlag, 49 brauchen 2 bis 6 Schläge (Median 2). Die Nachkorrektur ist also nötig und funktioniert meistens.
- Gelernter Haltewinkel am Ende: −22.5 bis +19.6° (Median −0.4°). Er gleicht also die lokale Ballstörung in der Mitte aus, nicht nur den Offset.
- Fails (6/80): 2 mit nur einem Schlag (Ball kriecht in der Haltephase langsam in eine Mulde knapp ausserhalb ±10 mm), 4 mit 14 bis 28 Schlägen (Ball pendelt zwischen zwei Mulden links und rechts der Mitte, die Mitte liegt auf einer «Kuppe» der Störung).

## Schwächen
- Kuppe in der Mitte (Störung stösst vom Ziel weg, Steigung bis ca. 11 mm/s² pro mm): Feder kp 1.2 reicht bei kleinem k nicht, stärkere Feder schwingt. Hauptursache der Fails und der p90.
- Kein Wissen über die Störung ausserhalb der Mitte (keine Fourier-Schätzung); Stösse werden darum oft zu kurz oder zu lang.
- Totband auf gemessener Position: Kennlinienfehler von einigen mm kann Ruhe ausserhalb ±10 mm vortäuschen (Mess!=Wahr 5/200).

## Rechenzeit
CPython in Sim-Läufen: Mittel 4.8 µs pro step (20 Läufe, 9258 Aufrufe). Ein Ausreisser 12 ms, vermutlich Planer/Konkurrenz der Maschine (Vermutung). Hochgerechnet × 100: ca. 0.5 ms auf dem Pico. Kein Speicherbedarf ausser ca. 30 Zahlen.

## Vorschlag erster echter Test
1. Standardwerte, Start Klotz und Sensor je 3×. Log: x, state, u, hold (Attribute `state`, `hold`).
2. Achten auf: Losbrechen (steigt u beim Stoss wirklich bis Losrollen?), Sensor-Fehlwerte nach dem Wechsel Stoss → Bremse (bis 48° in 0.16 s). Falls Fehlwerte: rate auf 200 senken.
3. Pendelt der Ball zwischen zwei Mulden: kp 1.6 versuchen, e_db auf 8.
4. Gelernten hold notieren: zeigt die lokale Neigung in der Mitte für diese Ballstellung.

## Kombination
Haltephase ist der Schwachpunkt: Eine Störungsschätzung (Fourier über x, z. B. aus MPC/PID-Agent) könnte den Haltewinkel vorsteuern statt ihn langsam zu lernen, und dem Bremsplan die Störung entlang des Wegs mitgeben. Umgekehrt kann der Schlag mit geführtem Bremsen und Losbrech-Rampe als schnelle Anfahrt vor einen anderen Halteregler gesetzt werden.
