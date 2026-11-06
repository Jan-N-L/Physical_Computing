# PID (Kurzname pid), neuer Simulator mit unrundem Ball

## Idee in 3 Sätzen
PD auf gefilterte Position und Tempo (Alpha-Beta), mit Vorhalt t_lead gegen die Totzeit, Reibungskompensation in Bewegungsrichtung und I-Anteil nur nahe am Ziel (Anti-Windup).
Bleibt der Ball ausserhalb der Ruhezone hängen (Mulde des unrunden Balls), wächst ein Losbrech-Zusatz bis Vollausschlag; «Hängen» wird über fehlenden Fortschritt (< p_min mm Richtung Ziel) erkannt, nicht über das verrauschte Tempo.
In der Ruhezone gilt u = I, und I lernt dort langsam weiter (ki_hold), damit der Ball nicht aus dem Fenster wandert.

## Geändert gegenüber altem MD / main_pid_alt.py und warum
- Echte mm, Ziel 169.5 statt Sensorwert 200, Offset 1.9 statt 0.5 (Erkenntnisse).
- Stillstand-Erkennung über Fortschritt statt |v| < 12 mm/s: Das Tempo rauscht nahe Sensor um ca. 10 bis 15 mm/s, der Zähler wurde ständig zurückgesetzt, der Ball blieb in Mulden liegen (Spuren Seeds 2, 9 im Entwurf).
- kick_max 30 statt 12, i_max 12 statt 4: Die Ballstörung (bis 180 mm/s², K ab 5.5) verlangt bis zu Vollausschlag. Mit i_max 4 pendelte der Ball am Fensterrand (Seed 15). Messung n=40: i_max 12: 33/40 statt 22/40.
- ki_hold 0.3 (I lernt in der Ruhezone weiter): Suche 37/40 statt 36/40, Kosten 65.6 statt 78.8.
- kp 0.8, t_lead 0.1, fc 4 (statt 0.6, 0.05, 2): Ergebnis der Koordinatensuche (tune_log.txt).

## Endparameter (Standardwerte in pico/ctrl_pid.py)
kp 0.8, kd 0.55, ki 0.15, i_zone 25 mm, i_max 12°, ki_hold 0.3, alpha 0.55, beta 0.2, t_lead 0.1 s,
fc 4° (weich über v_fc 25 mm/s), kick0 3°, kick_rate 12°/s, kick_max 30°, p_min 4 mm, v_stuck 40 mm/s,
e_hold 5 mm, e_out 7 mm, n_out 3, v_hold 20 mm/s, du_max 6°/Schritt, u -31..+28, a_br 120, v_max 400.
Vorsteuerung (l3, g_ff), Schwung (t_sw) und Haltewinkel aus gemitteltem u (t_uf) sind im Code, aber AUS.

## Varianten (Kontrolle Seeds 50000.., n=120, gleiche Grundwerte)
| Variante | Erfolg | T median | p90 |
|---|---|---|---|
| Endwert | 113/120 | 6.90 | 20.72 |
| ohne Sollwertführung (a_br, v_max aus) | 114/120 | 6.90 | 20.72 |
| I überall (i_zone 1000) | 115/120 | 7.01 | 19.52 |
| Störschätzer + Vorsteuerung g_ff 0.5 | 107/120 | 7.19 | 30.00 |
| Störschätzer + Vorsteuerung g_ff 1.0 | 91/120 | 15.03 | 30.00 |
| stärkerer Filter (alpha 0.4, beta 0.12) | 111/120 | 7.23 | 22.35 |
| Schwung holen nach 1.5 s Hängen | 112/120 | 6.90 | 19.81 |
| Haltewinkel = gemitteltes u (t_uf 0.3) | 95/120 | 10.18 | 30.00 |
| kp 1.0, t_lead 0.15 | 112/120 | 8.26 | 22.13 |
- Sollwertrampe/-führung: praktisch keine Wirkung (Grenze greift erst bei sehr grossem e). I nur nahe Ziel vs. überall: im Rauschen (Suchseeds 38/40 vs. 35/40, Kontrolle 113 vs. 115), darum nahe Ziel belassen.
- Vorsteuerung der Ballstörung über einen einfachen Störbeobachter (dh als Zustand, Modell a = 8.5·u + dh): schlechter. Vermutung: Beschleunigung aus 34 Hz und 2 bis 4 mm Rauschen zu ungenau, K streut 5.5 bis 12. Eine Fourier-Schätzung über x wurde aus Zeitgründen NICHT geprüft.

## Validierung (seed0=100000, n=200, einmal, result.json)
Erfolg 180/200 | T median 7.25 s, p90 28.91 s, max 30.00 s | <=5 s: 71, <=10 s: 130 | Servo 81 °/s | Mess!=Wahr 17
| Start | n | Erfolg | T median |
|---|---|---|---|
| sensor | 50 | 46 | 8.67 |
| klotz | 50 | 48 | 6.53 |
| nah_sensor | 50 | 42 | 7.41 |
| nah_klotz | 50 | 44 | 6.61 |

## Schwächen
- 20 Fails. 12 davon mit K < 7.6 mm/s² pro ° (K-Bereich 5.5 bis 12): Bei schwacher Wirkung reicht der Stellbereich gegen die Ballstörung kaum, der Ball pendelt um die Mitte oder hängt in einer Mulde.
- p90 fast 29 s: Viele Läufe brauchen mehrere Anläufe (Ball schwingt durch das Fenster). Zuverlässigkeit ist das Hauptproblem, nicht der Median.
- Mess!=Wahr 17: Bewertung mit Messwert weicht in 17 Läufen von der wahren ab (Fensterrand ±10 mm).

## Rechenzeit
CPython 1.3 µs pro step (realistische Bahn, 30000 Aufrufe), 5 µs bei zufälligen Sprüngen. Pico (Faktor 50 bis 100, Vermutung): ca. 0.1 bis 0.5 ms.

## Vorschlag erster echter Test
- Standardwerte, je 2 Starts Sensor und Klotz. Log: x, u, Zustand hold.
- Achten auf: Schwingt der Ball mit ca. 2 s Periode um die Mitte (dann kd 0.7 oder kp 0.6 probieren)? Bleibt er ausserhalb hängen, und steigt u dann wirklich bis Vollausschlag (Losbrechen)? Wandert er in der Ruhezone (ki_hold)?
- Grosse u-Sprünge: du_max 6°/Schritt begrenzt auf ca. 200°/s, Fehlwerte nach Sprüngen trotzdem prüfen.

## Kombination
- Ankunft schneller und ruhiger mit einer Bahnplanung (Bang-Bang/Golfer) bis ca. 30 mm vor Ziel, danach dieser PID mit Ruhezone.
- Wenn eine andere Methode die Ballstörung zuverlässig schätzt (Fourier über x), diese als Vorsteuerung hier einsetzen; der einfache Zeitbereich-Beobachter half nicht.
