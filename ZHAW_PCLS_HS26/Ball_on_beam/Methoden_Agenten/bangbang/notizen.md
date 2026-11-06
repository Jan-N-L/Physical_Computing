# Bang-Bang mit Schaltkurve (`bangbang`), Version 2

## Idee in 3 Sätzen
Fern vom Ziel zeitoptimal: voll kippen (U = 25°) Richtung Mitte, auf der Bremskurve s = e − v·T_LEAD − v·|v|/(2·a_b) voll gegenkippen.
Nah (|e| < 25 mm, |v| < 80 mm/s) übernimmt ein Halteregler u = ui + KP·e − KD·v, dessen I-Anteil ui den nötigen Haltewinkel gegen Offset und Ballstörung lernt (stossfrei gestartet beim aktuellen Winkel).
Liegt der Ball 0.3 s ruhig in ±7 mm (|v| < 10 mm/s), wird der Winkel eingefroren (Totband): die Haftreibung hält ihn.

## Was gegenüber dem alten MD geändert wurde und warum
- **Winkel 3° → 25°.** Die Ballstörung allein entspricht A/K = 5 bis 33° Servo (A 50..180 mm/s², K 5.5..12). Mit 3° kommt der Ball aus keiner Mulde heraus. 25° ist fast der volle Stellbereich (+28.1 / −31.9 relativ).
- **Bremsbeschleunigung aus der Physik:** a_b = SAFETY·(K·U + MU) = 0.9·(8·25 + 15) ≈ 194 mm/s² mit K = 8 (Mitte von 5.5..12). Zeitoptimal Ende → Mitte (170 mm, a ≈ K·U − MU ≈ 185 mm/s², halb beschleunigen, halb bremsen): 2·√(170/185) ≈ 1.9 s. Gemessen braucht die Fernphase ca. 1.5 bis 2 s, der Rest ist Endphase.
- **Endphase neu:** Reines Bang-Bang mit Totband (altes MD, alter Entwurf) pendelt in der neuen Sim um die Mitte (alter Entwurf: 19/40 auf Seeds 0..39). Nötig ist lokale Steifigkeit K·KP grösser als die Störungssteigung (bis A·2π/Umfang ≈ 12 mm/s² pro mm) plus Haltewinkel-Lernen.
- **Rampe 4.5°/Schritt** (statt Sprung): hält Sprünge unter ca. 20° in 120 ms, also weniger Fehlwerte. Mit 8°/Schritt traten im Trace laufend Fehlwerte auf. Zusätzlich: 0.2 s nach Sprüngen ≥ 15° (in 4 Schritten) werden Messwerte mit > 20 mm Abweichung verworfen, höchstens 4 in Folge (rastet nie ein).
- Geprüft und **verworfen** (Standard aus): Störgrössenbeobachter mit Vorsteuerung −d/K (FF, Alpha-Beta-Gamma). Raster auf Seeds 1000..1023: mit KI 1.5 FF=1 14 bis 18/24, FF=0 18/24; ohne KI schlechter. Fangfenster mit stärkerer Dämpfung (E_IN, KD_IN): 29 bis 35/40 auf Seeds 1000..1039, kein Gewinn. Beide Optionen sind im Code noch schaltbar.

## Endparameter (Standardwerte in `ctrl_bangbang.py`)
U 25, K 8, MU 15, SAFETY 0.9, T_LEAD 0.12, RAMPE 4.5, E_NAH 25, V_NAH 80, U_NAH 26, KP 1.5, KD 0.7, KI 1.5, UI_MAX 26,
E_HOLD 7, V_HOLD 10, T_HOLD 0.3, E_OUT 10, T_OUT 0.3, V_OUT 40, ALPHA 0.5, BETA 0.15, GATE 20, T_SUSP 0.2, D_SPRUNG 15, FF 0.
Abstimmung: `tune.py` (Stufe 1, Seeds 2000..2039, Zeitbudget 15 min) und `tune.py stufe2`. Stufe 2 (U 22, E_NAH 40) hatte besseren Median (4.32 s), aber auf Kontroll-Seeds 20000..20119 schlechtere Zuverlässigkeit (107/120, p90 30 s) als Stufe 1 (108/120, p90 24.7 s). Darum Stufe 1. Logs: `tune_log_stufe1.txt`, `tune_log_stufe2.txt`.

## Validierung (seed0 = 100000, n = 200, einmal am Ende)
Erfolg 186/200 | T median 5.24 s, p90 19.26 s, max 30.00 s | <=5 s: 97, <=10 s: 171 | Servo 87 °/s | Mess!=Wahr 5

| Start | n | Erfolg | T median | T max |
|---|---|---|---|---|
| sensor | 50 | 47 | 5.95 s | 30 (Fail) |
| klotz | 50 | 47 | 5.86 s | 30 (Fail) |
| nah_sensor | 50 | 47 | 4.50 s | 30 (Fail) |
| nah_klotz | 50 | 45 | 4.49 s | 30 (Fail) |

## Schwächen
- 7 % Fails. Typischer Fehlerfall (Trace Seed 20009, K 8.1): Ball pendelt langsam (±25 mm, Periode ca. 4 s) durch die Mitte, wenn die Mitte auf einem «Hügel» der Ballstörung liegt. Häufig bei kleinem K (5.5 bis 6.6). Der Halteregler ist dort zu wenig steif bzw. zu träge gedämpft (Sättigung bei ±26°).
- Langer Schwanz: p90 19 s. Die Fernphase ist schnell, die Endphase dauert.
- Servo 87 °/s: viel Bewegung (Verschleiss, Lärm).
- Kein Lernen von K: a_b rechnet mit K = 8 fest.

## Rechenzeit
CPython 2.6 µs pro step (20 000 Aufrufe, perf_counter). Hochgerechnet (×50..100) ca. 0.13 bis 0.26 ms auf dem Pico. Nur Grundtypen, keine Tabellen, Speicher vernachlässigbar.

## Vorschlag für den ersten echten Test
1. Standardwerte, Start beim Sensor. Erst prüfen, ob die Fernphase vor der Mitte zu bremsen beginnt (Vorzeichen, a_b). Überschiesst er weit: SAFETY auf 0.7.
2. Beobachten: Fehlwerte nach Umschalten (+25 → −25 dauert mit Rampe ca. 0.33 s). Viele Ausreisser im Log: RAMPE auf 3.5.
3. Endphase: Pendelt der Ball langsam durch die Mitte, KP auf 2.0 und KD auf 1.0 (Raster Seeds 1000..1023 lag dort ebenfalls gut, 22/24).
4. Den eingefrorenen Winkel nach Ruhe loggen: das ist der lokale Haltewinkel (Offset + Ballstörung an dieser Stelle). Vermutung: streut je nach Ballstellung stark, nicht als globalen Offset speichern.

## Kombination mit anderen Methoden
Die Fernphase (Bang-Bang mit Rampe, ca. 1.5 bis 2 s bis zur Mitte) ist robust und als Anfahrteil für jede andere Methode brauchbar. Die Endphase ist die Schwachstelle: Vermutung, dass ein Regler mit Online-Schätzung der Ballstörung über x (Fourier sin/cos, siehe Brief) als Nahregler deutlich zuverlässiger wäre. Mein einfacher zeitbasierter Störbeobachter brachte es nicht (siehe oben); eine ortsbasierte Schätzung wurde nicht getestet.
