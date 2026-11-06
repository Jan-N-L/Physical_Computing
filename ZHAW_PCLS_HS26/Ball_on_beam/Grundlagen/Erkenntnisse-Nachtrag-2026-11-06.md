# Nachtrag zu Erkenntnisse-Versuchsaufbau.md (06.11.2026, vormittags)

Messungen nach dem Erkenntnisse-MD, alle am echten Aufbau über die Brücke. Das Original bleibt unverändert, hier stehen nur Ergänzungen und Korrekturen.

| Grösse | Erkenntnisse-MD | Neu gemessen | Beleg |
| --- | --- | --- | --- |
| Ballbeschleunigung pro ° Servo | 10.1 mm/s² | **12.4 bis 15.2 mm/s²** im Regelbetrieb | Regression über 16 Regelläufe (0006 bis 0009), `Simulation/identifikation.py` |
| Verzögerung Befehl bis Wirkung | «kleine Verzögerung» | **ca. 0.12 s** (Totzeit 0.09 s + Servo 0.03 s) | gleiche Regression |
| Rollreibung | 25 mm/s² | 11 bis 28 mm/s² je nach Methode | Scan-Fit 11, Regression 28 |
| Unrunder Ball | periodische Störung, klein | **schwere Seite: ±90 bis ±160 mm/s²** (6 bis 15° Servo), ca. 100 mm Rollweg pro Umdrehung | Scan-Fits auf `0004_scan.log` (rms 5.4 mm) und `0005_scan_seitlich.log` (rms 4.2 mm) |
| Waagrecht | Servo +1.9° | nicht eindeutig: Fits ergaben −4 bis +5° Servo absolut, je nach Lauf und Ballstellung | Scan-Fit (+4.9°), Regression pro Log (−4.3 bis +1.5°) |

## Neue Befunde

1. **Schwere Seite des Balls.** Der Ball bleibt in «Mulden» liegen und schaukelt dort (Periode ca. 2 bis 3 s). Bei Tischtest markiert: Bleistiftpunkt waagrecht zur Bedienperson gelegt, damit der Schwerpunkt auf der Drehachse liegt. Das macht die Störung kleiner (157 → 86 mm/s²), aber nicht null.
2. **Start beim Sensor:** Der Ball schiesst auf ca. 195 bis 200 mm über das Ziel und bleibt dann oft 8 bis 12 s in einer Mulde bei 180 bis 190 mm hängen (Auftrag 0026).
3. **Start «Klotz»:** Beim Hinrollen mit 20° Neigung bleibt der Ball schon bei 274 bis 286 mm in einer Mulde (ab Auftrag 0020).
4. **Drift:** Derselbe Regler war eine Stunde später bis 3× langsamer (main_v3: Median 4.3 s in 0020, 11.3 s in 0021). Vermutlich dreht sich der Ball an den Anschlägen. Nur A/B-Tests im selben Auftrag sind vergleichbar.
5. **Rechenzeit Pico:** PID-artige Regler ca. 1 ms, MPC ca. 6 bis 8 ms pro Schritt (Schleife 30 ms).
6. **Ziel:** Klebband-Markierung = Ballmitte bei ca. 19 cm = Ballvorderkante 169.5 mm (Sensorwert ca. 206).
