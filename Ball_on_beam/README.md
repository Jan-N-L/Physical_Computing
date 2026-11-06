# Ball on Beam (Physical Computing HS26)

Raspberry Pi Pico H + VL53L0X + Servo: Ball von einem Ende in die Mitte (169.5 mm Ballvorderkante, Ballmitte ca. 19 cm) bringen und dort halten. Toleranz ±10 mm, «zur Ruhe» = 1 s durchgehend in der Toleranz, Fail ab 30 s.
Aufgeräumt am 06.11.2026. Protokoll aller Verschiebungen: `Archiv/aufraeumen_2026-11-06.txt`.

## Ordner

| Ordner | Inhalt |
| --- | --- |
| `Bericht/` | Ergebnisberichte und Link zum Report «main_v3 und main_q» |
| `Grundlagen/` | Erkenntnisse zum Versuchsaufbau (+ Nachtrag), PID-Kursunterlage |
| `Grundlagen/Methoden/` | Beschreibungen der 7 Regelmethoden, kreative Ansätze |
| `Pico_Programme/` | **Alles, was auf den Pico kommt** (flach, in einem Schritt hochladbar): main_v3, main_q, Kombi, Treiber |
| `Methoden_Agenten/` | Regler der 7 Methoden-Agenten mit Notizen und Abstimm-Skripten |
| `Messskripte/` | Messungen am Aufbau (Kennlinie, Rauschen, Neigung, Waagrecht, Schienen-Karte/Scan) |
| `Simulation/` | Simulator v3, Vergleich aller Regler, Identifikation, Abstimmungen |
| `Messdaten/pico_logs/` | Alle Logs der echten Läufe (Aufträge 0001 bis 0028) |
| `Messdaten/phyphox/` | Handy-Messungen der Schienenneigung |
| `Bruecke/` | `pico_bridge.py`: PC ↔ Pico für automatische Tests |
| `Kursmaterial/` | Unterlagen sw05 bis sw08 (unverändert, früher Ordner `pico/`) |
| `Archiv/` | Alte Versionen, Dateien vor dem Zusammenlegen, ZIP-Sicherungen |
| `_zu_loeschen/` | Nur exakte Kopien (gleiche Prüfsumme wie eine behaltene Datei). Kann gelöscht werden |

## Schnellstart

1. Alle Dateien aus `Pico_Programme/` mit MicroPico auf den Pico laden.
2. Ball mit dem Bleistiftpunkt waagrecht zu dir auf die Schiene legen.
3. `Pico_Programme/main_v3.py` oder `main_q.py` öffnen, MicroPico **Run**.

## Ergebnis in einem Satz

Echt: main_v3 8 von 8 Läufen (Median 7.5 s), main_q 82 von 84 (Median 8.2 s), Kombi-MPC 20 von 20 (Median 3.5 s, live zu unruhig). Grösste Störung: der Ball hat eine schwere Seite.
