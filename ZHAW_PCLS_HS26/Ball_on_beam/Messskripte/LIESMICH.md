# Messskripte

Messungen am Aufbau. Jans Skripte (früher `Scripts/Test/`) sind unverändert, dazu `schiene_messen.py`.

| Datei | Zweck | Status (laut Erkenntnisse-MD) |
| --- | --- | --- |
| `rauschtest.py` | Rauschen und Kennlinie messen | erledigt, bei Umbau wiederholen |
| `winkeltest.py` | Sensorwert abhängig vom Kippwinkel? | erledigt |
| `neigung.py`, `neigung_trend.py` | Handywerte / Servo-Treppe für phyphox | erledigt |
| `servo_bereich.py` | Servobereich ±30° abfahren | erledigt |
| `waagrecht_test.py`, `waagrecht_v3.py` | Offset mit rollendem Ball | nicht mehr verwenden |
| `test_offset.py` | Offset über Losrollen | nicht verwenden (misst Haftreibung) |
| `test_offset_sim.py`, `test_main_sim.py` | PC-Tests zu test_offset.py und altem main.py | alt |
| `Test.py` | Kurztest | |
| `schiene_messen.py` | `karte()` und `scan()`: Losbrechwinkel und Klemmstellen über die Schiene (Aufträge 0003 bis 0005). Zusammengelegt aus `karte.py` und `scan.py`, im Simulator identische Ausgabe | neu |

Auf dem Pico brauchen die Skripte `kennlinie.py` und `vl53l0x.py`, `schiene_messen.py` zusätzlich `bob.py` und `bob_core.py`. Alle liegen in `Pico_Programme/`.

**Achtung:** In `waagrecht_test.py` und `waagrecht_v3.py` hat VS Code beim Verschieben den Import zu `from Ball_on_beam.Scripts.Test.kennlinie import to_mm` umgeschrieben. Auf dem Pico muss es `from kennlinie import to_mm` heissen. Nicht korrigiert, weil die Skripte nicht mehr gebraucht werden.
