# Pico_Programme

Alle Dateien gemeinsam auf den Pico laden (Pico-Root, keine Unterordner). Imports sind einfache Modulnamen.

| Datei | Zweck | Braucht |
| --- | --- | --- |
| `main_v3.py` | **Live-Programm main_v3** (verbesserte main.py: PID mit Vorhalt, Haltewinkel, Losbrechen) | kennlinie, vl53l0x |
| `main_q.py` | **Live-Programm main_q** (Q-Learning wählt pro Startseite einen Regler) und Training über die Brücke (`main_q.training(...)`). Enthält das frühere `qlern.py` | main_v3, bob_core, kennlinie, vl53l0x, q_main.json; für Aktion A4 ctrl_mpc, ctrl_kombi; für das Training bob |
| `q_main.json` | Gelernte Q-Tabelle (Stand nach 84 echten Läufen, Auftrag 0028) | |
| `demo_kombi.py` | Live-Programm der Kombi (MPC). Echt 20 von 20, live von Jan als zu unruhig beurteilt | bob, bob_core, ctrl_kombi, ctrl_mpc |
| `ctrl_kombi.py` | Kombi-Regler (MPC neu abgestimmt + Losbrech-Stoss) | ctrl_mpc, bob_core |
| `ctrl_mpc.py` | MPC-Regler mit Online-Schätzung der Ballstörung | bob_core |
| `bob_core.py` | Kern für Pico und Simulator: Messung, Startvorbereitung, Bewertung «zur Ruhe» | kennlinie |
| `bob.py` | Hardware-Teil für Tests über die Brücke (`bob.reihe(...)`) | vl53l0x, bob_core |
| `kennlinie.py` | Sensorwert → echte mm (mit schwarzem Karton am Klotz) | |
| `vl53l0x.py` | Sensor-Treiber | |

Getestete Versionen, Prüfsummen siehe `Archiv/aufraeumen_2026-11-06.txt`.
Achtung: Dateien nicht in VS Code per Drag & Drop verschieben, VS Code schreibt dabei Imports auf `Ball_on_beam.…` um, und das läuft auf dem Pico nicht.
