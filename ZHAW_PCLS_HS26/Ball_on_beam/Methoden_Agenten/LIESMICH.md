# Methoden_Agenten

Regler der 7 Methoden-Agenten vom 06.11.2026 (je ein Agent, abgestimmt im Simulator v2), dazu Vergleichsregler.
Echt kam nur MPC auf Erfolge (10 von 14), alle anderen auf 0 von 4 (Logs `Messdaten/pico_logs/0006` bis `0012`).
Der MPC-Regler selbst liegt in `Pico_Programme/ctrl_mpc.py` (wird auch von der Kombi gebraucht).

| Datei / Ordner | Inhalt |
| --- | --- |
| `ctrl_pid.py`, `ctrl_bangbang.py`, `ctrl_fuzzy.py`, `ctrl_golfer.py`, `ctrl_imitation.py` (+ `_tab`), `ctrl_qlearn.py` (+ `_tab`) | Regler der Agenten, Interface `Regler.reset(x)` / `step(x, ok, dt)` |
| `ctrl_mainpid.py` | Jans altes `main.py` 1:1 als Regler-Klasse (Vergleich im A/B-Test 0020 und im Simulator) |
| `ctrl_demo.py` | Einfacher PD-Regler, erster Test der Kette (Auftrag 0002) |
| `<methode>/notizen.md` | Bericht des Agenten: Idee, Parameter, Validierung, Schwächen |
| `<methode>/result.json` | Validierung im Simulator v2 (n = 200) |
| `<methode>/tune.py` | Abstimm-Skript des Agenten. **Pfade zeigen auf den damaligen Arbeitsordner von Claude** (`/home/claude/bob/...`), zum Nachrechnen anpassen |
| `AGENT_BRIEF.md` | Auftrag an die Agenten |

Weitere Logs der Agenten (z.B. Q-Learning-Trainings) liegen in `Archiv/Sicherungen/temp_sicherung_2026-11-06.zip`.
