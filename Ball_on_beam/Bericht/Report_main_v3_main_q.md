# Report: main_v3 und main_q (Q-Learning)

Ausführlicher Report mit Messbedingungen, allen echten Läufen, Diagrammen und Vergleich mit dem Simulator:
**https://claude.ai/artifact/7zHZ9xMPbTUbUDvQy3FkY1** (Claude-Doc, bearbeitbar, Export nach Word/PDF möglich)

## Kurzfassung (Stand 06.11.2026)

| Variante | Läufe echt | Erfolg | Median T (Fail = 30 s) | Läufe ≤ 5 s |
| --- | --- | --- | --- | --- |
| main_v3 (Standardwerte) | 8 | 8 | 7.5 s | 2 |
| main_q (Q-Learning über 6 Regler) | 84 | 82 | 8.2 s | 27 |

- Q-Learning hält die Zuverlässigkeit, ein Zeitgewinn ist nicht gesichert. Die Drift zwischen den Durchgängen ist grösser als der Unterschied zwischen den Aktionen.
- Engpass beim Start am Sensor: Der Ball bleibt in einer Mulde bei 180 bis 190 mm hängen, knapp ausserhalb der Toleranz.

## Wo die Daten liegen

| Was | Pfad |
| --- | --- |
| Programme | `Pico_Programme/main_v3.py`, `Pico_Programme/main_q.py` (enthält das frühere `qlern.py`), `Pico_Programme/q_main.json` |
| Logs | `Messdaten/pico_logs/0020` bis `0028` |
| Simulator-Vergleich | `Simulation/sim_v3_vergleich.csv`, `Simulation/Simulation-Ergebnisse.md` |
| Abstimmung main_v3, Kandidaten A1 bis A3 | `Simulation/Abstimmung/` |
