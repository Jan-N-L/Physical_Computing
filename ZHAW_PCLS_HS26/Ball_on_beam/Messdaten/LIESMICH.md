# Messdaten

## pico_logs/ (echte Läufe über die Brücke, 06.11.2026)

Pro Auftrag `<nr>_<name>.log` (Ausgabe des Pico) und `.done` (Status, Dauer). Zeilen:
`D,t,s,x,ok,u` = jede Messung (Zeit s, Sensorwert, mm, gültig, Servowinkel relativ zu 1.9°) ·
`R,modul,start,OK/FAIL,T_ms,x0,servoweg,verlassen,dauer,rechenzeit_mittel_us,rechenzeit_max_us` = ein Lauf ·
`EP,zustand,x0,aktion,OK/FAIL,T` = eine Q-Learning-Episode · `B`, `N`, `E` = Losbrechen bei Karte/Scan.

| Nr. | Inhalt | Ergebnis |
| --- | --- | --- |
| 0001 | Pico-Info | MicroPython 1.19.1 |
| 0002 | Einfacher PD (ctrl_demo) | 1/2 |
| 0003 | Schienen-Karte (karte) | Ball hängt bei ca. 190 mm |
| 0004 | Scan, Ball beliebig | Pendeln um 295 mm, Fit ±157 mm/s², 109 mm |
| 0005 | Scan, Punkt seitlich | Fit ±86 mm/s², 96 mm |
| 0006 | MPC | 4/4 |
| 0007 bis 0012 | Golfer, PID, Imitation, Bang-Bang, Fuzzy, Q-Learning (Agenten) | je 0/4 |
| 0013 | Kombi v1 | 6/6 |
| 0014 | MPC | 3/6 |
| 0015 | A/B Kombi gegen MPC | 4/4 gegen 3/4 |
| 0016 | A/B Kombi v1 gegen v2 | 6/6 gegen 6/6 |
| 0017 | demo_kombi live | Ruhe nach 1.4 s |
| 0018, 0019 | andere Sitzung, nicht gelaufen (Port nicht offen) | |
| 0020 | A/B main_v3, main.py alt, Kombi | 4/4, 2/4, 4/4 |
| 0021 | main_v3 Standard, KD 0.15, RATE 150 | 4/4, 4/4, 3/4 |
| 0022 bis 0025, 0028 | Q-Learning Runden 1, 2, 3a (abgebrochen), 3b, 4 | 82/84 |
| 0026 | main_v3 A1, Sensor-Starts, Diagnose | 3/3 |
| 0027 | A/B Kriech-Erkennung PROG | 4/4 gegen 4/4 |

Ablauf, Messbedingungen und Bewertung: Report «main_v3 und main_q» (Link in `Bericht/`).

## phyphox/

Handy-Messungen der Schienenneigung (Experiment «Neigung»), siehe `phyphox/LIESMICH.md`.
