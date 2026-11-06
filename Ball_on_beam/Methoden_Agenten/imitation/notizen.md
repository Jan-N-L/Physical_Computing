# Imitations-Regelung (imitation), Stand 06.11., neue Sim mit unrundem Ball

## Idee in 3 Sätzen
Ein Lehrer fährt den Ball, der Pico merkt sich pro Situation (e, v, s) den Mittelwert des Lehrerwinkels in einer Tabelle (Behavioral Cloning). s = Standzeit: wie lange der Ball schon neben der Mitte klebt. Da kein Mensch verfügbar ist, ist der Lehrer ein **privilegierter Experte in der Simulation**, der Lage, Tempo, K, Offset, Reibung und die Ballstörung des Laufs exakt kennt und vorsteuert; der Schüler sieht davon nur die Messwerte.

## Geändert gegenüber altem MD und warum
- Lehrer: statt synthetischem Menschen ein Experte. Gemessen (n=80, seed0=0): synth. Mensch 8/80, PD+I (ctrl_demo_pd) 47/80, Experte 80/80. Ein Mensch-Modell mit ±15° Poti kommt gegen ±15° Ballstörung nicht an, als Lehrer unbrauchbar.
- **DAgger** statt nur Vormachen: der Schüler fährt selbst, der Experte sagt in jedem Schritt, was er getan hätte. Gemessen (n=80): nur BC 13/80, BC + 1 Runde (beta 0.5) 43/80, BC + 3 Runden (0.5, 0, 0) **73/80**. 5 Runden oder 200 statt 120 Episoden: 71 bis 72/80, keine Verbesserung.
- Merkmal s ist wesentlich: dieselbe Tabelle ohne s 67/80 statt 73/80. Grund (Deutung): DAgger sammelt genau die Lagen, in denen der Schüler in einer Mulde klebt; dort zeigt der Experte im Mittel "voll schieben". s ersetzt so teilweise das verborgene Wissen über die Störung.
- 3D-Tabelle e x v x s statt 9 x 7, trilinear gemischt (gelernte Fuzzy-Tabelle). Spiegeln aus (Offset im Mittel nicht symmetrisch; mit Spiegeln 72/80, gleichwertig).
- Rütteln (kurz zurück, dann voll schieben) als Zusatz getestet: 73/80 und 75/80 mit wie ohne. Bringt nichts, darum aus (`ruetteln=False`).

## Endparameter
Stützstellen E_N 15 (−170..170 mm), V_N 13 (−240..240 mm/s), S_N (0, 0.4, 0.8, 1.5 s); AlphaBeta a=0.5, b=0.15; V_STAND 20 mm/s, E_STAND 7 mm. Training: 120 Episoden BC + 3 x 120 DAgger (beta 0.5, 0, 0), Trainings-Seeds 50000+. Experte: Tempo-Profil v_ref = min(350, sqrt(2*150*|e|), 2*|e|), kv 6/s, exakte Störungs- und Reibungsvorsteuerung. Tabelle int8, Q = 0.25°, 780 B (Datei `ctrl_imitation_tab.py` 2.3 kB). 165 von 780 Stützstellen hatten zu wenig Daten und sind vom nächsten Nachbarn gefüllt.

## Validierung (n=200, seed0=100000, einmal am Ende)
| Regler | Erfolg | T med | T p90 | T max | <=5 s | Servo °/s |
|---|---|---|---|---|---|---|
| **Tabelle (Schüler)** | **182/200** | **3.65 s** | **24.5 s** | 30 (Fail) | 134 | 109 |
| Lehrer Experte (privilegiert, nur Sim) | 200/200 | 1.47 s | 1.95 s | 3.08 s | 200 | 20 |
| PD+I ctrl_demo_pd (einfacher Regler) | 111/200 | 22.2 s | 30 | 30 | 45 | 126 |

pro_start Tabelle: sensor 46/50 (med 3.1 s), klotz 48/50 (4.3 s), nah_sensor 43/50 (3.3 s), nah_klotz 45/50 (3.8 s).
Mess!=Wahr: 25 Läufe (Tabelle) gegen 0 (Experte), d.h. der Schüler zittert oft um die ±10 mm-Grenze.

**Ehrlich:** Die Tabelle ist viel besser als der einfache PD+I, aber klar schlechter als ihr Lehrer. Der Abstand (18 Fails, p90 24 s) ist genau das, was der Lehrer weiss und der Schüler nicht sieht: Störung des aktuellen Laufs und K. Gegenüber dem Lehrer bringt die Tabelle nur eines: sie läuft ohne privilegiertes Wissen. Rohdaten: `vergleich_validierung.json`, Abstimmen: `tune_log.txt`, `final_log.txt`.

## Schwächen (gemessen)
- Fails: Ball klebt bei Vollausschlag (u = +28 / −30) in einer Mulde, meist bei kleinem K (5.6 bis 6.4) und grosser Rollreibung; oder Grenzzyklus ±10 bis 15 mm um die Mitte.
- Servo 109 °/s: die Tabelle ist fast Bang-Bang (Werte springen nahe e = 0 zwischen ±25°). Gefahr Fehlmessungen nach grossen Servosprüngen.
- Tabelle aus der Sim gelernt: am echten Aufbau ist das ein Sim-zu-Real-Transfer ohne Lehrer. DAgger am echten Aufbau ist ohne Mensch nicht möglich.
- Ränder mit wenig Daten (z.B. s = 1.5 s bei e nahe 0) sind nur Nachbar-Füllung.

## Rechenzeit
CPython 5.4 µs pro step (20000 Aufrufe, perf_counter). Vermutung Pico: 0.3 bis 0.6 ms. RAM: Tabelle bleibt als bytes (780 B).

## Vorschlag erster echter Test
`ctrl_imitation.py` + `ctrl_imitation_tab.py` auf den Pico, `Regler()` mit Standardwerten, je 2 Läufe ab Sensor und Klotz. Achten auf: Servo-Zittern in der Mitte (hörbar, Fehlmessungen im Log?), Ball klebt in Mulde bei Vollausschlag, wie oft verlässt er ±10 mm nach dem Ruhe-Fenster. Zittert er stark: `skala=0.7` testen (in Sim nicht geprüft). Optional `ruetteln=True` (in Sim neutral).

## Kombination mit anderen Methoden
- Der ganze Abstand Schüler zu Lehrer ist die unbekannte Ballstörung. Ein Online-Schätzer (Fourier sin/cos über x + Offset) als zusätzliches Vorsteuer-Signal, und der Lehrer-Anteil `-d/K` in der Tabelle weggelassen, sollte viel davon schliessen (Vermutung, nicht gemessen).
- Der Experte selbst ist mit geschätzter statt wahrer Störung ein guter Kandidat für einen modellbasierten Regler; die Tabelle könnte dann als Rückfallebene dienen.
- Als Lehrer kann auch jeder andere Agenten-Regler dienen (tune.py: `lehrer_fabrik`), der Schüler wird aber nie besser als der Lehrer plus das, was s ihm verrät.
