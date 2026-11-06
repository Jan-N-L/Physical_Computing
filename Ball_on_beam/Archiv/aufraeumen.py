# aufraeumen.py: Ball_on_beam in die neue Ordnerstruktur bringen (06.11.2026).
# Löscht NICHTS. Verschiebt nur. Exakte Kopien (gleiche Prüfsumme wie eine behaltene Datei)
# kommen nach _zu_loeschen/, nicht zuordenbare Dateien nach Archiv/unzugeordnet/.
#   python3 aufraeumen.py <Ball_on_beam-Pfad>              Probelauf (zeigt nur an)
#   python3 aufraeumen.py <Ball_on_beam-Pfad> ausfuehren   verschiebt und schreibt das Protokoll
import glob
import hashlib
import os
import sys

BASE = os.path.abspath(sys.argv[1])
GO = len(sys.argv) > 2 and sys.argv[2] == 'ausfuehren'
STAGING = '_staging_neu'
LOG = 'Archiv/aufraeumen_2026-11-06.txt'
DUP = '_zu_loeschen'


def md5(rel):
    h = hashlib.md5()
    with open(os.path.join(BASE, rel), 'rb') as f:
        h.update(f.read())
    return h.hexdigest()


def rel_files(pattern):
    out = []
    for p in glob.glob(os.path.join(BASE, pattern), recursive=True):
        if os.path.isfile(p):
            out.append(os.path.relpath(p, BASE).replace(os.sep, '/'))
    return sorted(out)


M = []                                         # (quelle, ziel)


def mv(src, dst):
    M.append((src, dst))


def mv_dir(pattern, src_prefix, dst_prefix):
    for f in rel_files(pattern):
        mv(f, dst_prefix + f[len(src_prefix):])


# --- neue und angepasste Dateien aus dem Staging
for f in rel_files(STAGING + '/**/*'):
    mv(f, f[len(STAGING) + 1:])

# --- Bericht / Grundlagen
mv('Ergebnis-Regelung-Kombi.md', 'Archiv/vor_aufraeumen/Ergebnis-Regelung-Kombi.md')
mv('Erkenntnisse-Versuchsaufbau.md', 'Grundlagen/Erkenntnisse-Versuchsaufbau.md')
mv('PID-Regelung.md', 'Grundlagen/PID-Regelung.md')
for f in ['Bang-Bang-Regelung.md', 'Fuzzy-Regelung.md', 'Golfer-Impuls-Regelung.md', 'Imitations-Regelung.md',
          'Q-Learning-Regelung.md', 'Vorhersage-Regelung-MPC.md', 'Gebogene-Schiene.md', 'Kreative-Ansaetze.md']:
    mv(f, 'Grundlagen/Methoden/' + f)
mv('Uebergabe-Claude-Regelung.md', 'Archiv/Uebergabe-Claude-Regelung.md')

# --- Pico_Programme
for f in ['main_v3.py', 'q_main.json', 'kennlinie.py', 'vl53l0x.py', 'bob_core.py', 'ctrl_mpc.py', 'ctrl_kombi.py']:
    mv('Scripts/' + f, 'Pico_Programme/' + f)
mv('bridge/pico_files/bob.py', 'Pico_Programme/bob.py')
mv('bridge/pico_files/demo_kombi.py', 'Pico_Programme/demo_kombi.py')
mv('Scripts/main_q.py', 'Archiv/vor_zusammenlegung/main_q.py')
mv('Scripts/qlern.py', 'Archiv/vor_zusammenlegung/qlern.py')

# --- Methoden_Agenten
for f in ['pid', 'bangbang', 'fuzzy', 'golfer', 'imitation', 'imitation_tab', 'qlearn', 'qlearn_tab', 'demo']:
    mv('Claude_Arbeit/pico/ctrl_%s.py' % f, 'Methoden_Agenten/ctrl_%s.py' % f)
mv('Claude_Arbeit/main_v3_q/ctrl_mainpid.py', 'Methoden_Agenten/ctrl_mainpid.py')
mv('Claude_Arbeit/sim/AGENT_BRIEF.md', 'Methoden_Agenten/AGENT_BRIEF.md')
for m in ['pid', 'bangbang', 'fuzzy', 'golfer', 'imitation', 'qlearn', 'mpc']:
    for f in ['notizen.md', 'result.json', 'tune.py']:
        mv('Claude_Arbeit/methods/%s/%s' % (m, f), 'Methoden_Agenten/%s/%s' % (m, f))

# --- Messskripte
for f in ['Test.py', 'neigung.py', 'neigung_trend.py', 'rauschtest.py', 'servo_bereich.py', 'test_main_sim.py',
          'test_offset.py', 'test_offset_sim.py', 'waagrecht_test.py', 'waagrecht_v3.py', 'winkeltest.py']:
    mv('Scripts/Test/' + f, 'Messskripte/' + f)
mv('Claude_Arbeit/pico/karte.py', 'Archiv/vor_zusammenlegung/karte.py')
mv('Claude_Arbeit/pico/scan.py', 'Archiv/vor_zusammenlegung/scan.py')

# --- Simulation
mv('Claude_Arbeit/sim/sim.py', 'Archiv/vor_aufraeumen/sim.py')
mv('Claude_Arbeit/sim/fit5.py', 'Archiv/vor_zusammenlegung/fit5.py')
mv('Claude_Arbeit/sim/ident.py', 'Archiv/vor_zusammenlegung/ident.py')
mv('Claude_Arbeit/ergebnisse/Simulation-Ergebnisse.md', 'Archiv/vor_aufraeumen/Simulation-Ergebnisse.md')
mv('Claude_Arbeit/ergebnisse/sim_v3_vergleich.py', 'Archiv/vor_aufraeumen/sim_v3_vergleich.py')
mv('Claude_Arbeit/ergebnisse/sim_v3_vergleich.csv', 'Simulation/sim_v3_vergleich.csv')
mv('Claude_Arbeit/ergebnisse/sim_v3_vergleich.json', 'Simulation/sim_v3_vergleich.json')
mv('Claude_Arbeit/methods/search_mpc.py', 'Simulation/Abstimmung/search_mpc.py')
mv('Claude_Arbeit/methods/mpc_best.json', 'Simulation/Abstimmung/mpc_best.json')
for f in ['tune_v3.py', 'kand.py', 'kand.json', 'best_v3.json']:
    mv('Claude_Arbeit/main_v3_q/' + f, 'Simulation/Abstimmung/' + f)

# --- Messdaten
mv_dir('bridge/results/*', 'bridge/results/', 'Messdaten/pico_logs/')
mv_dir('Claude_Arbeit/messdaten/phyphox/*', 'Claude_Arbeit/messdaten/phyphox/', 'Messdaten/phyphox/')

# --- Bruecke
mv('bridge/pico_bridge.py', 'Archiv/vor_aufraeumen/pico_bridge.py')
mv('bridge/auswertung.py', 'Bruecke/auswertung.py')
mv('bridge/status.txt', 'Bruecke/status.txt')
mv_dir('bridge/jobs/done/*', 'bridge/jobs/done/', 'Bruecke/jobs/done/')

# --- Kursmaterial (unverändert)
mv_dir('pico/**/*', 'pico/', 'Kursmaterial/')

# --- Archiv
for f in ['main.py', 'V1.py', 'V2_Vor_Jan.py']:
    mv('ALT/' + f, 'Archiv/Jan_alt/' + f)
for f in ['bob.py', 'bob_core.py', 'ctrl_kombi.py', 'ctrl_mpc.py', 'demo_kombi.py']:
    mv('ALT/Kombi/' + f, 'Archiv/Kombi_mit_VSCode_Imports/' + f)
for f in ['ctrl_ruhig.py', 'ctrl_hybrid.py']:
    mv('Claude_Arbeit/main_v3_q/' + f, 'Archiv/andere_Sitzung/' + f)
for f in ['temp_sicherung_2026-11-06.zip', 'weitere_temp_dateien_2026-11-06.zip']:
    mv('Claude_Arbeit/' + f, 'Archiv/Sicherungen/' + f)

# --- Prüfen
fehler = []
quellen = set()
ziele = set()
for s, d in M:
    if not os.path.isfile(os.path.join(BASE, s)):
        fehler.append('FEHLT Quelle: ' + s)
    if s in quellen:
        fehler.append('DOPPELT Quelle: ' + s)
    if d in ziele:
        fehler.append('DOPPELT Ziel: ' + d)
    if os.path.exists(os.path.join(BASE, d)) and d not in [x for x, _ in M]:
        fehler.append('ZIEL EXISTIERT: ' + d)
    quellen.add(s)
    ziele.add(d)

behalten = {}                                   # md5 -> Ziel
for s, d in M:
    if os.path.isfile(os.path.join(BASE, s)):
        behalten.setdefault(md5(s), d)

rest = [f for f in rel_files('**/*') if f not in quellen and not f.startswith(DUP + '/')
        and f not in ('README.md',) and not f.startswith('Archiv/')]
kopien = []
unklar = []
for f in rest:
    h = md5(f)
    if h in behalten:
        kopien.append((f, DUP + '/' + f, behalten[h]))
    else:
        unklar.append((f, 'Archiv/unzugeordnet/' + f))

print('Verschiebungen: %d, exakte Kopien: %d, unzugeordnet: %d, Fehler: %d' % (len(M), len(kopien), len(unklar), len(fehler)))
for x in fehler:
    print('  ' + x)
for s, d, k in kopien:
    print('  KOPIE  %s  (gleich wie %s)' % (s, k))
for s, d in unklar:
    print('  UNKLAR %s -> %s' % (s, d))
if fehler:
    print('Abbruch wegen Fehlern, nichts verschoben.')
    sys.exit(1)
if not GO:
    print('Probelauf, nichts verschoben.')
    sys.exit(0)

log = []
for s, d in M + [(a, b) for a, b, _ in kopien] + unklar:
    h = md5(s)
    ziel = os.path.join(BASE, d)
    os.makedirs(os.path.dirname(ziel), exist_ok=True)
    os.rename(os.path.join(BASE, s), ziel)
    if md5(d) != h:
        print('PRÜFSUMME FALSCH nach Verschieben: ' + d)
    log.append('%s  %s  ->  %s' % (h, s, d))

# leere alte Ordner nach _zu_loeschen/leere_Ordner (löschen ist nicht erlaubt)
for top in ['Scripts', 'ALT', 'bridge', 'Claude_Arbeit', 'pico', STAGING]:
    p = os.path.join(BASE, top)
    if os.path.isdir(p) and not any(os.path.isfile(os.path.join(r, f)) for r, _, fs in os.walk(p) for f in fs):
        os.makedirs(os.path.join(BASE, DUP, 'leere_Ordner'), exist_ok=True)
        os.rename(p, os.path.join(BASE, DUP, 'leere_Ordner', top))
        log.append('LEERER ORDNER  %s  ->  %s/leere_Ordner/%s' % (top, DUP, top))

with open(os.path.join(BASE, DUP, 'LIESMICH.md'), 'w', encoding='utf-8') as f:
    f.write('# _zu_loeschen\n\nNur exakte Kopien: Jede Datei hier hat dieselbe Prüfsumme wie eine behaltene Datei '
            '(siehe Liste). Dazu leere alte Ordner. Der ganze Ordner kann gelöscht werden.\n\n')
    for s, d, k in kopien:
        f.write('- `%s` = `%s`\n' % (s, k))
with open(os.path.join(BASE, LOG), 'w', encoding='utf-8') as f:
    f.write('Aufräumen Ball_on_beam, 06.11.2026. Prüfsumme (md5)  alt  ->  neu\n\n' + '\n'.join(log) + '\n')
print('Fertig: %d Einträge im Protokoll %s' % (len(log), LOG))
