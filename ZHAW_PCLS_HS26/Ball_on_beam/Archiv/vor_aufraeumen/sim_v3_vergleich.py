# Einheitlicher Vergleich aller Regler im Simulator v3 (n=100, seed0=300000, gleiche Läufe für alle)
import sys, json, csv, importlib, time
sys.path.insert(0,'/home/claude/bob/ruhig'); sys.path.insert(1,'/home/claude/bob/pico'); sys.path.insert(0,'/home/claude/bob/sim')
import sim, main_v3, qlern
R=[('PID (Agent)','ctrl_pid',None),('Bang-Bang (Agent)','ctrl_bangbang',None),('Fuzzy (Agent)','ctrl_fuzzy',None),
   ('Golfer (Agent)','ctrl_golfer',None),('Imitation (Agent)','ctrl_imitation',None),('Q-Learning (Agent)','ctrl_qlearn',None),
   ('MPC (Agent)','ctrl_mpc',None),('Kombi v1 = main_q A4','ctrl_kombi',None),('main.py alt (1:1)','ctrl_mainpid',None),
   ('main_v3 Standard = main_q A0','main_v3',{}),('main_q A1','main_v3','A1'),('main_q A2','main_v3','A2'),('main_q A3','main_v3','A3'),('main_q A5','main_v3','A5')]
acts={'A1':qlern.ACTIONS[1],'A2':qlern.ACTIONS[2],'A3':qlern.ACTIONS[3],'A5':qlern.ACTIONS[5]}
out=[]
for name,mod,kw in R:
    m=importlib.import_module(mod)
    p=acts.get(kw,kw) if isinstance(kw,str) else (kw or {})
    t0=time.time()
    r=sim.evaluate(lambda: m.Regler(**p), n=100, seed0=300000); s=r['stats']
    row={'regler':name,'erfolg':s['erfolg'],'n':s['n'],'T_median':round(s['T_median'],2),'T_p90':round(s['T_p90'],2),'unter_5s':s['unter_5s'],'unter_10s':s['unter_10s'],'servo_grad_pro_s':round(s['servo_weg_pro_s'],1)}
    for g,v in s['pro_start'].items(): row['median_'+g]=round(v['T_median'],2); row['erfolg_'+g]=v['erfolg']
    out.append(row); print(name, r['summary'], '%.0fs'%(time.time()-t0), flush=True)
keys=list(out[0].keys())
with open('/home/claude/bob/ergebnisse/sim_v3_vergleich.csv','w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=keys); w.writeheader(); w.writerows(out)
json.dump(out,open('/home/claude/bob/ergebnisse/sim_v3_vergleich.json','w'),indent=1)
