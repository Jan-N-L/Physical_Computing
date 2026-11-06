import sys, random, json
sys.path.insert(0,'.'); sys.path.insert(1,'../pico'); sys.path.insert(0,'../sim')
import sim, main_v3 as M
rng=random.Random(5)
base=dict(kp_fine=M.KP_FINE,kp_coarse=M.KP_COARSE,kd=M.KD,t_lead=M.T_LEAD,ki=M.KI,i_max=M.I_MAX,fc=M.FC,
          e_hold=M.E_HOLD,v_hold=M.V_HOLD,kick_t=M.KICK_T,kick_rate=M.KICK_RATE,kick_max=M.KICK_MAX,rate=M.RATE,alpha=M.ALPHA,beta=M.BETA)
def ev(p,n=60,s0=0):
    r=sim.evaluate(lambda: M.Regler(**p), n=n, seed0=s0); s=r['stats']
    return s, r['summary']
res=[]
s,sm=ev({}); print('base',sm,flush=True); res.append(({},s))
for k in range(45):
    p={}
    for key,v in base.items():
        if rng.random()<0.5: p[key]=round(v*rng.choice([0.6,0.8,1.25,1.5]),3)
    s,sm=ev(p); res.append((p,s)); print(k,sm,p,flush=True)
ok=[r for r in res if r[1]['erfolg']>=55]
ok.sort(key=lambda r:r[1]['T_median']+0.3*r[1]['T_p90'])
json.dump([r[0] for r in ok[:8]],open('kand.json','w'))
print('TOP'); [print(r[1]['summary'] if False else (r[1]['erfolg'],round(r[1]['T_median'],2),round(r[1]['T_p90'],2)),r[0]) for r in ok[:8]]
