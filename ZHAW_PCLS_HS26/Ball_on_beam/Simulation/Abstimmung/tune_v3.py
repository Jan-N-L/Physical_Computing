import sys, json
sys.path.insert(0,'.'); sys.path.insert(1,'../pico'); sys.path.insert(0,'../sim')
import sim, main_v3 as M
def cost(p, n=60, s0=0):
    r=sim.evaluate(lambda: M.Regler(**p), n=n, seed0=s0)
    Ts=[x['T_true'] if x['ok_true'] else 45.0 for x in r['runs']]
    return sum(Ts)/len(Ts)+0.5*r['stats']['T_p90'], r['summary']
p={}
c0,s=cost(p); print('base',round(c0,2),s,flush=True)
space={'kp_fine':[0.1,0.25,0.35],'kp_coarse':[0.2,0.45,0.6],'kd':[0.1,0.25,0.35],'t_lead':[0.0,0.15,0.2],
       'ki':[0.02,0.08,0.15],'i_max':[12.,25.],'fc':[1.5,5.],'e_hold':[4.,8.],'v_hold':[10.,25.],
       'kick_t':[0.5,1.2],'kick_rate':[8.,30.],'kick_max':[8.,25.],'rate':[90.,250.],'alpha':[0.5],'beta':[0.15,0.5]}
for rnd in range(2):
  for k,vals in space.items():
    for v in vals:
        q=dict(p); q[k]=v
        if q==p: continue
        c,s=cost(q)
        if c<c0-0.05: c0,p=c,q; print('  besser',k,v,round(c,2),s,flush=True)
  print('runde',rnd,p,round(c0,2),flush=True)
json.dump(p,open('best_v3.json','w'))
print('kontrolle', sim.evaluate(lambda: M.Regler(**p), n=150, seed0=90000)['summary'])
