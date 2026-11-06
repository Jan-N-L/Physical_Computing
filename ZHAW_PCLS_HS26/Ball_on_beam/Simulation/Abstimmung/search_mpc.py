import sys, json, time
sys.path.insert(0,'/home/claude/bob/pico'); sys.path.insert(0,'/home/claude/bob/sim')
import sim, ctrl_mpc as M
def cost(p, n=60, s0=0):
    r=sim.evaluate(lambda: M.Regler(**p), n=n, seed0=s0)
    Ts=[x['T_true'] if x['ok_true'] else 45.0 for x in r['runs']]
    st=r['stats']
    return sum(Ts)/len(Ts) + 0.5*st['T_p90'], st['summary']
p={}
c0,s=cost(p); print('base',round(c0,2),s,flush=True)
space={'wt':[1.0,2.0,4.0],'abrk':[80.,120.,170.],'h':[0.1,0.15,0.2],'nh':[2,3,4],'erel':[6.,9.,14.],'ehold':[0.,4.,7.],
       'vhold':[8.,12.,18.],'rate_u':[100.,140.,200.],'gs':[0.15,0.3,0.5],'l1':[0.35,0.45,0.6],'tpre':[0.05,0.1],'K':[10.,13.]}
for rnd in range(2):
  for k,vals in space.items():
    for v in vals:
        q=dict(p); q[k]=v
        if q==p: continue
        c,s=cost(q)
        if c < c0-0.05:
            c0,p=c,q; print('  besser',k,v,round(c,2),s,flush=True)
  print('runde',rnd,p,round(c0,2),flush=True)
json.dump(p,open('/home/claude/bob/methods/kombi/mpc_best.json','w'))
c,s=cost(p,100,50000); print('kontrolle',p,s)
