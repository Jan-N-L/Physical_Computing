import numpy as np
from scipy.signal import savgol_filter
def load(fn):
    runs=[];cur=[]
    for l in open(fn):
        if l.startswith('D,'):
            p=l.strip().split(','); cur.append((float(p[1]),float(p[3]),int(p[4]),float(p[5])))
        elif l.startswith('R,'): runs.append(np.array(cur)); cur=[]
    return runs
data=[]
for fn in ['0007_golfer.log','0008_pid.log','0009_imitation.log','0006_mpc.log']:
    for r in load(fn):
        # resample uniform 30ms
        t=r[:,0]; tg=np.arange(t[0],t[-1],0.03)
        x=np.interp(tg,t,r[:,1]); u=np.interp(tg,t,r[:,3])
        xs=savgol_filter(x,11,2); v=savgol_filter(x,11,2,deriv=1,delta=0.03); a=savgol_filter(x,15,2,deriv=2,delta=0.03)
        m=(xs>25)&(xs<315)
        data.append((fn,tg,xs,v,a,u,m))
def fit(C,d,tau,sel):
    rows=[];ys=[]
    for fn,tg,xs,v,a,u,m in data:
        if fn not in sel: continue
        # servo: first-order lag tau after dead time d (in samples of 30 ms, fractional via interp)
        ud=np.interp(tg-d,tg,u)
        th=np.zeros_like(ud); al=0.03/(tau+0.03) if tau>0 else 1
        for i in range(1,len(ud)): th[i]=th[i-1]+al*(ud[i]-th[i-1])
        w=2*np.pi/C
        X=np.column_stack([th,np.ones_like(th),np.sin(w*xs),np.cos(w*xs),-np.tanh(v/10)])
        rows.append(X[m]); ys.append(a[m])
    X=np.vstack(rows); y=np.concatenate(ys)
    c,res,_,_=np.linalg.lstsq(X,y,rcond=None)
    r=y-X@c
    return np.sqrt(np.mean(r**2)),c
for sel in (['0007_golfer.log','0008_pid.log','0009_imitation.log'],['0006_mpc.log'],['0007_golfer.log','0008_pid.log','0009_imitation.log','0006_mpc.log']):
    best=None
    for C in range(88,124,4):
        for d in (0.0,0.03,0.06,0.09,0.12,0.15):
            for tau in (0.0,0.03,0.06,0.1):
                e,c=fit(C,d,tau,sel)
                if best is None or e<best[0]: best=(e,C,d,tau,c)
    e,C,d,tau,c=best
    K=c[0]; lvl=-c[1]/K; A=np.hypot(c[2],c[3])
    print(sel, 'rms a %.0f'%e,'C',C,'d',d,'tau',tau,'K %.2f lvl %.2f A %.0f mu %.1f'%(K,lvl,A,c[4]))
print('--- feiner')
sel=['0007_golfer.log','0008_pid.log','0009_imitation.log','0006_mpc.log']
res=[]
for C in range(50,140,5):
    for d in (0.03,0.06,0.09,0.12):
        for tau in (0.0,0.03,0.06):
            e,c=fit(C,d,tau,sel); res.append((e,C,d,tau,c))
res.sort(key=lambda r:r[0])
for e,C,d,tau,c in res[:6]:
    print('rms %.1f C %d d %.2f tau %.2f K %.2f lvl %.2f A %.0f mu %.1f'%(e,C,d,tau,c[0],-c[1]/c[0],np.hypot(c[2],c[3]),c[4]))
# per-log K with best C
for fn in sel:
    e,c=fit(res[0][1],res[0][2],res[0][3],[fn]); print(fn,'K %.2f lvl %.2f A %.0f mu %.1f rms %.0f'%(c[0],-c[1]/c[0],np.hypot(c[2],c[3]),c[4],e))
