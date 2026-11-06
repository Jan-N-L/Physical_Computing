# identifikation.py: Streckenparameter aus echten Logs schätzen (Ball on Beam, PC, nicht für den Pico).
# Zusammengelegt aus fit5.py und ident.py (06.11.2026), Rechnung unverändert.
#
# scan_fit():   Bewegungsgleichung x'' = K*(u - lvl) + A*sin(2*pi*x/C + ph) - mu*tanh(v/5)
#               auf 3 Abschnitte von 0005_scan_seitlich.log fitten (konstanter Servowinkel).
#               Ergebnis 06.11.: rms 4.2 mm, K 6.1, lvl +3.0, A 86 mm/s², C 96 mm, mu 11 mm/s².
# regression(): Beschleunigung (Savitzky-Golay) aus den Regelläufen 0006 bis 0009 linear
#               zurückrechnen, Totzeit d, Servo-Verzögerung tau und Rollweg C per Raster.
#               Ergebnis 06.11.: K 12.4 bis 15.2 mm/s² pro ° Servo, d 0.09 s, tau 0.03 s, mu ca. 28.
#
# Aufruf: python identifikation.py [alle|scan|regression] [Log-Ordner]
#         Standard Log-Ordner: ../Messdaten/pico_logs  (braucht numpy, scipy)
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Messdaten', 'pico_logs')


def scan_fit(logdir=LOGS):
    D=[]
    for l in open(os.path.join(logdir, '0005_scan_seitlich.log')):
        if l.startswith('D,'):
            t,s,x,u,ph=l.strip().split(',')[1:]; D.append((float(t),float(x),float(u)))
    D=np.array(D); D[:,0]-=D[0,0]
    def seg(a,b):
        m=(D[:,0]>=a)&(D[:,0]<=b); return D[m]
    SEG=[(seg(0.3,8.0),0.0),(seg(8.5,13.6),5.5),(seg(16.5,18.9),-2.5)]
    def simulate(p,S,u,x0,v0,dt=0.005):
        K,lvl,A,C,ph,mu=p
        ts=S[:,0]; out=np.empty(len(ts)); t=ts[0]; x=x0; v=v0; k=0; w=2*math.pi/C; a0=K*(u-lvl)
        while k<len(ts):
            while t < ts[k]-1e-9:
                a=a0+A*math.sin(w*x+ph)-mu*math.tanh(v/5)
                vm=v+a*dt/2; xm=x+v*dt/2
                am=a0+A*math.sin(w*xm+ph)-mu*math.tanh(vm/5)
                x+=vm*dt; v+=am*dt; t+=dt
            out[k]=x; k+=1
        return out
    def res(q):
        p=q[:6]; r=[]
        for i,(S,u) in enumerate(SEG):
            r.append(simulate(p,S,u,q[6+2*i],q[7+2*i])-S[:,1])
        return np.concatenate(r)
    best=None
    for C in (100,110,120):
      for ph in np.linspace(0,2*np.pi,4,endpoint=False):
        q0=[10.1,1.0,100,C,ph,20, 10,50, 140,30, 330,-100]
        lo=[6,-4,0,90,-10,0, -10,-200, 100,-200, 250,-500]; hi=[14,6,300,125,20,80, 60,300, 200,300, 345,100]
        r=least_squares(res,q0,bounds=(lo,hi),max_nfev=120)
        c=np.sqrt(np.mean(r.fun**2))
        print('C0',C,'ph0 %.1f rms %.2f'%(ph,c),' K %.1f lvl %.2f A %.0f C %.1f mu %.1f'%(r.x[0],r.x[1],r.x[2],r.x[3],r.x[5]),flush=True)
        if best is None or c<best[0]: best=(c,r.x)
    print('BEST',best[0],np.round(best[1],2))


def regression(logdir=LOGS):
    def load(fn):
        runs=[];cur=[]
        for l in open(os.path.join(logdir, fn)):
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


if __name__ == '__main__':
    was = sys.argv[1] if len(sys.argv) > 1 else 'alle'
    logdir = sys.argv[2] if len(sys.argv) > 2 else LOGS
    if was in ('alle', 'regression'):
        regression(logdir)
    if was in ('alle', 'scan'):
        scan_fit(logdir)
