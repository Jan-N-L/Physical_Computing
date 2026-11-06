import numpy as np, math
from scipy.optimize import least_squares
D=[]
for l in open('0005_scan_seitlich.log'):
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
