#!/usr/bin/env python3
import csv,sys,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from risk_models import generate_dataset,BayesianRiskModel,LogisticRiskModel,RiskAvoidanceController
class Obs:
    def __init__(self,p0,v,r=.3): self.p=np.array(p0,float); self.v=np.array(v,float); self.r=r
    def at(self,t): return self.p+self.v*t

def trial(model,speed,avoid,seed):
    rng=np.random.default_rng(seed); cruise=1.5; xc=rng.uniform(3,9); ta=xc/cruise+rng.uniform(-.5,.5); direction=rng.choice([-1.,1.]); ob=Obs((xc,-direction*speed*ta),(0,direction*speed))
    pos=np.array([0.,0.]); vel=np.zeros(2); goal=np.array([12.,0.]); ctrl=RiskAvoidanceController(model)
    dt=.02; minc=99.; hit=False; reached=False
    for k in range(int(25/dt)):
        t=k*dt; op=ob.at(t); 
        if avoid:
            vcmd,info=ctrl.command(pos,vel,goal, (op,ob.v,ob.r),0,0, left_clear=3,right_clear=3)
        else:
            d=goal-pos; vcmd=1.5*d/(np.linalg.norm(d)+1e-9); info={'action':0}
        acc=(vcmd-vel)/.35; an=np.linalg.norm(acc)
        if an>4: acc*=4/an
        vel+=acc*dt; pos+=vel*dt; c=np.linalg.norm(op-pos)-ob.r-.15; minc=min(minc,c); hit|=c<0
        if np.linalg.norm(goal-pos)<.3: reached=True; break
    return hit,reached,minc

def main(out='results/speed_sweep.csv'):
    X,y=generate_dataset(10000,0); m=BayesianRiskModel().fit(X,y); rows=[]
    for sp in [.5,1,2,3]:
      for av in [False,True]:
       vals=[trial(m,sp,av,100+int(sp*10)+j) for j in range(6)]
       rows.append([sp,'with avoidance' if av else 'no avoidance',np.mean([v[0] for v in vals]),np.mean([v[1] for v in vals]),np.mean([v[2] for v in vals])])
    p=Path(out); p.parent.mkdir(exist_ok=True)
    with open(p,'w',newline='') as f: w=csv.writer(f); w.writerow(['speed_mps','mode','collision_rate','success_rate','mean_min_clearance_m']); w.writerows(rows)
    print('Benchmark:',p)
if __name__=='__main__': main(sys.argv[1] if len(sys.argv)>1 else 'results/speed_sweep.csv')
