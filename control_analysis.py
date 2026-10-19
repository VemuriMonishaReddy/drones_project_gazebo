#!/usr/bin/env python3
import csv,sys,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load(p): return list(csv.DictReader(open(p)))
def c(rows,k): return np.array([float(r.get(k,0) or 0) for r in rows])
def s(rows,k): return np.array([r.get(k,'') for r in rows])
def plot(x,y,lab,title,path,ylab=''):
 plt.figure(figsize=(10,4)); plt.plot(x,y); plt.xlabel('time (s)'); plt.ylabel(ylab); plt.title(title); plt.grid(alpha=.3); plt.tight_layout(); plt.savefig(path,dpi=140); plt.close()
def main(p):
 rows=load(p); out=Path(p).parent; rows_by={n:[r for r in rows if r['drone']==n] for n in ['alpha','bravo']}
 metrics=[]
 for n,rs in rows_by.items():
  if not rs: continue
  t=c(rs,'time'); ex=c(rs,'goal_x')-c(rs,'x'); ey=c(rs,'goal_y')-c(rs,'y'); ez=c(rs,'goal_z')-c(rs,'z'); e=np.sqrt(ex*ex+ey*ey+ez*ez)
  for key,title in [('pid_P','P term'),('pid_I','I term'),('pid_D','D term')]: plot(t,c(rs,key),key,title+' — '+n,out/f'{n}_{key}.png',key)
  plot(t,e,'error','position tracking error — '+n,out/f'{n}_tracking_error.png','m')
  plot(t,c(rs,'risk_dist_m'),'LiDAR distance','risk distance / clearance — '+n,out/f'{n}_risk_distance.png','m')
  plot(t,c(rs,'ttc_s'),'TTC','time-to-collision — '+n,out/f'{n}_ttc.png','s')
  plot(t,c(rs,'roll_deg'),'roll','self-stabilization — '+n,out/f'{n}_stabilization.png','deg')
  plot(t,c(rs,'pitch_deg'),'pitch','pitch stabilization — '+n,out/f'{n}_pitch_stabilization.png','deg')
  plot(t,c(rs,'nonlinear_drag_x'),'drag','nonlinear quadratic drag — '+n,out/f'{n}_nonlinear_drag.png','m/s²')
  plot(t,c(rs,'x'), 'x', 'actual X vs target — '+n,out/f'{n}_x.png','m')
  plot(t,c(rs,'y'), 'y', 'actual Y vs target — '+n,out/f'{n}_y.png','m')
  plot(t,c(rs,'z'), 'z', 'actual Z vs target — '+n,out/f'{n}_z.png','m')
  ti=3/1; td=3.5/3
  metrics.append([n,float(np.sqrt(np.mean(e*e))),float(np.max(e)),float(np.mean(c(rs,'roll_deg')**2+c(rs,'pitch_deg')**2)**.5),float(np.min(c(rs,'risk_dist_m'))),ti,td])
 plt.figure(figsize=(10,4)); plt.plot(c(rows,'time'),c(rows,'p_safe'),label='SAFE'); plt.plot(c(rows,'time'),c(rows,'p_warning'),label='WARNING'); plt.plot(c(rows,'time'),c(rows,'p_danger'),label='DANGER'); plt.legend(); plt.grid(alpha=.3); plt.xlabel('time (s)'); plt.ylabel('probability'); plt.title('Bayesian risk probabilities'); plt.tight_layout(); plt.savefig(out/'risk_probabilities.png',dpi=140); plt.close()
 with open(out/'metrics.csv','w',newline='') as f:
  w=csv.writer(f); w.writerow(['drone','position_RMSE_m','max_position_error_m','RMS_tilt_deg','min_risk_clearance_m','Ti_s','Td_s']); w.writerows(metrics)
 print('Analysis complete:',out)
if __name__=='__main__': main(sys.argv[1] if len(sys.argv)>1 else 'results/telemetry.csv')
