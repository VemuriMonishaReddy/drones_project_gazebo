#!/usr/bin/env python3
"""Gazebo telemetry -> linear and nonlinear/SINDy-style model identification.
Matches the ML/system-identification section of the PyBullet reference without
requiring a separate simulator."""
import csv,sys,math
from pathlib import Path
import numpy as np

def load(path):
    rows=list(csv.DictReader(open(path))); return rows

def col(rows,name,default=0.): return np.array([float(r.get(name,default) or default) for r in rows])

def library(X,U):
    # columns: x,y,z,vx,vy,vz,roll,pitch plus commanded vx,vy,vz
    x,y,z,vx,vy,vz,roll,pitch=X.T; ux,uy,uz=U.T
    return np.column_stack([np.ones(len(x)),x,y,z,vx,vy,vz,roll,pitch,ux,uy,uz,
      np.sin(roll),np.cos(roll),np.sin(pitch),np.cos(pitch),vx*np.abs(vx),vy*np.abs(vy),vz*np.abs(vz),
      roll*pitch,vx*vy,vx*vz,vy*vz,ux*np.tanh(ux),uy*np.tanh(uy),uz*np.tanh(uz)]),

def main(path):
    rows=load(path)
    if len(rows)<20: print('Not enough telemetry for system identification'); return
    # Alpha state/inputs
    t=col(rows,'time'); dt=max(float(np.median(np.diff(t))),1e-3)
    X=np.column_stack([col(rows,k) for k in ['alpha_x','alpha_y','alpha_z']])
    # finite-difference velocities, with attitude appended
    V=np.column_stack([np.gradient(X[:,i],dt) for i in range(3)])
    A=np.column_stack([col(rows,k) for k in ['alpha_roll_deg','alpha_pitch_deg']])*math.pi/180
    S=np.column_stack([X,V,A]); U=np.column_stack([col(rows,k) for k in ['a_cmd_vx','a_cmd_vy','a_cmd_vz']])
    Phi=library(S,U)[0]
    dS=np.column_stack([np.gradient(S[:,i],dt) for i in range(S.shape[1])])
    theta=dS.T@np.linalg.pinv(Phi.T)
    pred=Phi@theta.T; rmse=np.sqrt(np.mean((pred-dS)**2,axis=0))
    out=Path(path).parent/'system_id_metrics.csv'
    with open(out,'w',newline='') as f:
        w=csv.writer(f); w.writerow(['state','one_step_derivative_RMSE']);
        for n,e in zip(['x','y','z','vx','vy','vz','roll','pitch'],rmse): w.writerow([n,float(e)])
    print('System identification written:',out)
    print('RMSE:',dict(zip(['x','y','z','vx','vy','vz','roll','pitch'],rmse)))
if __name__=='__main__': main(sys.argv[1] if len(sys.argv)>1 else 'results/telemetry.csv')
