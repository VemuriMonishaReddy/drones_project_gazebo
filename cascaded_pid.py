#!/usr/bin/env python3
import math
import numpy as np
G=9.81
class PID:
    def __init__(self,kp,ki,kd,i_limit=5.,d_filter=.15):
        self.kp,self.ki,self.kd=kp,ki,kd; self.i_limit=i_limit; self.d_filter=d_filter; self.reset()
    def reset(self): self.integral=0.; self.e_prev=None; self.d_state=0.; self.last=(0,0,0,0,0,0)
    @property
    def Ti(self): return self.kp/self.ki if self.ki else float('inf')
    @property
    def Td(self): return self.kd/self.kp if self.kp else 0.
    def update(self,ref,meas,dt,de_dt=None):
        e=ref-meas; self.integral=float(np.clip(self.integral+e*dt,-self.i_limit/max(self.ki,1e-9),self.i_limit/max(self.ki,1e-9)))
        if de_dt is None:
            raw=0 if self.e_prev is None else (e-self.e_prev)/max(dt,1e-4); self.d_state += self.d_filter*(raw-self.d_state); de_dt=self.d_state
        self.e_prev=e; P=self.kp*e; I=self.ki*self.integral; D=self.kd*de_dt; u=P+I+D; self.last=(e,self.integral,de_dt,P,I,D); return u

DEFAULT=dict(pos_xy=(3.,1.,3.5),vel_xy=(4.,1.,.05),z=(12.,4.,6.),att=(70.,15.,14.),yaw=(20.,2.,8.))
class CascadedController:
    def __init__(self):
        self.x=PID(*DEFAULT['pos_xy']); self.y=PID(*DEFAULT['pos_xy']); self.vx=PID(*DEFAULT['vel_xy']); self.vy=PID(*DEFAULT['vel_xy']); self.z=PID(*DEFAULT['z']); self.roll=PID(*DEFAULT['att']); self.pitch=PID(*DEFAULT['att']); self.yaw=PID(*DEFAULT['yaw']); self.reset()
    def reset(self):
        for p in (self.x,self.y,self.vx,self.vy,self.z,self.roll,self.pitch,self.yaw): p.reset()
    def update(self,pos,vel,rpy,rates,ref_pos,ref_vel,dt,velocity_override=None):
        roll,pitch,yaw=rpy; pr,qr,rr=rates
        if velocity_override is None:
            ax=self.x.update(ref_pos[0],pos[0],dt,ref_vel[0]-vel[0]); ay=self.y.update(ref_pos[1],pos[1],dt,ref_vel[1]-vel[1])
            vx_ref=np.clip(vel[0]+ax*.25,-1.5,1.5); vy_ref=np.clip(vel[1]+ay*.25,-1.5,1.5)
        else:
            vx_ref,vy_ref=velocity_override
            ax=self.vx.update(vx_ref,vel[0],dt); ay=self.vy.update(vy_ref,vel[1],dt)
        abx=math.cos(yaw)*ax+math.sin(yaw)*ay; aby=-math.sin(yaw)*ax+math.cos(yaw)*ay
        pitch_des=float(np.clip(math.atan(abx/G),-.5,.5)); roll_des=float(np.clip(-math.atan(aby/G),-.5,.5))
        az=self.z.update(ref_pos[2],pos[2],dt,ref_vel[2]-vel[2]); T=(1.35*(G+az))/max(math.cos(roll)*math.cos(pitch),.5); T=float(np.clip(T,0,40.))
        tx=.025*self.roll.update(roll_des,roll,dt,-pr); ty=.025*self.pitch.update(pitch_des,pitch,dt,-qr); self.yaw.update(yaw,yaw,dt,-rr)
        return (vx_ref,vy_ref, np.clip(vel[2]+az*.25,-1.,1.)), dict(roll_des=roll_des,pitch_des=pitch_des,T=T,tx=tx,ty=ty,pids=self)

def pid_terms(ctrl): return ctrl.vx.last
