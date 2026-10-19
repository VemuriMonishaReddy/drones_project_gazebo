#!/usr/bin/env python3
"""Complete two-drone professor demo: PID + nonlinear outer loop + LiDAR + wind recovery.

The Gazebo MulticopterVelocityControl plugin remains the inner flight/attitude layer.
This script adds a transparent outer position PID, anti-windup, nonlinear quadratic-drag
compensation, command/acceleration limiting, LiDAR obstacle supervision, and telemetry.
"""
import math, time, csv, threading, subprocess, sys
from pathlib import Path
from gz.transport import Node
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.twist_pb2 import Twist
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.laserscan_pb2 import LaserScan

WORLD='professor_demo'
DRONES={'alpha':(-6.0,-4.0,3.6),'bravo':(6.0,-1.0,3.6)}
NODE=Node(); LOCK=threading.Lock(); POSES={}; SCANS={}; LOG=[]
POSETOPIC=f'/world/{WORLD}/pose/info'
PUB={n:NODE.advertise(f'/model/{n}/cmd_vel',Twist) for n in DRONES}
ENABLE={n:NODE.advertise(f'/model/{n}/enable',Boolean) for n in DRONES}
SAFE_X=1.95; SAFE_Y_NEAR=-2.0; SAFE_Y_FAR=-6.15; SAFE_Z=3.6

class PID:
    def __init__(self,kp,ki,kd,imax=2.0,umax=0.75):
        self.kp,self.ki,self.kd=kp,ki,kd; self.imax,self.umax=imax,umax
        self.integral=0.0; self.prev_error=0.0; self.first=True
    @property
    def Ti(self): return self.kp/self.ki if self.ki else float('inf')
    @property
    def Td(self): return self.kd/self.kp if self.kp else 0.0
    def update(self,e,dt):
        if dt<=0: dt=0.02
        p=self.kp*e
        if self.first: self.prev_error=e; self.first=False
        self.integral=max(-self.imax,min(self.imax,self.integral+e*dt))
        i=self.ki*self.integral
        d=self.kd*(e-self.prev_error)/dt
        raw=p+i+d; u=max(-self.umax,min(self.umax,raw))
        # simple conditional anti-windup
        if raw!=u and ((raw>u and e>0) or (raw<u and e<0)):
            self.integral-=e*dt
            i=self.ki*self.integral
            raw=p+i+d; u=max(-self.umax,min(self.umax,raw))
        self.prev_error=e
        return u,p,i,d,raw

PIDS={n:{'x':PID(.70,.12,.08),'y':PID(.70,.12,.08),'z':PID(.90,.16,.06)} for n in DRONES}
last_time=None; prev_pos={}; prev_vel={}; prev_cmd={n:(0.0,0.0,0.0) for n in DRONES}


def rpy(q):
    sr=2*(q.w*q.x+q.y*q.z); cr=1-2*(q.x*q.x+q.y*q.y)
    roll=math.atan2(sr,cr); sp=2*(q.w*q.y-q.z*q.x)
    pitch=math.copysign(math.pi/2,sp) if abs(sp)>=1 else math.asin(sp)
    sy=2*(q.w*q.z+q.x*q.y); cy=1-2*(q.y*q.y+q.z*q.z)
    return roll,pitch,math.atan2(sy,cy)

def pose_cb(msg):
    with LOCK:
        for p in msg.pose:
            if p.name in DRONES: POSES[p.name]=(p.position.x,p.position.y,p.position.z,rpy(p.orientation))

def scan_cb(name):
    def cb(msg):
        with LOCK: SCANS[name]=msg
    return cb

def clamp(x,a,b): return max(a,min(b,x))
def sector(scan,a0,a1):
    if scan is None or not scan.ranges: return 99.0
    vals=[]
    for i,v in enumerate(scan.ranges):
        if not math.isfinite(v) or v<=0: continue
        a=scan.angle_min+i*scan.angle_step
        if math.radians(a0)<=a<=math.radians(a1): vals.append(v)
    return min(vals) if vals else 99.0

def world_to_body(vx,vy,yaw):
    c,s=math.cos(yaw),math.sin(yaw); return c*vx+s*vy,-s*vx+c*vy

def body_to_world(vx,vy,yaw):
    c,s=math.cos(yaw),math.sin(yaw); return c*vx-s*vy,s*vx+c*vy

def command(n,vx,vy,vz):
    m=Twist(); m.linear.x=float(vx); m.linear.y=float(vy); m.linear.z=float(vz); PUB[n].publish(m)

def wrench(name,on,strength=1.0):
    topic=f'/world/{WORLD}/wrench/persistent'; clear=f'/world/{WORLD}/wrench/clear'
    if on:
        payload=('entity: {name: "%s", type: MODEL}, wrench: {force: {x: %.3f, y: %.3f, z: 0.0}, torque: {x: %.3f, y: %.3f, z: 0.0}}'
                 %(name,.85*strength,.10*strength,.030*strength,-.020*strength))
        subprocess.run(['gz','topic','-t',topic,'-m','gz.msgs.EntityWrench','-p',payload],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=2)
    else:
        subprocess.run(['gz','topic','-t',clear,'-m','gz.msgs.Entity','-p',f'name: "{name}", type: MODEL'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=2)

def particle_cmd(model,emitter,emitting):
    topic=f'/model/{model}/link/link/particle_emitter/{emitter}/cmd'
    value='true' if emitting else 'false'
    subprocess.run(['gz','topic','-t',topic,'-m','gz.msgs.ParticleEmitter','-p',f'emitting: {{data: {value}}}'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=2)

def pid_velocity(name,pos,target,yaw,dt,nonlinear_enabled=True):
    global prev_pos,prev_vel
    x,y,z=pos
    ex,ey,ez=target[0]-x,target[1]-y,target[2]-z
    ux,px,ix,dx,rawx=PIDS[name]['x'].update(ex,dt)
    uy,py,iy,dy,rawy=PIDS[name]['y'].update(ey,dt)
    uz,pz,iz,dz,rawz=PIDS[name]['z'].update(ez,dt)
    # finite-difference velocity estimate, then nonlinear quadratic drag compensation
    if name in prev_pos:
        vx=(x-prev_pos[name][0])/dt; vy=(y-prev_pos[name][1])/dt; vz=(z-prev_pos[name][2])/dt
    else: vx=vy=vz=0.0
    prev_pos[name]=(x,y,z); prev_vel[name]=(vx,vy,vz)
    drag_c=.10
    ndx=-drag_c*vx*abs(vx) if nonlinear_enabled else 0.0
    ndy=-drag_c*vy*abs(vy) if nonlinear_enabled else 0.0
    ndz=-.06*vz*abs(vz) if nonlinear_enabled else 0.0
    wx,wy=ux+ndx,uy+ndy
    bx,by=world_to_body(wx,wy,yaw)
    bz=uz+ndz
    # nonlinear actuator saturation + command acceleration limiting
    maxv=.62
    desired=(clamp(bx,-maxv,maxv),clamp(by,-maxv,maxv),clamp(bz,-.35,.35))
    old=prev_cmd[name]; maxdv=.045
    cmd=tuple(old[i]+clamp(desired[i]-old[i],-maxdv,maxdv) for i in range(3))
    prev_cmd[name]=cmd
    return cmd,(ex,ey,ez),(px,ix,dx,py,iy,dy,pz,iz,dz),(ndx,ndy,ndz),(vx,vy,vz)

NODE.subscribe(Pose_V,POSETOPIC,pose_cb)
for n in DRONES: NODE.subscribe(LaserScan,f'/model/{n}/lidar',scan_cb(n))
print('COMPLETE PROFESSOR DEMO — TWO DRONES / PID / NONLINEAR / LiDAR / WIND',flush=True)
start=time.time()
while len(POSES)<2 and time.time()-start<20: time.sleep(.1)
if len(POSES)<2:
    print('ERROR: two drone poses not received',file=sys.stderr); sys.exit(2)
for n in DRONES:
    b=Boolean(); b.data=True; ENABLE[n].publish(b)
time.sleep(2)
alpha_waypoints=[(-6.0,-4.0,3.6,'ALPHA_HOVER'),(-3.2,-4.0,3.6,'ALPHA_APPROACH'),(-3.2,-6.8,3.6,'ALPHA_AVOID_DOWN'),(3.0,-6.8,3.6,'ALPHA_AVOID_ACROSS'),(3.0,-4.0,3.6,'ALPHA_CLEAR_AND_RETURN')]
wp=0; wind_on=wind_off=False; rain_on=rain_wind_on=rain_wind_off=rain_off=False
T_END=105.0; t0=time.time(); last_time=t0
while True:
    now=time.time(); t=now-t0
    if t>=T_END: break
    dt=clamp(now-last_time,.01,.08); last_time=now
    with LOCK: poses=dict(POSES); scans=dict(SCANS)
    if len(poses)<2: continue
    ax,ay,az,ar=poses['alpha']; bx,by,bz,br=poses['bravo']; yaw=ar[2]
    scan=scans.get('alpha'); front=sector(scan,-22,22); left=sector(scan,25,80); right=sector(scan,-80,-25)
    tx,ty,tz,phase=alpha_waypoints[wp]
    if math.hypot(tx-ax,ty-ay)<.28 and abs(tz-az)<.22 and wp<len(alpha_waypoints)-1:
        wp+=1; tx,ty,tz,phase=alpha_waypoints[wp]; print(f'[ALPHA] waypoint {wp}/{len(alpha_waypoints)-1}: {phase}',flush=True)
    scale=clamp((front-1.6)/1.4,0,1) if wp<=1 and front<3.0 else 1.0
    inside=(-SAFE_X<ax<SAFE_X and SAFE_Y_FAR<ay<SAFE_Y_NEAR)
    safety=False
    if inside:
        safety=True; target=(ax,SAFE_Y_FAR if abs(ay-SAFE_Y_FAR)<abs(ay-SAFE_Y_NEAR) else SAFE_Y_NEAR); phase='ALPHA_EMERGENCY_SAFETY'
    else: target=(tx,ty,tz)
    nonlinear_enabled=t>7.0
    acmd,aerr,apd,anl,avel=pid_velocity('alpha',(ax,ay,az),target,yaw,dt,nonlinear_enabled)
    acmd=(acmd[0]*scale,acmd[1]*scale,acmd[2])
    if front<1.6 and wp<=1: acmd=(min(acmd[0],0),acmd[1],acmd[2])
    command('alpha',*acmd)
    bcmd,berr,bpd,bnl,bvel=pid_velocity('bravo',(bx,by,bz),(6.0,-1.0,SAFE_Z),br[2],dt,nonlinear_enabled)
    command('bravo',*bcmd)
    if 47<=t<51 and not wind_on: wrench('bravo',True); wind_on=True; print('[WIND] >>> GUST ON BRAVO',flush=True)
    if t>=51 and not wind_off: wrench('bravo',False); wind_off=True; print('[WIND] >>> GUST OFF — recovery',flush=True)
    if t>=44 and int(t*10)%100==0: print('[WEATHER] wind-borne leaves + paper active',flush=True)
    if t>=62 and not rain_on: particle_cmd('rain_field','rain',True); rain_on=True; print('[WEATHER] >>> RAIN ON',flush=True)
    if 62<=t<72 and not rain_wind_on: wrench('bravo',True,.55); rain_wind_on=True; print('[WEATHER] >>> STORM GUST ON',flush=True)
    if t>=72 and not rain_wind_off: wrench('bravo',False); rain_wind_off=True; print('[WEATHER] >>> STORM GUST OFF',flush=True)
    if t>=82 and not rain_off: particle_cmd('rain_field','rain',False); rain_off=True; print('[WEATHER] >>> RAIN OFF',flush=True)
    mind=math.hypot(ax-bx,ay-by); weather='rain' if 62<=t<82 else ('wind' if 47<=t<51 else ('breeze' if t>=44 else 'clear'))
    LOG.append([t,ax,ay,az,math.degrees(ar[0]),math.degrees(ar[1]),front,left,right,bx,by,bz,math.degrees(br[0]),math.degrees(br[1]),mind,tx,ty,tz,phase,wp,weather,int(wind_on and not wind_off),*aerr,*apd,*anl,*avel,*acmd,*berr,*bpd,*bnl,*bvel,*bcmd,int(nonlinear_enabled),int(safety)])
    time.sleep(.02)
for n in DRONES: command(n,0,0,0)
out=Path(__file__).resolve().parents[1]/'results'; out.mkdir(exist_ok=True)
header=['time','alpha_x','alpha_y','alpha_z','alpha_roll_deg','alpha_pitch_deg','lidar_front_m','lidar_left_m','lidar_right_m','bravo_x','bravo_y','bravo_z','bravo_roll_deg','bravo_pitch_deg','inter_drone_distance','alpha_des_x','alpha_des_y','alpha_des_z','alpha_phase','alpha_waypoint','weather_phase','wind_active','a_ex','a_ey','a_ez','a_px','a_ix','a_dx','a_py','a_iy','a_dy','a_pz','a_iz','a_dz','a_nonlinear_x','a_nonlinear_y','a_nonlinear_z','a_vx','a_vy','a_vz','a_cmd_bx','a_cmd_by','a_cmd_bz','b_ex','b_ey','b_ez','b_px','b_ix','b_dx','b_py','b_iy','b_dy','b_pz','b_iz','b_dz','b_nonlinear_x','b_nonlinear_y','b_nonlinear_z','b_vx','b_vy','b_vz','b_cmd_bx','b_cmd_by','b_cmd_bz','nonlinear_enabled','alpha_safety']
with open(out/'telemetry.csv','w',newline='') as f: csv.writer(f).writerows([header,*LOG])
print('105-second complete experiment finished; telemetry saved.',flush=True)
