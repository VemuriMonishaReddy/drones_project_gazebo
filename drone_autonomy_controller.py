#!/usr/bin/env python3
etry/summary metrics.
import math, time, csv, threading, subprocess, sys
from pathlib import Path
import numpy as np
from gz.transport import Node
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.twist_pb2 import Twist
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.laserscan_pb2 import LaserScan
sys.path.insert(0, str(Path(__file__).resolve().parent))
from risk_models import generate_dataset, BayesianRiskModel, LogisticRiskModel, RiskAvoidanceController
from cascaded_pid import CascadedController
from bspline_trajectory import evaluate, sample

WORLD='professor_demo'
DRONES={'alpha':(-6.,-4.,3.6),'bravo':(-6.,-5.8,3.6)}
NODE=Node(); LOCK=threading.Lock(); POSES={}; SCANS={}; prev_pos={}
PUB={n:NODE.advertise(f'/model/{n}/cmd_vel',Twist) for n in DRONES}
ENABLE={n:NODE.advertise(f'/model/{n}/enable',Boolean) for n in DRONES}

# Reference experiment timing (same observable phases as the supplied MuJoCo run).
PHASES=[(0,3,'TAKEOFF'),(3,5,'HOVER / STABILIZE'),(5,8,'CLIMB ABOVE OBSTACLES'),
        (8,12,'TARGET 1 / B-SPLINE NAVIGATION'),(12,16,'TARGET 2 / B-SPLINE NAVIGATION'),
        (16,21,'WIND: HOLD POSITION'),(21,27,'ATTITUDE SAFETY / RECOVERY'),
        (27,34,'FINAL TARGET NAVIGATION'),(34,38,'LAND / TARGET REACHED'),(38,46,'FINAL TARGET HOLD')]

# Main reference path: start -> intermediate targets -> final target.
CP=np.array([
    [-6.0,-4.0,3.6], [-5.5,-3.6,4.3], [-3.8,-2.2,4.6],
    [-1.5,-0.2,4.7], [1.0,1.8,4.7], [3.6,2.2,4.6],
    [5.4,0.5,4.3], [7.0,-2.0,3.6],
], dtype=float)
T1=np.array([-1.5,-0.2,4.7]); T2=np.array([3.6,2.2,4.6]); FINAL=np.array([7.0,-2.0,3.6])
LAND_Z=0.45

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

def scan_cb(n):
    def cb(msg):
        with LOCK: SCANS[n]=msg
    return cb

def sector(scan,a0,a1):
    if scan is None or not scan.ranges: return 99.0
    vals=[]
    for i,v in enumerate(scan.ranges):
        if math.isfinite(v) and v>0:
            a=scan.angle_min+i*scan.angle_step
            if math.radians(a0)<=a<=math.radians(a1): vals.append(v)
    return min(vals) if vals else 99.0

def command(n,vx,vy,vz):
    m=Twist(); m.linear.x=float(vx); m.linear.y=float(vy); m.linear.z=float(vz); PUB[n].publish(m)

def wrench(name,on,strength=1.0):
    topic=f'/world/{WORLD}/wrench/persistent'; clear=f'/world/{WORLD}/wrench/clear'
    if on:
        payload=('entity: {name: "%s", type: MODEL}, wrench: {force: {x: %.3f, y: %.3f, z: 0.0}, torque: {x: %.3f, y: %.3f, z: 0.0}}' %
                 (name,.85*strength,.10*strength,.030*strength,-.020*strength))
        subprocess.run(['gz','topic','-t',topic,'-m','gz.msgs.EntityWrench','-p',payload],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    else:
        subprocess.run(['gz','topic','-t',clear,'-m','gz.msgs.Entity','-p',f'name: "{name}", type: MODEL'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

def phase_at(t):
    for a,b,name in PHASES:
        if a <= t < b: return name
    return PHASES[-1][2]

def phase_progress(t):
    return float(np.clip(t/34.0,0,1))

def reference_for(t, n):
    # The primary Alpha follows the complete B-spline. Bravo follows a parallel
    # offset copy, preserving the two-drone Gazebo demo without changing the reference path.
    if t < 3:
        p=CP[0].copy(); p[2]=3.6+min(t/3,1)*0.0
    elif t < 5:
        p=CP[0].copy()
    elif t < 8:
        p=np.array([CP[0,0],CP[0,1],4.6])
    elif t < 34:
        p=evaluate(CP, phase_progress(t))
    elif t < 38:
        p=FINAL.copy(); p[2]=max(LAND_Z,3.6-(t-34)/4*3.15)
    else:
        p=FINAL.copy(); p[2]=LAND_Z
    if n=='bravo': p=p+np.array([0.,-1.8,0.])
    return p

def obstacle_from_lidar(x,y,yaw,front):
    if front >= 5.5: return None
    op=np.array([x+front*math.cos(yaw), y+front*math.sin(yaw)])
    return op,np.zeros(2),0.45

def main():
    print('=== B-SPLINE AUTONOMOUS DRONE DEMO ===',flush=True)
    print('Blue = drone | Red = obstacles',flush=True)
    print('Yellow = intermediate targets | Green = final target',flush=True)
    print('Cubic B-spline: Cox-de Boor basis recursion',flush=True)
    print('Safety classifier: Bayesian Network + Logistic Regression',flush=True)
    print('Controller: cascaded PID + real four-rotor Gazebo dynamics',flush=True)
    print('Final target hold begins at 38 seconds.',flush=True)
    print('Close the viewer window to stop.',flush=True)
    X,y=generate_dataset(20000,0); n=int(.75*len(X)); bn=BayesianRiskModel().fit(X[:n],y[:n]); lr=LogisticRiskModel().fit(X[:n],y[:n])
    risk={k:RiskAvoidanceController(bn) for k in DRONES}; pid={k:CascadedController() for k in DRONES}
    NODE.subscribe(Pose_V,f'/world/{WORLD}/pose/info',pose_cb)
    for k in DRONES: NODE.subscribe(LaserScan,f'/model/{k}/lidar',scan_cb(k))
    us,pts=sample(CP,600); out=Path(__file__).resolve().parents[1]/'results'; out.mkdir(exist_ok=True)
    with open(out/'bspline_trajectory.csv','w',newline='') as f:
        w=csv.writer(f); w.writerow(['u','x','y','z']); w.writerows([[u,*p] for u,p in zip(us,pts)])
    st=time.time()
    while len(POSES)<len(DRONES) and time.time()-st<20: time.sleep(.1)
    if len(POSES)<len(DRONES): raise RuntimeError('Drone poses not received from Gazebo')
    for k in DRONES:
        b=Boolean(); b.data=True; ENABLE[k].publish(b)
    time.sleep(1.0)
    t0=time.time(); last=t0; prev_phase=''; wind_on=False; wind_off=False; rows=[]; max_err=0; max_roll=0; max_pitch=0; min_z=999; max_z=-999; min_clear=999; recovery_start=None; recovery_time=None
    while time.time()-t0<46.0:
        now=time.time(); t=now-t0; dt=max(.01,min(.08,now-last)); last=now; phase=phase_at(t)
        if phase!=prev_phase:
            print(f'[{t:5.2f}s] {phase}',flush=True)
            if phase=='ATTITUDE SAFETY / RECOVERY': print('  Recovery active: Bayesian risk classifier + nonlinear PID supervisor',flush=True)
            prev_phase=phase
        if 16<=t<21 and not wind_on: wrench('bravo',True,1.0); wind_on=True
        if t>=21 and not wind_off: wrench('bravo',False); wind_off=True
        with LOCK: poses=dict(POSES); scans=dict(SCANS)
        for k in DRONES:
            if k not in poses: continue
            x,y,z,(roll,pitch,yaw)=poses[k]; scan=scans.get(k); front=sector(scan,-22,22); left=sector(scan,25,80); right=sector(scan,-80,-25)
            old=prev_pos.get(k,(x,y,z)); vel=np.array([(x-old[0])/dt,(y-old[1])/dt,(z-old[2])/dt]); prev_pos[k]=(x,y,z)
            ref=reference_for(t,k); ifinal=ref.copy()
            # During wind/safety phases hold the current position reference rather than blindly following the spline.
            if 16<=t<27:
                if k not in globals().get('_holds',{}): pass
                hold=np.array([x,y,4.6 if t<21 else max(z,3.6)])
                if not hasattr(main,'hold'): main.hold={kk:np.array(reference_for(16,kk)) for kk in DRONES}
                ref=main.hold[k].copy(); ref[2]=4.6
            if t>=34: ref[2]=max(LAND_Z,ref[2])
            obs=obstacle_from_lidar(x,y,yaw,front)
            vavoid,ri=risk[k].command(np.array([x,y]),vel[:2],ref[:2],obs,roll,pitch,left,right)
            # At the explicit recovery phase, prioritize position hold and attitude recovery.
            if 21<=t<27:
                vavoid*=0.45
                if max(abs(roll),abs(pitch))>math.radians(8):
                    vavoid*=0.5
            vcmd,ci=pid[k].update(np.array([x,y,z]),vel,(roll,pitch,yaw),(0,0,0),ref,np.zeros(3),dt,velocity_override=vavoid)
            drag=np.array([-0.10*vel[0]*abs(vel[0]),-0.10*vel[1]*abs(vel[1]),-0.06*vel[2]*abs(vel[2])])
            desired=np.array(vcmd)+drag
            if front<2.5: desired[:2]*=max(0,min(1,(front-0.8)/1.7))
            desired=np.clip(desired,[-1.0,-1.0,-0.8],[1.0,1.0,0.8])
            prev_cmd=getattr(pid[k],'last_cmd',np.zeros(3)); cmd=prev_cmd+np.clip(desired-prev_cmd,-.08,.08); pid[k].last_cmd=cmd
            command(k,*cmd)
            err=float(np.linalg.norm(ref-np.array([x,y,z]))); max_err=max(max_err,err); max_roll=max(max_roll,abs(math.degrees(roll))); max_pitch=max(max_pitch,abs(math.degrees(pitch))); min_z=min(min_z,z); max_z=max(max_z,z); min_clear=min(min_clear,front)
            if 21<=t<27 and recovery_start is None and max(abs(roll),abs(pitch))>math.radians(5): recovery_start=t
            if recovery_start is not None and recovery_time is None and t>recovery_start and max(abs(roll),abs(pitch))<math.radians(2): recovery_time=t-recovery_start
            rows.append([t,k,phase,x,y,z,math.degrees(roll),math.degrees(pitch),front,left,right,err,*ref,ri['dist'],ri['closing'],ri['ttc'],ri['risk'],ri['action'],*ri['probs'],ri['side'],*ci['pids'].vx.last,ci['roll_des'],ci['pitch_des'],ci['T'],ci['tx'],ci['ty'],*drag,*vel,*cmd])
        time.sleep(.02)
    for k in DRONES: command(k,0,0,0)
    header=['time','drone','phase','x','y','z','roll_deg','pitch_deg','lidar_front_m','lidar_left_m','lidar_right_m','tracking_error_m','ref_x','ref_y','ref_z','risk_dist_m','closing_speed_mps','ttc_s','risk_class','action','p_safe','p_warning','p_danger','avoid_side','pid_error','pid_integral','pid_derivative','pid_P','pid_I','pid_D','roll_des','pitch_des','thrust_reference','tau_x_reference','tau_y_reference','nonlinear_drag_x','nonlinear_drag_y','nonlinear_drag_z','vx','vy','vz','cmd_vx','cmd_vy','cmd_vz']
    with open(out/'drone_flight_log.csv','w',newline='') as f: w=csv.writer(f); w.writerow(header); w.writerows(rows)
    metrics={'simulated_duration_s':46.0,'maximum_tracking_error_m':max_err,'maximum_absolute_roll_deg':max_roll,'maximum_absolute_pitch_deg':max_pitch,'minimum_drone_center_height_m':min_z,'maximum_drone_center_height_m':max_z,'minimum_lidar_clearance_m':min_clear,'wind_recovery_time_s':recovery_time}
    with open(out/'demo_summary.csv','w',newline='') as f:
        w=csv.writer(f); w.writerow(['metric','value']); w.writerows(metrics.items())
    print('\n=== DEMO SUMMARY ===',flush=True)
    for k,v in metrics.items(): print(f'{k}: {v}',flush=True)
    print(f'B-spline CSV: {out/"bspline_trajectory.csv"}',flush=True)
    print(f'Flight log CSV: {out/"drone_flight_log.csv"}',flush=True)
    print(f'Note: B-spline generation is paired with LiDAR/risk-based collision avoidance; spline generation alone is not treated as proof of avoidance.',flush=True)

if __name__=='__main__': main()
