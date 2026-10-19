#!/usr/bin/env python3
"""PyBullet-equivalent probabilistic obstacle-risk stack for Gazebo.

Implements the same feature/risk/decision pipeline as the reference project:
  features = distance, closing speed, TTC, |roll|, |pitch|
  Logistic regression risk classifier
  Discrete Bayesian Network with Distance -> TTC <- ClosingSpeed,
      Distance -> Risk, TTC -> Risk, Tilt -> Risk
  expected-cost decision: CONTINUE / SLOW_DOWN / AVOID

The control loop uses the fast empirical-Bayesian table so it does not require
pgmpy at runtime. If pgmpy is installed, validate_models.py additionally fits
an exact pgmpy BN using the same structure and BDeu estimator.
"""
import math
import numpy as np

RISK = ["SAFE", "WARNING", "DANGER"]
ACTIONS = ["CONTINUE", "SLOW_DOWN", "AVOID"]
COST = np.array([[0.0, 3.0, 10.0], [1.0, 1.0, 5.0], [3.0, 2.0, 1.0]], dtype=float)
FEATS = ["dist", "closing", "ttc", "roll", "pitch"]


def compute_features(pos, vel, obs_pos, obs_vel=(0.0, 0.0), obs_r=0.4, roll=0.0, pitch=0.0, drone_r=0.15):
    pos = np.asarray(pos, float)[:2]; vel = np.asarray(vel, float)[:2]
    op = np.asarray(obs_pos, float)[:2]; ov = np.asarray(obs_vel, float)[:2]
    rel = op - pos
    dc = float(np.linalg.norm(rel)) + 1e-9
    n = rel / dc
    d = max(dc - float(obs_r) - float(drone_r), 0.0)
    closing = float(-np.dot(ov - vel, n))
    ttc = d / closing if closing > 0.05 else 20.0
    return dict(dist=d, closing=closing, ttc=float(np.clip(ttc, 0, 20)),
                roll=abs(float(roll)), pitch=abs(float(pitch)))


def discretise(f):
    D = "Near" if f["dist"] < 0.8 else ("Mid" if f["dist"] < 2.0 else "Far")
    S = "Low" if f["closing"] < 0.2 else ("Med" if f["closing"] < 1.2 else "High")
    T = "Short" if f["ttc"] < 1.2 else ("Medium" if f["ttc"] < 3.0 else "Long")
    L = "Tilted" if max(f["roll"], f["pitch"]) >= 0.3 else "Level"
    return D, S, T, L


def generate_dataset(n=20000, seed=0):
    rng = np.random.default_rng(seed)
    d = rng.uniform(0.02, 6.0, n); v = rng.uniform(-1.5, 5.0, n)
    roll = rng.uniform(0.0, 0.5, n); pitch = rng.uniform(0.0, 0.5, n)
    ttc = np.where(v > 0.05, d / np.maximum(v, 0.05), 20.0).clip(0, 20)
    k = 1.0 + 0.4 * (np.maximum(roll, pitch) > 0.3)
    danger = (d < 0.6*k) | ((ttc < 1.0*k) & (d < 3.0))
    warn = (d < 1.5*k) | ((ttc < 2.5*k) & (d < 4.0))
    y = np.where(danger, 2, np.where(warn, 1, 0))
    flip = rng.random(n) < 0.05
    y = np.where(flip, np.clip(y + rng.choice([-1, 1], n), 0, 2), y)
    return np.column_stack([d, v, ttc, roll, pitch]), y


class LogisticRiskModel:
    name = "Logistic Regression"
    def __init__(self, lr=0.08, epochs=700, reg=1e-3):
        self.lr, self.epochs, self.reg = lr, epochs, reg
    def fit(self, X, y):
        self.mean = X.mean(0); self.std = X.std(0) + 1e-9
        Z = (X-self.mean)/self.std
        self.W = np.zeros((3, Z.shape[1]+1)); self.W[:,0] = 0
        Y = np.eye(3)[y]
        for _ in range(self.epochs):
            scores = Z @ self.W[:,1:].T + self.W[:,0]
            scores -= scores.max(1, keepdims=True)
            P = np.exp(scores); P /= P.sum(1, keepdims=True)
            grad = (P-Y).T @ np.column_stack([np.ones(len(Z)), Z]) / len(Z)
            grad[:,1:] += self.reg*self.W[:,1:]
            self.W -= self.lr*grad
        return self
    def predict_proba(self, f):
        x = np.array([f[k] for k in FEATS], float)
        z = (x-self.mean)/self.std
        s = self.W[:,0] + self.W[:,1:] @ z; s -= s.max()
        p = np.exp(s); return p/p.sum()


class BayesianRiskModel:
    """Fast learned discrete BN equivalent using Laplace/BDeu-like smoothing."""
    name = "Bayesian Network"
    D = ["Near","Mid","Far"]; S=["Low","Med","High"]; T=["Short","Medium","Long"]; L=["Level","Tilted"]
    def fit(self, X, y, equivalent_sample_size=5.0):
        table = {}
        # empirical joint P(D,S,T,L,R), with BDeu-style symmetric pseudo-counts
        counts = {}
        alpha = equivalent_sample_size/(3*3*3*2*3)
        for row, yi in zip(X, y):
            f = dict(zip(FEATS,row)); key = discretise(f); counts.setdefault(key, np.zeros(3)); counts[key][int(yi)] += 1
        for D in self.D:
            for S in self.S:
                for T in self.T:
                    for L in self.L:
                        key=(D,S,T,L); c=counts.get(key,np.zeros(3))+alpha
                        table[key]=c/c.sum()
        self.table=table; return self
    def predict_proba(self, f): return self.table[discretise(f)]


def decide(probs):
    return int(np.argmin(COST @ np.asarray(probs,float)))


class RiskAvoidanceController:
    """Goal attraction + expected-cost risk decision + side hysteresis."""
    def __init__(self, model, v_cruise=1.5, v_avoid=1.8, v_max=2.0, sense_range=6.0):
        self.model=model; self.v_cruise=v_cruise; self.v_avoid=v_avoid; self.v_max=v_max; self.sense_range=sense_range; self.side=None
    def reset(self): self.side=None
    def command(self, pos, vel, goal, obstacle, roll=0.0, pitch=0.0, left_clear=99, right_clear=99):
        pos=np.asarray(pos,float)[:2]; vel=np.asarray(vel,float)[:2]; goal=np.asarray(goal,float)[:2]
        dgoal=goal-pos; ng=np.linalg.norm(dgoal)+1e-9
        v_pref=self.v_cruise*dgoal/ng*min(1.0,ng/1.0)
        if obstacle is None:
            return v_pref, dict(action=0,probs=np.array([1.,0.,0.]),risk="SAFE",dist=99.,closing=0.,ttc=20.,side=0)
        op,ov,rad=obstacle
        f=compute_features(pos,vel,op,ov,rad,roll,pitch)
        if f['dist']>self.sense_range:
            return v_pref, dict(action=0,probs=np.array([1.,0.,0.]),risk="SAFE",dist=f['dist'],closing=f['closing'],ttc=f['ttc'],side=0)
        probs=self.model.predict_proba(f); a=decide(probs)
        hv=v_pref/(np.linalg.norm(v_pref)+1e-9); perp=np.array([-hv[1],hv[0]])
        if self.side is None:
            # Prefer the side with more clearance; fall back to moving away from obstacle motion.
            self.side = 1.0 if left_clear >= right_clear else -1.0
            if abs(left_clear-right_clear)<0.05:
                s=np.dot(perp,ov)
                self.side = -1.0 if s>0 else 1.0
        if a==0: v=v_pref
        elif a==1:
            gain=0.35 if (f['closing']>0.05 and f['dist']<1.8) else 0.0
            v=0.6*v_pref + gain*self.side*perp
        else:
            n=(op-pos)/(np.linalg.norm(op-pos)+1e-9)
            v=self.v_avoid*(0.9*self.side*perp-0.3*n)+0.4*v_pref
        sp=np.linalg.norm(v)
        if sp>self.v_max: v=v/sp*self.v_max
        return v,dict(action=a,probs=probs,risk=RISK[int(np.argmax(probs))],dist=f['dist'],closing=f['closing'],ttc=f['ttc'],side=self.side)


def train_default_models(seed=0):
    X,y=generate_dataset(seed=seed)
    split=int(.75*len(X)); idx=np.random.default_rng(seed).permutation(len(X)); tr=idx[:split]; te=idx[split:]
    lr=LogisticRiskModel().fit(X[tr],y[tr]); bn=BayesianRiskModel().fit(X[tr],y[tr])
    metrics={}
    for name,m in [('logistic',lr),('bayesian',bn)]:
        pred=np.array([np.argmax(m.predict_proba(dict(zip(FEATS,r)))) for r in X[te]])
        metrics[name]=float((pred==y[te]).mean())
    return {'lr':lr,'bn':bn},metrics
