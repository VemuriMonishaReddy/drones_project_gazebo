#!/usr/bin/env python3
"""Validate Logistic Regression vs Bayesian Network exactly like the reference."""
import sys, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from risk_models import generate_dataset, LogisticRiskModel, BayesianRiskModel

def main(out='results/risk_model_metrics.json'):
    X,y=generate_dataset(20000,0); n=int(.75*len(X)); tr,te=X[:n],X[n:]; yt,ye=y[:n],y[n:]
    lr=LogisticRiskModel().fit(tr,yt); bn=BayesianRiskModel().fit(tr,yt)
    res={}
    for name,m in [('logistic',lr),('bayesian',bn)]:
        p=[]
        for r in te: p.append(int(max(range(3),key=lambda k:m.predict_proba(dict(zip(['dist','closing','ttc','roll','pitch'],r)))[k])))
        res[name]={'accuracy':sum(a==b for a,b in zip(p,ye))/len(ye)}
    res['bayesian_structure']=['Distance->TTC','ClosingSpeed->TTC','Distance->Risk','TTC->Risk','Tilt->Risk']
    Path(out).write_text(json.dumps(res,indent=2)); print(json.dumps(res,indent=2))
if __name__=='__main__': main(sys.argv[1] if len(sys.argv)>1 else 'results/risk_model_metrics.json')
