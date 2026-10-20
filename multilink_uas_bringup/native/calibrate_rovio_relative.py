#!/usr/bin/env python3
"""Measure held-out, single-camera relative-pose errors; never tune on fusion output."""
import csv, hashlib, json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import expm, logm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_livo_comparison import read, interpolate, LABELS

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'logs/zab-offline/livo_comparison'
OUT=ROOT/'logs/zab-offline/livo_calibrated'

def se3_log(rotation, translation):
    phi=rotation.as_rotvec();theta=np.linalg.norm(phi,axis=1)
    a=np.full(len(theta),1/12.)
    k=theta>1e-4
    a[k]=(1-0.5*theta[k]/np.tan(0.5*theta[k]))/theta[k]**2
    rho=translation-0.5*np.cross(phi,translation)+a[:,None]*np.cross(phi,np.cross(phi,translation))
    return np.c_[phi,rho]

def relative_error(t,p,r,gp,gr,lag):
    dr=r[:-lag].inv()*r[lag:];dt=r[:-lag].inv().apply(p[lag:]-p[:-lag])
    dg=gr[:-lag].inv()*gr[lag:];dgt=gr[:-lag].inv().apply(gp[lag:]-gp[:-lag])
    # Exactly the Pose3 BetweenFactor residual: Log(Z_vio^-1 * Z_truth).
    return t[lag:],se3_log(dr.inv()*dg,dr.inv().apply(dgt-dt))

def stats(e):
    sd=e.std(axis=0,ddof=1);acf=[]
    for i in range(6):acf.append(float(np.corrcoef(e[:-1,i],e[1:,i])[0,1]))
    return dict(samples=len(e),bias=e.mean(axis=0).tolist(),std=sd.tolist(),
                rmse=np.sqrt((e*e).mean(axis=0)).tolist(),p95_abs=np.quantile(abs(e),.95,axis=0).tolist(),
                lag1_correlation=acf)

def calibration(e):
    # Include squared bias; do not correct measurements using ground truth.
    second=e.T@e/len(e)
    scale=np.sqrt(second.diagonal());corr=second/np.outer(scale,scale)
    val,vec=np.linalg.eigh(corr);floor=1e-6
    used=(vec@np.diag(np.maximum(val,floor))@vec.T)*np.outer(scale,scale)
    return used,dict(raw_second_moment=second.tolist(),normalized_eigenvalues=val.tolist(),
                     numerical_eigenvalue_floor=floor,floored_count=int((val<floor).sum()))

def check_log():
    rng=np.random.default_rng(140)
    r=Rotation.from_rotvec(rng.normal(size=(30,3))*.2);p=rng.normal(size=(30,3))
    x=se3_log(r,p)
    for R,t,y in zip(r.as_matrix(),p,x):
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
        L=logm(T).real
        expected=np.r_[[L[2,1],L[0,2],L[1,0]],L[:3,3]]
        assert np.allclose(y,expected,atol=1e-11)
    assert np.array_equal(se3_log(Rotation.identity(2),np.ones((2,3))),np.c_[np.zeros((2,3)),np.ones((2,3))])

def main():
    check_log();OUT.mkdir(exist_ok=True);summaries={};errors={}
    for label in LABELS:
        path=BASE/'rovio'/label/'result_vio.csv';v=read(path)
        g=read(BASE/'inputs'/label/'truth.csv')
        window=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window']
        t,p,r=v;keep=(t>=window['start_ns'])&(t<=window['end_ns']);t,p,r=t[keep],p[keep],r[keep]
        assert np.all(np.diff(t)==50000000)
        gp,gr=interpolate(g,t);metrics={}
        for lag in [1,2,10,20]:
            te,e=relative_error(t,p,r,gp,gr,lag);metrics[str(lag*.05)]=stats(e)
            if lag==1:errors[label]=(te,e)
        te,e=errors[label]
        dest=OUT/'calibration'/label;dest.mkdir(parents=True,exist_ok=True)
        with (dest/'relative_errors.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['stamp_ns','rx_rad','ry_rad','rz_rad','tx_m','ty_m','tz_m']);w.writerows(zip(te,*e.T))
        summaries[label]=dict(input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),window=window,
                              axes=['rx_rad','ry_rad','rz_rad','tx_m','ty_m','tz_m'],intervals=metrics)
        fig,axes=plt.subplots(6,1,figsize=(12,13),sharex=True)
        for j,ax in enumerate(axes):
            scale=180/np.pi if j<3 else 1000
            ax.plot((te-window['start_ns'])*1e-9,e[:,j]*scale,lw=.6)
            ax.set_ylabel(['Rx [deg]','Ry [deg]','Rz [deg]','Tx [mm]','Ty [mm]','Tz [mm]'][j]);ax.grid(alpha=.25)
        axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | ROVIO 50 ms relative-pose error')
        fig.tight_layout();fig.savefig(dest/'relative_errors.png',dpi=140);plt.close(fig)
    folds={}
    for target in LABELS:
        train=next(x for x in LABELS if x!=target);C,details=calibration(errors[train][1]);e=errors[target][1]
        nis=np.einsum('ni,ij,nj->n',e,np.linalg.inv(C),e)
        folds[target]=dict(training_flight=train,evaluation_flight=target,interval_s=.05,
            covariance=C.tolist(),sigma_rotation_deg=np.rad2deg(np.sqrt(C.diagonal()[:3])).tolist(),
            sigma_translation_m=np.sqrt(C.diagonal()[3:]).tolist(),heldout_normalized_squared_error_mean=float(nis.mean()),
            heldout_fraction_below_chi2_95=float((nis<=12.591587243743977).mean()),**details)
    report=dict(protocol='Two-flight leave-one-flight-out; tracking only; full uncentered error second moment; fixed 50ms nominal covariance; no outcome tuning',
        limitations=['No joint ROVIO/MIMOSA covariance or shared-IMU correction','No temporal whitening; lag correlations reported',
                     'Reported ROVIO marginal pose covariance is not treated as relative-pose covariance','Only two flights in the same simulated tunnel'],
        characteristics=summaries,folds=folds)
    (OUT/'calibration.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:{x:v[x] for x in ['training_flight','sigma_rotation_deg','sigma_translation_m','heldout_normalized_squared_error_mean']} for k,v in folds.items()},indent=2))

if __name__=='__main__':main()
