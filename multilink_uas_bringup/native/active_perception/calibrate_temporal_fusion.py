#!/usr/bin/env python3
"""Fit a fixed temporal model from the original independent calibration flights."""
import argparse,gzip,hashlib,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from report_routes import poses,interp,load,csv_path
from calibrate_rovio_relative import relative_error,stats


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('calibration',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('Refusing to overwrite calibration')
    original=json.loads(args.calibration.read_text());training=[]
    for source in original['sources']:
        path=Path(source['path'])
        for name in ['rovio','truth']:
            file=csv_path(path/(name+'.csv'))
            with (gzip.open(file,'rb') if file.suffix=='.gz' else file.open('rb')) as stream:
                digest=hashlib.file_digest(stream,'sha256').hexdigest() if hasattr(hashlib,'file_digest') else hashlib.sha256(stream.read()).hexdigest()
            if digest!=source[name+'_sha256']:raise ValueError('Original calibration input changed: '+str(file))
        v=poses(path/'rovio.csv');r=load(path/'route_tracking.csv');st=r['sim_t'][r['route_t']>0]
        v=v[(v[:,0]>=st[0]*1e9)&(v[:,0]<=st[-1]*1e9)];t=v[:,0]
        if not np.all(abs(np.diff(t)-50000000)<=2):raise ValueError('Input gaps in calibration')
        vp,vq=interp(v,t);gp,gq=interp(poses(path/'truth.csv'),t)
        _,error=relative_error(t,vp,vq,gp,gq,1)
        rotation=(vq[1:-1].inv()*vq[2:]).inv()
        translation=-rotation.apply(vq[1:-1].inv().apply(vp[2:]-vp[1:-1]))
        er=rotation.apply(error[:-1,:3])
        previous=np.c_[er,rotation.apply(error[:-1,3:])+np.cross(translation,er)]
        training.append((previous,error[1:],source['path']))
    numerator=np.mean([np.mean(a*b,axis=0) for a,b,_ in training],axis=0)
    denominator=np.mean([np.mean(a*a,axis=0) for a,b,_ in training],axis=0)
    rho=numerator/denominator
    if not np.isfinite(rho).all() or np.any(abs(rho)>=1):raise ValueError('Nonstationary fitted AR model')
    innovations=[b-rho*a for a,b,_ in training]
    Q=np.mean([z.T@z/len(z) for z in innovations],axis=0)
    if np.linalg.eigvalsh(Q).min()<=0:raise ValueError('Innovation covariance is not positive definite')
    result=dict(original)
    result['protocol']='Fixed AR(1) relative-pose errors, rotation/translation tangent at increment end; previous residual transported using measured increment. Equal weight for the two original fixed-camera calibration flights, no fusion-outcome fitting.'
    result['temporal_model']=dict(period_s=.05,rho=rho.tolist(),innovation_covariance=Q.tolist(),
        transport='rho diagonal times Adjoint(measured current increment inverse)',
        training=[dict(source=p,innovation_statistics=stats(z)) for (_,_,p),z in zip(training,innovations)])
    result['limitations']=['AR(1) is an approximation; higher-order temporal correlations remain',
        'Shared IMU correlation between estimators is not explicitly modelled',
        'Fixed-camera calibration need not describe every moving view exactly']
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(rho=rho.tolist(),output=str(args.output))))


if __name__=='__main__':main()
