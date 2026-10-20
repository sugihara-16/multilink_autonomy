#!/usr/bin/env python3
"""Post-hoc association of image tracks with the simulator far-plane boundary."""
import argparse,json
from pathlib import Path
import cv2,numpy as np,yaml
from audit_raw_tracks import images


def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('features',type=Path);p.add_argument('raw_audit',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cv2.setNumThreads(1);f=np.genfromtxt(a.features,delimiter=',',names=True);raw=json.loads(a.raw_audit.read_text());windows={}
    for r in raw['tracks']:
        windows.setdefault(r['id'],[]).append((r['start_ns'],r['start_ns']+r['duration_s']*1e9,r['raw_geometry_rms_px']>=2 or r['positive_fraction']<=.9))
    f=f[(f['status']==4)&(f['age_s']>0)];groups={}
    bounds=np.r_[0,np.flatnonzero(np.diff(f['stamp_ns']))+1,len(f)]
    for s,e in zip(bounds[:-1],bounds[1:]):groups[int(f['stamp_ns'][s])]=f[s:e]
    K=np.array(yaml.safe_load((a.flight/'cam0.yaml').read_text())['camera_matrix']['data']).reshape(3,3);measurements=[]
    for stamp,im in images(a.flight/'vio_input.bin'):
        if stamp%1000000000 or stamp not in groups:continue
        bright=(im>200).astype(np.uint8);n,labels,stats,_=cv2.connectedComponentsWithStats(bright,8)
        mask=np.zeros(im.shape,np.uint8)
        for i in range(1,n):
            if stats[i,cv2.CC_STAT_AREA]>100:mask[labels==i]=1
        distance=cv2.distanceTransform(1-mask,cv2.DIST_L2,5)
        for row in groups[stamp]:
            intervals=windows.get(int(row['id']),[]);found=[bad for lo,hi,bad in intervals if lo<=stamp<=hi]
            if not found:continue
            bearing=np.array([row[k] for k in ('mx','my','mz')]);uv=K@bearing
            if uv[2]<=0:continue
            x,y=np.rint(uv[:2]/uv[2]).astype(int)
            if 0<=x<im.shape[1] and 0<=y<im.shape[0]:measurements.append({'stamp_ns':stamp,'id':int(row['id']),'inconsistent_geometry':bool(any(found)),'distance_to_bright_region_px':float(distance[y,x])})
    report={'scope':__doc__,'threshold':200,'region_min_pixels':100,'samples':len(measurements),'groups':{}}
    for bad in [False,True]:
        rows=[m for m in measurements if m['inconsistent_geometry']==bad];ds=np.array([r['distance_to_bright_region_px'] for r in rows]);report['groups'][str(bad)]={'samples':len(rows),'distance_p10_p50_p90_px':np.quantile(ds,[.1,.5,.9]).tolist(),'within_8px_fraction':float(np.mean(ds<=8)),'within_24px_fraction':float(np.mean(ds<=24))}
    report['observations']=measurements;a.output.parent.mkdir(exist_ok=True,parents=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='observations'},indent=2))


if __name__=='__main__':main()
