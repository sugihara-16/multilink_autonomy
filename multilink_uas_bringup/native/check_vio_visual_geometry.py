#!/usr/bin/env python3
"""Independent image-pair geometry check on the existing 27 sensor probes."""
import json,re
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/vio_audit'
def main():
 cfg=(ROOT/'logs/20261002-230519-90350/rovio.info').read_text();cams=[]
 for i in range(3):
  b=re.search(r'Camera'+str(i)+r'\s*\{([^}]+)',cfg)[1];v={k:float(x) for k,x in re.findall(r'(qCM_\w|MrMC_\w)\s+([^;\s]+)',b)};cams.append((Rotation.from_quat([v['qCM_'+k] for k in 'xyzw']).as_matrix(),np.array([v['MrMC_'+k] for k in 'xyz'])))
 K=np.array([[138.56406460551023,0,240],[0,138.56406460551023,180],[0,0,1.]]);sift=cv2.SIFT_create(2000);out=[];cv2.setNumThreads(1)
 for f in sorted((ROOT/'logs/zab-offline/environment_analysis').glob('*/metrics.json')):
  folder=f.parent;feats=[sift.detectAndCompute(cv2.imread(str(folder/f'cam_link{i}.png'),0),None) for i in [1,3,4]]
  for c,d in [(0,1),(0,2),(1,2)]:
   kp,ds=feats[c];kq,dt=feats[d]
   if ds is None or dt is None:continue
   matches=[m for m,n in cv2.BFMatcher().knnMatch(ds,dt,k=2) if m.distance<.7*n.distance];r=cams[d][0]@cams[c][0].T;t=cams[d][0]@(cams[c][1]-cams[d][1]);tx=np.array([[0,-t[2],t[1]],[t[2],0,-t[0]],[-t[1],t[0],0]]);F=np.linalg.inv(K).T@tx@r@np.linalg.inv(K)
   if not matches:continue
   a=np.array([[*kp[m.queryIdx].pt,1] for m in matches]);b=np.array([[*kq[m.trainIdx].pt,1] for m in matches]);lines=a@F.T;err=np.abs(np.sum(lines*b,axis=1))/np.linalg.norm(lines[:,:2],axis=1);out.append(dict(sample=folder.name,pair=[c,d],matches=len(matches),median_px=float(np.median(err)),under2px=int(sum(err<2)),geometry_inlier_median_px=float(np.median(err[err<2])) if np.any(err<2) else None))
 (OUT/'epipolar.json').write_text(json.dumps(out,indent=2));print('image geometry probes',len(out))
if __name__=='__main__':main()
