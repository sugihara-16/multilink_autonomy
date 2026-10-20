#!/usr/bin/env python3
"""Validate the simulated range noise from fixed indexed rays, before takeoff."""
import json,hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'logs/zab-offline/drpm_noise';BASE=ROOT/'logs/zab-offline/livo_comparison'
results={}
for label in ['run2a_end','run1_tail_a2_prefix']:
    run={'run2a_end':'20261003-011702-110549','run1_tail_a2_prefix':'20261002-230519-90350'}[label]
    urdf=ROOT/'logs'/run/'dragon_slam.urdf'
    sensor=ET.parse(urdf).find('.//sensor[@name="base_lidar"]')
    assert sensor.attrib['type']=='gpu_lidar'
    assert sensor.findtext('lidar/noise/type')=='gaussian'
    assert float(sensor.findtext('lidar/noise/mean'))==0
    assert float(sensor.findtext('lidar/noise/stddev'))==.03
    a=np.load(BASE/'inputs'/label/'rosbag_0.npz')
    t=a['cloud_t'];select=(t>=5_000_000_000)&(t<=14_000_000_000)
    cloud=a['cloud'][select,:,:3].astype(float)
    # Prespecified unclipped selection: finite in every frame and all ranges
    # at least 0.2m away from either sensor bound. No variance-based selection.
    finite=np.isfinite(cloud).all(axis=(0,2));cloud=cloud[:,finite]
    ranges=np.linalg.norm(cloud,axis=2);valid=(ranges.min(axis=0)>.3)&(ranges.max(axis=0)<29.8)
    cloud=cloud[:,valid];ranges=ranges[:,valid];unit=cloud/ranges[:,:,None]
    ref=unit.mean(axis=0);ref/=np.linalg.norm(ref,axis=1)[:,None]
    residual=cloud-cloud.mean(axis=0)
    parallel=np.einsum('fij,ij->fi',residual,ref)
    perpendicular=residual-parallel[:,:,None]*ref
    tt=a['truth_t'];truth=a['truth'][(tt>=5_000_000_000)&(tt<=14_000_000_000)]
    q=Rotation.from_quat(truth[:,3:7]);span=np.rad2deg((q[0].inv()*q).magnitude().max())
    results[label]=dict(window_s=[5,14],frames=len(cloud),rays=len(ref),
      median_range_std_m=float(np.median(ranges.std(axis=0,ddof=1))),
      pooled_range_std_m=float(np.sqrt((parallel**2).sum()/((len(cloud)-1)*len(ref)))),
      range_std_quantiles_m=np.quantile(ranges.std(axis=0,ddof=1),[.05,.5,.95]).tolist(),
      perpendicular_rms_m=float(np.sqrt((perpendicular**2).sum(axis=2).mean())),
      truth_position_span_m=np.ptp(truth[:,:3],axis=0).tolist(),truth_rotation_span_deg=float(span))
    results[label]['urdf']=str(urdf)
    results[label]['urdf_sha256']=hashlib.sha256(urdf.read_bytes()).hexdigest()
    results[label]['configured_range_sigma_m']=.03
    assert .028<results[label]['median_range_std_m']<.032
    assert results[label]['perpendicular_rms_m']<2e-6
(OUT/'sensor_noise.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
