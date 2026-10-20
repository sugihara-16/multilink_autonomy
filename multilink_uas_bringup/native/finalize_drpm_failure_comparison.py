#!/usr/bin/env python3
"""Validate completed trials and generate six-axis comparison figures."""
import csv,json,re,shutil
from pathlib import Path
import numpy as np
from run_drpm_failure_comparison import OUT,BASE,LABELS,sha,save
from evaluate_drpm_failure_comparison import plot
from plot_livo_comparison import read

def main():
 checks={}
 for label in LABELS:
  d=OUT/'fusion'/label;inp=OUT/'inputs'/label;m=json.loads((d/'metrics.json').read_text());s=json.loads((d/'summary.json').read_text());assert s['returncode']==0
  a=json.loads((inp/'input_audit.json').read_text());v=json.loads((OUT/'rovio'/label/'summary.json').read_text());b=json.loads((inp/'bag_summary.json').read_text())
  assert v['camera']=='link3'
  assert v['counts']==[a['streams']['imu_t']['count'],a['streams']['cam1_t']['count']]
  assert b['counts']['/comparison/imu']==v['counts'][0]
  assert b['counts']['/comparison/points']==a['streams']['cloud_t']['count']
  assert '/comparison/image' not in b['counts']
  assert b['vio_count']==sum(1 for _ in (OUT/'rovio'/label/'result_vio.csv').open())-1
  filechecks={}
  for name in ['odometry_tracking.csv','rovio_tracking.csv','truth_tracking.csv']:
   f=d/name;t=[int(r['stamp_ns']) for r in csv.DictReader(f.open())];assert (np.diff(t)>0).all();filechecks[name]=dict(samples=len(t),sha256=sha(f))
  checks[label]=dict(input_counts_verified=True,timestamp_unique_and_original=True,files=filechecks,rovio_binary_sha256=v['binary_sha256'])
  preview=OUT/'common_preview/fusion'/label/'odometry.csv'
  if preview.exists():
   old=read(preview);new=read(d/'odometry.csv');t=np.intersect1d(old[0],new[0]);i=np.searchsorted(old[0],t);j=np.searchsorted(new[0],t)
   checks[label]['preview_repeat']=dict(samples=len(t),max_position_difference_m=float(np.linalg.norm(old[1][i]-new[1][j],axis=1).max()),max_rotation_difference_deg=float(np.rad2deg((old[2][i].inv()*new[2][j]).magnitude()).max()))
 assert len({v['rovio_binary_sha256'] for v in checks.values()})==1
 plot()
 html=(OUT/'plots/index.html').read_text()
 for f in re.findall(r'(?:src|href)="([^"]+)"',html):assert (OUT/'plots'/f).exists()
 pdf=(OUT/'plots/comparison.pdf').read_bytes();assert len(re.findall(rb'/Type\s*/Page\b',pdf))==6 and pdf.rstrip().endswith(b'%%EOF')
 val=json.loads((OUT/'validation.json').read_text());val['input_and_output_checks']=checks;val['plot_pdf_pages']=6;save(OUT/'validation.json',val)
 # Only derived sensor copies from this task; originals and historical results stay intact.
 removed=0
 for label in LABELS:
  for f in (OUT/'inputs'/label).glob('*.npz'):removed+=f.stat().st_size;f.unlink()
  f=OUT/'inputs'/f'{label}.bag'
  if f.exists():removed+=f.stat().st_size;f.unlink()
 cache=OUT/'cache'
 if cache.is_symlink():
  target=cache.resolve();assert target==Path('/dev/shm/drpm_fast_failures_cache');shutil.rmtree(target);cache.unlink()
 elif cache.exists():shutil.rmtree(cache)
 ram_bag=Path('/dev/shm/drpm_fast_failures_run1_prefix.bag')
 if ram_bag.exists():ram_bag.unlink()
 val['temporary_sensor_copies_removed']=True;val['removed_sensor_bytes_in_finalizer']=removed;save(OUT/'validation.json',val)
 for label in LABELS:
  m=json.loads((OUT/'fusion'/label/'metrics.json').read_text());a=m['common']['drpm'];b=m['common']['fast_official'];f=m['full']['drpm']
  print(f"TABLE {label}: common DRPM {a['position_rmse_m']:.6f}m/{a['rotation_rmse_deg']:.6f}deg; FAST {b['position_rmse_m']:.6f}m/{b['rotation_rmse_deg']:.6f}deg; full DRPM {f['position_rmse_m']:.6f}m/{f['rotation_rmse_deg']:.6f}deg, max {f['position_max_m']:.6f}m")
if __name__=='__main__':main()
