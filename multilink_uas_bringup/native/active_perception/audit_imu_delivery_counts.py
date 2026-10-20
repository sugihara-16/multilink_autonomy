#!/usr/bin/env python3
"""Check actual IMU callback counts between consecutive fusion events."""
import argparse,csv,gzip,json
from pathlib import Path
from report_routes import csv_path

def audit(root,candidate):
 results=[]
 for campaign in sorted(root.iterdir()):
  if not campaign.is_dir() or not campaign.name.startswith((candidate+'-','validation-'+candidate+'-')):continue
  for base in sorted(campaign.glob('*_active')):
   runtime=base/'runtime.json'
   if not runtime.exists() or not json.loads(runtime.read_text()).get('stopped'):continue
   rate=json.loads((base/'imu_sampling.json').read_text())['rate_hz'];period=round(1e9/rate)
   for mode,path in [('active',base),('fixed',base/'fixed')]:
    fpath=csv_path(path/'mimosa/ordered_inputs.csv')
    with (gzip.open(fpath,'rt') if fpath.suffix=='.gz' else fpath.open()) as f:rows=list(csv.DictReader(f))
    previous=0;checked=0;bad=[]
    for row in rows:
     upto=int(row['imu_upto_ns']);count=int(row['imu_count'])
     if previous and upto:
      checked+=1
      if abs(upto-previous-count*period)>2:bad.append(dict(stamp_ns=int(row['stamp_ns']),span_ns=upto-previous,received=count))
     previous=upto
    results.append(dict(campaign=campaign.name,route=base.name,mode=mode,period_ns=period,checked_intervals=checked,mismatches=bad,valid=checked>0 and not bad))
 return dict(scope='Stopped flights only. Acquisition tick counts are checked after the first nonzero prefix, with 2ns rounding tolerance. This supplements the raw duplicate/order and prefix-endpoint audits; no estimator or timestamps are altered.',candidate=candidate,streams=len(results),all_valid=bool(results) and all(r['valid'] for r in results),results=results)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--candidate',required=True);a=p.parse_args()
 report=audit(a.root,a.candidate);(a.root/('imu_delivery_audit_'+a.candidate+'.json')).write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k!='results'}))
 if not report['all_valid']:raise SystemExit(1)
