#!/usr/bin/env python3
"""Run the selected campaign sequentially, with bounded streaming input storage."""
import json,subprocess,sys,os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
c=ROOT/'logs/zab-offline';m=json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())
# First finish the other short bag to check timing before the longer traversals.
labels=['run4']+[x['label'] for x in m['valid_bags'] if x['label'] not in ('run3','run4')]
for label in labels:
 f=c/label/'result.json'
 if f.exists() and json.loads(f.read_text())['status']=='complete':continue
 print('START',label,flush=True)
 subprocess.run([sys.executable,str(ROOT/'offline_odometry.py'),label,'--rate',os.environ.get('UAS_OFFLINE_RATE','1')],check=True)
 result=json.loads(f.read_text())
 if any(set(v)-set(result.get('unavailable_tail_stamps',{}).get(k,[])) for k,v in result['missing_source_stamps'].items()) or any(result['unexpected_output_stamps'].values()):raise RuntimeError('Source/output timestamp audit failed: '+label)
 print('VERIFIED',label,result['counts'],flush=True)
print('ALL OFFLINE RUNS COMPLETE',flush=True)
