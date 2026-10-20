#!/usr/bin/env python3
"""Finalize a deliberately segmented, stationary simulation capture."""
import argparse,json
from pathlib import Path
from cloud_archive import ROOT,archive_split,finalize,save
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--label',required=True);args=p.parse_args();run=args.run.resolve()
if (ROOT/'state.json').exists():raise RuntimeError('Stop the owned simulation before finalizing a partial capture')
for source in sorted((run/'rosbag').glob('*.db3'),key=lambda x:int(x.stem.rsplit('_',1)[1])):archive_split(run,source)
result=finalize(run)
assert all(not s['missing_intervals'] and not s['nonmonotonic'] for s in result['sensors'].values()),result
f=ROOT/'logs/zab-full-campaign/results.json';results=json.loads(f.read_text());results[args.label]=dict(path=str(run),partial=True,flight=json.loads((run/'planned_segmentation.json').read_text())['final_flight_status'],bag=result,segmentation=json.loads((run/'planned_segmentation.json').read_text()));save(f,results)
print(json.dumps(result),flush=True)
