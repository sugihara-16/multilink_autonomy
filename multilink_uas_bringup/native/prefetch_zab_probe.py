"""Bounded extra transfers, using six distant chunks not reached by the main queue."""
import concurrent.futures,json,subprocess,time,shutil
from pathlib import Path
from cache_zab_bags import ROOT,DEST,md5
label='run1_tail_b1';entry=next(x for x in json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'] if x['label']==label);src=ROOT/'logs'/entry['run']
def transfer(i):
 n=f'rosbag_{i}.mcap';p=DEST/label/'rosbag'/n;r=json.loads((src/'cloud_chunks'/f'rosbag_{i}.json').read_text());start=time.monotonic()
 assert shutil.disk_usage('/mnt/c').free>15*2**30
 assert not p.exists(),'Probe assumes distant, not yet fetched chunks'
 with (DEST/label/(n+'.probe.log')).open('w') as f:subprocess.run(['rclone','copyto',entry['remote']+'/'+n,str(p),'--retries','4','--buffer-size','8M'],stdout=f,stderr=f,check=True)
 assert p.stat().st_size==r['compressed_bytes'] and md5(p)==r['md5']
 return dict(name=n,bytes=p.stat().st_size,wall_s=time.monotonic()-start)
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:result=list(pool.map(transfer,range(6)))
(DEST/'parallel_probe.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
