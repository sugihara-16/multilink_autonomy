#!/usr/bin/env python3
"""Keep checksum-verified original MCAP bags locally for repeated experiments."""
import argparse,concurrent.futures,datetime,hashlib,json,shutil,subprocess,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent
DEST=ROOT/'logs/zab-local-bags'
LABELS=['run1_prefix','run1_tail_a1','run1_tail_b1']
def md5(p):
 h=hashlib.md5()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(4*1024**2),b''):h.update(b)
 return h.hexdigest()
def save(p,v):
 tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(v,indent=2));tmp.replace(p)
def main(wifi_ip=None):
 DEST.mkdir(parents=True,exist_ok=True);entries=json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'];tasks=[];bags={}
 for label in LABELS:
  e=next(e for e in entries if e['label']==label);src=ROOT/'logs'/e['run'];dest=DEST/label/'rosbag';dest.mkdir(parents=True,exist_ok=True)
  meta=yaml.safe_load((src/'cloud_metadata.yaml').read_text());names=meta['rosbag2_bagfile_information']['relative_file_paths'];receipts={n:json.loads((src/'cloud_chunks'/(Path(n).stem+'.json')).read_text()) for n in names}
  bags[label]=dict(remote=e['remote'],metadata=meta,receipts=receipts,dest=dest)
  for n in names:tasks.append((label,n,receipts[n]))
 total=sum(r['compressed_bytes'] for _,_,r in tasks);existing=sum((DEST/l/'rosbag'/n).stat().st_size for l,n,r in tasks if (DEST/l/'rosbag'/n).exists())
 assert shutil.disk_usage('/mnt/c').free>max(0,total-existing)+15*2**30,'Not enough physical Windows space for complete bags and reserve'
 started=time.monotonic();verified={l:{} for l in LABELS}
 win_dir=Path('/mnt/c/Users/j-sug/AppData/Local/Temp/codex-rclone-probe')
 win_exe=win_dir/'rclone.exe'
 win_config=subprocess.check_output(['wslpath','-w','/root/.config/rclone/rclone.conf'],text=True).strip() if wifi_ip else None
 if wifi_ip:assert win_exe.is_file()
 def fetch(task,route):
  label,n,r=task;p=DEST/label/'rosbag'/n;expected=r['md5']
  if not(p.exists() and p.stat().st_size==r['compressed_bytes'] and md5(p)==expected):
   assert shutil.disk_usage('/mnt/c').free>10*2**30,'Physical disk reserve reached'
   begin=time.monotonic()
   if route=='wifi':
    stage=win_dir/(label+'__'+n);target=subprocess.check_output(['wslpath','-w',str(stage)],text=True).strip();cmd=[str(win_exe),'--config',win_config,'--bind',wifi_ip,'copyto',bags[label]['remote']+'/'+n,target]
   else:cmd=['rclone','copyto',bags[label]['remote']+'/'+n,str(p)]
   with (DEST/label/(n+'.download.log')).open('a') as log:
    subprocess.run(cmd+['--retries','4','--low-level-retries','10','--buffer-size','8M'],stdout=log,stderr=log,check=True)
   if route=='wifi':
    assert stage.stat().st_size==r['compressed_bytes'] and md5(stage)==expected
    shutil.move(str(stage),str(p))
   save(DEST/label/(n+'.transfer.json'),dict(route=route,wall_s=time.monotonic()-begin,bytes=p.stat().st_size))
   assert p.stat().st_size==r['compressed_bytes'] and md5(p)==expected,f'Checksum mismatch: {p}'
  return label,n,dict(md5=expected,bytes=p.stat().st_size)
 with concurrent.futures.ThreadPoolExecutor(max_workers=4 if wifi_ip else 6) as pool, concurrent.futures.ThreadPoolExecutor(max_workers=2) as wired:
  futures=[(wired if wifi_ip and i%4==3 else pool).submit(fetch,t,'ethernet' if not wifi_ip or i%4==3 else 'wifi') for i,t in enumerate(tasks)]
  for future in concurrent.futures.as_completed(futures):
   l,n,record=future.result();verified[l][n]=record
   progress=dict(status='downloading',verified_files=sum(len(v) for v in verified.values()),total_files=len(tasks),verified_bytes=sum(r['bytes'] for v in verified.values() for r in v.values()),total_bytes=total,wall_s=time.monotonic()-started)
   save(DEST/'progress.json',progress);print(json.dumps(progress),flush=True)
 for l,b in bags.items():
  (b['dest']/'metadata.yaml').write_text(yaml.safe_dump(b['metadata'],sort_keys=False))
  save(DEST/l/'verified.json',dict(label=l,source_remote=b['remote'],files=verified[l],verified_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),retention='Persistent original sensor bags; do not delete during experiment cleanup'))
 save(DEST/'progress.json',dict(status='download_verified',total_bytes=total,total_files=len(tasks),wall_s=time.monotonic()-started))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--wifi-ip');a=p.parse_args();main(a.wifi_ip)
