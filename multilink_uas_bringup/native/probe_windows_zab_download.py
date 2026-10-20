import subprocess,time,json,shutil,sys
from pathlib import Path
from cache_zab_bags import ROOT,DEST,md5
label='run1_tail_a1';idx=int(sys.argv[2]) if len(sys.argv)>2 else (1 if len(sys.argv)>1 else 0);bind=sys.argv[1] if len(sys.argv)>1 else None;tag=('windows_wifi_probe'+(f'_{idx}' if idx!=1 else '')) if bind else 'windows_probe';name=f'rosbag_{idx}.mcap';e=next(e for e in json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'] if e['label']==label);receipt=json.loads((ROOT/'logs'/e['run']/'cloud_chunks'/f'rosbag_{idx}.json').read_text())
exe='/mnt/c/Users/j-sug/AppData/Local/Temp/codex-rclone-probe/rclone.exe';dest=Path(f'/mnt/c/Users/j-sug/AppData/Local/Temp/codex-rclone-probe/tail_a1_{idx}.mcap')
config=subprocess.check_output(['wslpath','-w','/root/.config/rclone/rclone.conf'],text=True).strip();win_dest=subprocess.check_output(['wslpath','-w',str(dest)],text=True).strip()
started=time.monotonic()
with (DEST/(tag+'.log')).open('w') as f:subprocess.run([exe,*(['--bind',bind] if bind else []),'--config',config,'copyto',e['remote']+'/'+name,win_dest,'--retries','3','--buffer-size','8M','--stats','10s','--stats-one-line','--stats-log-level','NOTICE'],stdout=f,stderr=f,check=True)
elapsed=time.monotonic()-started;assert dest.stat().st_size==receipt['compressed_bytes'] and md5(dest)==receipt['md5'];target=DEST/label/'rosbag'/name;assert not target.exists();shutil.move(str(dest),str(target))
result=dict(bytes=target.stat().st_size,wall_s=elapsed,MBps=target.stat().st_size/elapsed/1e6,md5_verified=True);(DEST/(tag+'.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
