#!/usr/bin/env python3
"""Losslessly compress stopped-flight numeric logs, preserving hard-link sharing."""
import argparse,gzip,hashlib,json,os,shutil
from collections import defaultdict
from pathlib import Path

def digest(f):
 h=hashlib.sha256()
 for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--exclude-prefix',default='v17');a=p.parse_args()
 manifest=a.root/'numeric_compression.json';rows=json.loads(manifest.read_text()) if manifest.exists() else []
 groups=defaultdict(list)
 for runtime in a.root.glob('*/*_active/runtime.json'):
  if runtime.parent.parent.name.startswith(a.exclude_prefix):continue
  if not json.loads(runtime.read_text()).get('stopped'):continue
  for path in runtime.parent.rglob('*'):
   if path.suffix not in ('.csv','.tum') or not path.is_file():continue
   st=path.stat()
   if st.st_size<100000:continue
   groups[(st.st_dev,st.st_ino)].append(path)
 for paths in groups.values():
  src=paths[0];dst=src.with_suffix(src.suffix+'.gz');tmp=dst.with_suffix('.gz.tmp')
  if any(p.with_suffix(p.suffix+'.gz').exists() for p in paths):raise RuntimeError('Archive already exists')
  with src.open('rb') as f:sha=digest(f)
  with src.open('rb') as fi,gzip.open(tmp,'wb',compresslevel=3) as fo:shutil.copyfileobj(fi,fo,1024*1024)
  with gzip.open(tmp,'rb') as f:assert digest(f)==sha
  tmp.replace(dst)
  aliases=[]
  for path in paths[1:]:
   other=path.with_suffix(path.suffix+'.gz');os.link(dst,other);aliases.append(str(other.relative_to(a.root)))
  rows.append(dict(sources=[str(p.relative_to(a.root)) for p in paths],archive=str(dst.relative_to(a.root)),aliases=aliases,
                   uncompressed_sha256=sha,original_bytes=src.stat().st_size,compressed_bytes=dst.stat().st_size,verified=True))
  temporary=manifest.with_suffix('.tmp');temporary.write_text(json.dumps(rows,indent=2)+'\n');temporary.replace(manifest)
  for path in paths:path.unlink()
 for runtime in a.root.glob('*/*_active/runtime.json'):
  if runtime.parent.parent.name.startswith(a.exclude_prefix):continue
  if not json.loads(runtime.read_text()).get('stopped'):continue
  (runtime.parent/'RESTORE_NUMERIC.txt').write_text('CSV/TUM logs are stored losslessly as .gz; all samples and timestamps are preserved.\nCommon accuracy reports read compressed CSVs directly. For older tools restore the required file with gzip -dk FILE.csv.gz.\nThe task root numeric_compression.json records hashes and shared aliases.\n')
 print(json.dumps(dict(groups=len(rows),paths=sum(len(r['sources']) for r in rows),saved_bytes=sum(r['original_bytes']-r['compressed_bytes'] for r in rows))))
if __name__=='__main__':main()
