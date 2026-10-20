#!/usr/bin/env python3
"""Losslessly archive only finished high-rate encoder CSVs, verifying every byte."""
import argparse,gzip,hashlib,json,shutil
from pathlib import Path

def digest(stream):
    h=hashlib.sha256()
    for part in iter(lambda:stream.read(1024*1024),b''):h.update(part)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args()
    manifest=a.root/'encoder_compression.json'
    rows=json.loads(manifest.read_text()) if manifest.exists() else []
    for runtime in sorted(a.root.glob('*/*_active/runtime.json')):
        r=json.loads(runtime.read_text())
        if not r.get('stopped'):continue
        for folder in [runtime.parent,runtime.parent/'fixed']:
            src=folder/'encoders.csv'
            if not src.exists() or src.stat().st_size<1000000:continue
            dst=src.with_suffix('.csv.gz');tmp=dst.with_suffix('.gz.tmp')
            if dst.exists():raise RuntimeError('Archive already exists: '+str(dst))
            old=src.stat().st_size
            with src.open('rb') as f:sha=digest(f)
            with src.open('rb') as fi,gzip.open(tmp,'wb',compresslevel=3) as fo:shutil.copyfileobj(fi,fo,1024*1024)
            with gzip.open(tmp,'rb') as f:assert digest(f)==sha,'Roundtrip SHA256 mismatch'
            tmp.replace(dst)
            rows.append(dict(source=str(src.relative_to(a.root)),archive=str(dst.relative_to(a.root)),original_bytes=old,compressed_bytes=dst.stat().st_size,sha256_uncompressed=sha,verified=True))
            # Write the durable audit before removing only the redundant copy.
            manifest.write_text(json.dumps(rows,indent=2)+'\n')
            src.unlink()
            (folder/'RESTORE_ENCODERS.txt').write_text('encoders.csv is stored losslessly as encoders.csv.gz.\nThe report tools read it directly. Restore with: gzip -dk encoders.csv.gz\nThe task root encoder_compression.json records the verified SHA256.\n')
    print(json.dumps(dict(files=len(rows),saved_bytes=sum(r['original_bytes']-r['compressed_bytes'] for r in rows))))
if __name__=='__main__':main()
