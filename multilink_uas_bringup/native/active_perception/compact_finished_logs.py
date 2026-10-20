#!/usr/bin/env python3
"""Compact finished research logs without changing any recorded sample.

The two estimators share one physical flight. Identical truth/route files can
therefore share one inode. Processing timing logs are archived byte-for-byte.
Never touch a flight before its runtime manifest confirms all children stopped.
"""
import argparse,gzip,hashlib,json,os,shutil
from pathlib import Path

def digest(stream):
    h=hashlib.sha256()
    for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args()
    manifest=a.root/'compact_log_audit.json'
    rows=json.loads(manifest.read_text()) if manifest.exists() else []
    def save():
        tmp=manifest.with_suffix('.tmp');tmp.write_text(json.dumps(rows,indent=2)+'\n');tmp.replace(manifest)
    for runtime in sorted(a.root.glob('*/*_active/runtime.json')):
        if not json.loads(runtime.read_text()).get('stopped'):continue
        base=runtime.parent
        for name in ('truth.csv','route_tracking.csv'):
            src=base/name;dst=base/'fixed'/name
            if not src.exists() or not dst.exists() or os.path.samefile(src,dst):continue
            if src.stat().st_size!=dst.stat().st_size:continue
            with src.open('rb') as f:sha=digest(f)
            with dst.open('rb') as f:other=digest(f)
            if sha!=other:continue
            size=dst.stat().st_size;tmp=dst.with_suffix('.dedup.tmp')
            os.link(src,tmp);tmp.replace(dst)
            assert os.path.samefile(src,dst)
            rows.append(dict(kind='identical_physical_reference',source=str(src.relative_to(a.root)),
                alias=str(dst.relative_to(a.root)),sha256=sha,saved_bytes=size));save()
            (base/'SHARED_REFERENCES.txt').write_text('Identical truth/route CSVs in fixed/ share storage with the parent flight.\nDo not edit research measurements in place; copy a file first if editing is necessary.\nThe task root compact_log_audit.json records verified hashes.\n')
        src=base/'processing_times.jsonl'
        if not src.exists() or src.stat().st_size<1000000:continue
        dst=src.with_suffix('.jsonl.gz');tmp=dst.with_suffix('.gz.tmp')
        if dst.exists():raise RuntimeError('Archive already exists: '+str(dst))
        size=src.stat().st_size
        with src.open('rb') as f:sha=digest(f)
        with src.open('rb') as fi,gzip.open(tmp,'wb',compresslevel=3) as fo:shutil.copyfileobj(fi,fo,1024*1024)
        with gzip.open(tmp,'rb') as f:assert digest(f)==sha
        tmp.replace(dst)
        rows.append(dict(kind='lossless_gzip',source=str(src.relative_to(a.root)),archive=str(dst.relative_to(a.root)),
            sha256=sha,saved_bytes=size-dst.stat().st_size));save();src.unlink()
        (base/'RESTORE_TIMING_LOG.txt').write_text('Processing timings are archived losslessly as processing_times.jsonl.gz.\nRestore for older timing tools with: gzip -dk processing_times.jsonl.gz\nAccuracy report tools do not require this timing log.\n')
    print(json.dumps(dict(entries=len(rows),saved_bytes=sum(r['saved_bytes'] for r in rows))))

if __name__=='__main__':main()
