#!/usr/bin/env python3
"""Losslessly compress completed experiment records; deduplicate source snapshots.

Only an explicitly supplied completed campaign is changed. No prior logs,
assets or runtime dependencies are traversed. The per-file digest verifies
that the compressed bytes recover the original record before replacement.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path


def digest(data):
    return hashlib.sha256(data).hexdigest()


def compact(root):
    state = json.loads((root/'status.json').read_text())
    if state.get('status') != 'complete':
        raise RuntimeError('Refusing to compact an incomplete campaign')
    manifest = dict(policy='Lossless numerical/log compression; identical snapshot files share storage. No raw images, point clouds or bags.', files={}, deduplicated=[], compressed_aliases=[])
    aliases = [(p, p.resolve()) for p in root.rglob('*') if p.is_symlink() and p.is_file()]
    snapshots = {}
    before = sum(p.stat().st_size for p in root.rglob('*') if p.is_file())
    for p in sorted(root.rglob('*')):
        if not p.is_file() or p.is_symlink():
            continue
        if 'implementation' in p.relative_to(root).parts:
            h = digest(p.read_bytes())
            if h in snapshots:
                original = snapshots[h]
                temp = p.with_name(p.name+'.dedup')
                os.link(original,temp)
                os.replace(temp,p)
                manifest['deduplicated'].append(dict(path=str(p.relative_to(root)),same_as=str(original.relative_to(root)),sha256=h))
            else:
                snapshots[h] = p
        if p.suffix not in ('.csv','.jsonl','.log','.tum') or p.stat().st_size < 4096:
            continue
        data = p.read_bytes()
        compressed = gzip.compress(data,compresslevel=6,mtime=0)
        if len(compressed) >= len(data):
            continue
        target = p.with_name(p.name+'.gz')
        if target.exists():
            raise RuntimeError('Existing compressed destination: '+str(target))
        target.write_bytes(compressed)
        assert digest(gzip.decompress(target.read_bytes())) == digest(data)
        manifest['files'][str(p.relative_to(root))] = dict(sha256=digest(data),bytes=len(data),compressed_bytes=len(compressed))
        p.unlink()
    # Shared truth/route/encoder records must remain readable from the fixed
    # reference directory after their original target becomes a gzip file.
    for alias, original in aliases:
        compressed = original.with_name(original.name+'.gz')
        if alias.exists() or not compressed.exists():
            continue
        target = alias.with_name(alias.name+'.gz')
        if target.exists() or target.is_symlink():
            raise RuntimeError('Existing compressed alias: '+str(target))
        target.symlink_to(os.path.relpath(compressed, target.parent))
        alias.unlink()
        manifest['compressed_aliases'].append(dict(source=str(alias.relative_to(root)), archive=str(target.relative_to(root))))
    unique = {}
    for p in root.rglob('*'):
        if p.is_file():
            st=p.stat();unique[st.st_dev,st.st_ino]=st.st_size
    manifest.update(logical_bytes_before=before,unique_bytes_after=sum(unique.values()),all_compressed_digests_verified=True)
    (root/'storage_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:v for k,v in manifest.items() if k not in ('files','deduplicated','compressed_aliases')},indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    compact(parser.parse_args().root.resolve())
