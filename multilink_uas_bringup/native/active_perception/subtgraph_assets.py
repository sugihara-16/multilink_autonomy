#!/usr/bin/env python3
"""Fetch two pinned official benchmark worlds, without cloning the benchmark."""
import hashlib
import json
from pathlib import Path
import shutil
import urllib.request

REV = 'a6cd7d213e72883c2dbb8aad36123c5da68cd6c2'
ROOT = Path('/root/multilink_autonomy_ws/assets/subtgraph')
MODELS = {
    'operational': ('3bb92470394761a75eb0d73e86172d08b487f9dabebc56c4dbb33ea607ff8447',
                    ['CaveWall_Operational.png']),
    'lavatube': ('80bde3daa8d984d912454052209bc5ed817f2b475d63b240530f1ebf102df771',
                ['CaveWall_LavaTube.png', 'RockPile_LavaTube.png']),
}


def main():
    manifest = {'repository': 'https://github.com/LTU-RAI/SubTGraph',
                'revision': REV, 'files': {}}
    for family, (mesh_sha, textures) in MODELS.items():
        for name in ['subtgraph.obj', 'subtgraph.mtl',
                     *['textures/' + t for t in textures],
                     *['pkl/subtgraph.' + t + '.pkl' for t in ['spatial', 'structural', 'topological']]]:
            path = ROOT / family / '01' / name
            remote = f'benchmark/{family}/01/{name}'
            host = 'media.githubusercontent.com/media' if name.endswith('.obj') else 'raw.githubusercontent.com'
            url = f'https://{host}/LTU-RAI/SubTGraph/{REV}/{remote}'
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists():
                if shutil.disk_usage('/mnt/c').free < 10 * 1024**3:
                    raise RuntimeError('Windows free space below 10 GiB')
                print('DOWNLOAD', remote, flush=True)
                temp = path.with_suffix(path.suffix + '.part')
                with urllib.request.urlopen(url, timeout=60) as src, temp.open('wb') as dst:
                    shutil.copyfileobj(src, dst, 1024**2)
                temp.replace(path)
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1024**2), b''):
                    digest.update(block)
            sha = digest.hexdigest()
            if name.endswith('.obj') and sha != mesh_sha:
                raise RuntimeError('LFS content hash mismatch: ' + str(path))
            manifest['files'][str(path.relative_to(ROOT))] = {
                'url': url, 'sha256': sha, 'bytes': path.stat().st_size}
            (ROOT / 'source.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('DONE', flush=True)


if __name__ == '__main__':
    main()
