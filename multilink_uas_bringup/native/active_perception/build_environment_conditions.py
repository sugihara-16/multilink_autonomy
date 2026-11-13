#!/usr/bin/env python3
"""Five predeclared environmental conditions; no estimator outcomes used."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image, ImageEnhance
from build_weathered_tunnel import build

CONDITIONS = [
    ('baseline', '基準：前回と同じ形状・模様', 24301),
    ('layout', '壁模様の配置変更', 24302),
    ('low_contrast', '壁模様のコントラスト45%', 24301),
    ('dim', '環境光35%', 24301),
    ('wide', 'トンネル幅7m（基準5m）', 24301),
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    a = p.parse_args()
    root = a.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    records = []
    for name, label, seed in CONDITIONS:
        target = root / name
        if name in ('baseline', 'layout'):
            build(target, seed, 100)
        else:
            original = root / 'baseline'
            shutil.copytree(original, target)
            for f in target.rglob('*'):
                if f.is_file() and f.suffix in ('.json', '.sdf'):
                    f.write_text(f.read_text().replace(str(original), str(target)))
        world = ET.parse(target / 'world.sdf')
        env = json.loads((target / 'environment.json').read_text())
        if name == 'low_contrast':
            for f in (target / 'textures').glob('*.png'):
                with Image.open(f) as im:
                    ImageEnhance.Contrast(im).enhance(.45).save(f)
            env['texture_contrast_multiplier'] = .45
        if name == 'dim':
            world.find('./world/scene/ambient').text = '.35 .35 .35 1'
            env['ambient_rgb'] = [.35] * 3
        if name == 'wide':
            for mesh in world.findall('./world/model/link/*/geometry/mesh'):
                ET.SubElement(mesh, 'scale').text = '1 1.4 1'
            env['width_m'] = 7
            env['mesh_scale_xyz'] = [1, 1.4, 1]
        ET.indent(world)
        world.write(target / 'world.sdf', encoding='unicode')
        env.update(condition=name, label=label, reference='baseline', independent_variable=name)
        (target / 'environment.json').write_text(json.dumps(env, indent=2, ensure_ascii=False))
        # Texture overviews copied from baseline are not actual rendered QA.
        if name not in ('baseline', 'layout'):
            for f in ('texture_overview.jpg', 'distribution.png'):
                (target / f).unlink()
        hashes = {str(f.relative_to(target)): hashlib.sha256(f.read_bytes()).hexdigest()
                  for f in sorted(target.rglob('*')) if f.is_file() and f.name != 'hashes.json'}
        (target / 'hashes.json').write_text(json.dumps(hashes, indent=2))
        records.append(dict(name=name, label=label, world=str(target/'world.sdf'), seed=seed, environment=env))
    (root / 'conditions.json').write_text(json.dumps(dict(conditions=records,
        common=dict(distance_m=90, speed_mps=.5, noise_seeds=[20262801,20262802,20262803]),
        design='Baseline plus four one-factor changes: texture layout, texture contrast, ambient light, width. Not five field sites. Ground truth and generation metadata are excluded from estimation and view selection.'), indent=2, ensure_ascii=False))
    print(root / 'conditions.json', flush=True)


if __name__ == '__main__':
    main()
