#!/usr/bin/env python3
"""Reuse the historical three layouts and 130 m route without geometry changes."""
import argparse
import json
from pathlib import Path
from build_switching_tunnel import build


def main():
    p = argparse.ArgumentParser()
    p.add_argument('output', type=Path)
    a = p.parse_args()
    root = a.output.resolve()
    build(root)
    catalog = json.loads((root/'catalog.json').read_text())
    labels = {'alternating': '左壁→天井→右壁→床',
              'reversed': '床→右壁→天井→左壁', 'uniform': '全方向に模様が豊富'}
    design = dict(distance_m=130, speed_mps=.5, origin=[20., 0., 2.5],
                  description='以前の人工トンネルと同じ形状・模様・130m経路。32mごとに模様の多い面が切り替わる正順・逆順と、全方向に豊富な対照条件。',
                  historical_generator='build_switching_tunnel.py',
                  conditions=[dict(name=k, label=labels[k], world=v['world'])
                              for k,v in catalog.items()])
    (root/'conditions.json').write_text(json.dumps(design,indent=2,ensure_ascii=False)+'\n')


if __name__ == '__main__':
    main()
