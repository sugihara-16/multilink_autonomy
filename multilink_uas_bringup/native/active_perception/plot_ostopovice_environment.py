#!/usr/bin/env python3
"""Illustrate source cross-section and random placements, without SLAM inputs."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser();p.add_argument('assets',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    g=np.load(a.assets/'source/interior_geometry.npz');r=g['radius'];theta=g['theta'];z=float(g['center_z'])
    fig,ax=plt.subplots(figsize=(7,6))
    for rr in r[::8]:ax.plot(rr*np.sin(theta),z+rr*np.cos(theta),color='gray',alpha=.2)
    mean=r.mean(0);ax.plot(mean*np.sin(theta),z+mean*np.cos(theta),color='#2565a8',label='Mean original inner section')
    ax.scatter([0],[2],c='black',label='Flight centre');ax.set(xlabel='Y [m]',ylabel='Z [m]',title='Original scan scaled to 5 m median width');ax.set_aspect('equal');ax.legend();ax.grid(alpha=.3);fig.tight_layout();fig.savefig(a.output/'cross_section.png',dpi=150);fig.savefig(a.output/'cross_section.pdf');plt.close(fig)
    catalog=json.loads((a.assets/'catalog.json').read_text());fig,axes=plt.subplots(len(catalog),1,figsize=(15,10),sharex=True)
    ticktheta=np.array([-np.pi,-np.pi/2,0,np.pi/2,np.pi]);ticks=np.interp(ticktheta,np.r_[theta,np.pi],g['arc'])
    for ax,(name,entry) in zip(axes,catalog.items()):
        data=json.loads((Path(entry['world']).parent/'texture_placements.json').read_text())
        for item in data['placements']:
            ax.scatter(item['x_m'],item['arc_m'],s=60*item['size_m']**2,c=[plt.cm.tab20(item['cluster']%20)],alpha=.7,edgecolors='none')
        ax.set(title=name,ylabel='Inner surface',yticks=ticks,yticklabels=['Floor','Right wall','Ceiling','Left wall','Floor'],ylim=(0,float(g['arc'][-1])))
        ax.grid(alpha=.2)
    axes[-1].set(xlim=(0,100),xlabel='Distance along generated tunnel [m]')
    fig.suptitle('Actual random graffiti placements; circle size indicates patch size, colour indicates cluster')
    fig.tight_layout();fig.savefig(a.output/'placements.png',dpi=150);fig.savefig(a.output/'placements.pdf');plt.close(fig)
    print(a.output)


if __name__=='__main__':main()
