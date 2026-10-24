#!/usr/bin/env python3
"""Export all repeated fixed-view advantages and their unfiltered time histories."""
import argparse,csv,json,re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_direction_repeats import LABEL


def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();out=a.root/'report'
 summary=json.loads((out/'summary.json').read_text())
 with (out/'visual_samples.csv').open() as f:samples=list(csv.DictReader(f))
 candidates=[];selected=[]
 for key,s in summary['sections'].items():
  d=s['windows']['10'];N=len(d['trials']);local=[]
  if N!=3:continue
  for n,c in d['comparisons'].items():
   if c['fixed_wins']!=3 or c['reduction_of_mean_percent']<20:continue
   sensitivities={W:s['windows'][W]['comparisons'][n] for W in ['5','10','20']}
   record=dict(section=key,route=s['route'],segment=s['segment'],bounds_s=s['bounds_s'],direction=n,
               primary=c,sensitivity=sensitivities,
               all_window_lengths_all_three_wins=all(v['fixed_wins']==3 for v in sensitivities.values()))
   candidates.append(record);local.append(record)
  if local:
   best=max(local,key=lambda c:c['primary']['active']['mean']-c['primary']['fixed']['mean'])
   selected.append(best)
 result=dict(complete=summary['complete'],criterion='All three new paired flights lower fixed VIO velocity RMS and >=20 percent lower flight-mean RMS. Exploratory cases, no significance claim.',all_candidates=candidates,representatives=selected)
 (out/'cases.json').write_text(json.dumps(result,indent=2)+'\n')
 html='<section id="fixed-cases"><h2>反復して固定が有利だったケース</h2><p>各区間の代表方向は、3回とも固定が勝ち、平均誤差が20%以上小さい方向から選択。全候補と5/20秒確認は<a href="cases.json">cases.json</a>、<a href="cases.pdf">時系列PDF</a>。これはデータ収集後の代表例の選択です。</p><table><tr><th>区間</th><th>固定方向</th><th>固定[m/s]</th><th>偏向[m/s]</th><th>低減</th><th>5/10/20秒すべて3勝</th></tr>'
 with PdfPages(out/'cases.pdf') as pdf:
  for c in selected:
   key=c['section'];n=c['direction'];d=summary['sections'][key]['windows']['10'];trials=d['trials'];p=c['primary']
   html+=f'<tr><td>{c["route"]} {c["bounds_s"]}</td><td>{LABEL[n]}</td><td>{p["fixed"]["mean"]:.4f}</td><td>{p["active"]["mean"]:.4f}</td><td>{p["reduction_of_mean_percent"]:.1f}%</td><td>{c["all_window_lengths_all_three_wins"]}</td></tr>'
   fig,axs=plt.subplots(4,3,figsize=(16,12),squeeze=False)
   for col,t in enumerate(trials):
    for method,color in [(n,'#2677ac'),('active','#be4a36')]:
     ss=[s for s in samples if s['trial']==t['trial'] and s['segment']==str(c['segment']) and s['window_s']=='10' and s['method']==method]
     ts=np.array([float(s['elapsed_s']) for s in ss])
     axs[0,col].plot(ts,[float(s['error_mps']) for s in ss],color=color,label=LABEL[method])
     axs[1,col].plot(ts,[float(s['sigma_mps']) for s in ss],color=color)
     axs[2,col].plot(ts,[float(s['features']) for s in ss],color=color)
     if method=='active':
      axs[3,col].plot(ts,[float(s['yaw_deg']) for s in ss],label='Gimbal yaw',color='#777')
      axs[3,col].plot(ts,[float(s['pitch_deg']) for s in ss],label='Gimbal pitch',color='#c67c10')
    axs[0,col].axhline(0,color='gray',lw=.7);axs[0,col].set_title(t['trial']);axs[0,col].legend(fontsize=8)
    axs[3,col].legend(fontsize=8);axs[3,col].set_xlabel('Elapsed time from tracking onset [s]')
    for row in range(4):axs[row,col].grid(alpha=.25)
   for row,label in enumerate(['VIO velocity error [m/s]','Logged constraint sigma [m/s]','Tracked features','Gimbal angle [deg]']):axs[row,0].set_ylabel(label)
   fig.suptitle(f'{c["route"]} {c["bounds_s"]} s: {LABEL[n]} versus gimbal (all trials)');fig.tight_layout();pdf.savefig(fig);fig.savefig(out/(key+'_case.png'),dpi=120);plt.close(fig)
 html+='</table>'
 for c in selected:html+=f'<h3>{c["route"]} {c["bounds_s"]}</h3><img src="{c["section"]}_case.png">'
 html+='</section>'
 path=out/'index.html';text=re.sub(r'<section id="fixed-cases">.*?</section>','',path.read_text(),flags=re.S);path.write_text(text+html)
 print(json.dumps(dict(candidates=len(candidates),representatives=len(selected))))

if __name__=='__main__':main()
