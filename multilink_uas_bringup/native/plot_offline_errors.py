#!/usr/bin/env python3
"""Timestamp-matched, start-aligned six-axis errors for each independent flight."""
import argparse,csv,json,warnings
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MaxNLocator
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'logs/zab-offline/results'
OUT=ROOT/'logs/zab-offline/error_plots'
COLORS={'lio':'#0072B2','vio':'#D55E00'}
NAMES=['X error [m]','Y error [m]','Z error [m]','Roll error [deg]','Pitch error [deg]','Yaw error [deg]']
def read(path):
 with path.open() as f: rows=list(csv.DictReader(f))
 t=np.array([int(r['stamp_ns']) for r in rows],dtype=np.int64)
 p=np.array([[float(r[k]) for k in ['x','y','z']] for r in rows])
 q=Rotation.from_quat([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in rows])
 assert np.all(np.diff(t)>0)
 return t,p,q

def errors(folder):
 truth=read(folder/'truth.csv'); estimates={k:read(folder/(k+'.csv')) for k in COLORS}
 tt,tp,tq=truth;origin=int(tt[0]);ts=(tt-origin)*1e-9;slerp=Slerp(ts,tq)
 shared=np.intersect1d(estimates['lio'][0],estimates['vio'][0]);shared=shared[(shared>=tt[0])&(shared<=tt[-1])];anchor=int(shared[0])
 def gt(t):
  seconds=(t-origin)*1e-9
  return np.column_stack([np.interp(seconds,ts,tp[:,j]) for j in range(3)]),slerp(seconds)
 data={};meta={'anchor_ns':anchor,'alignment':'Single rigid pose alignment at first common estimate timestamp inside truth support. No scale fitting or subsequent realignment.','axes':'Position: truth world XYZ. Attitude: xyz Euler angles of R_truth^-1 R_aligned (truth body frame), degrees.','truth_interpolation':'Linear position and quaternion SLERP at unchanged estimate timestamps; no extrapolation.','estimators':{}}
 for k,(t,p,q) in estimates.items():
  i=np.searchsorted(t,anchor);ga,gr=gt(np.array([anchor]));ra=gr[0]*q[i].inv();pa=ga[0]-ra.apply(p[i])
  valid=(t>=anchor)&(t<=tt[-1]);t=t[valid];p=p[valid];q=q[valid];g,r=gt(t)
  brackets=np.searchsorted(tt,t,side='left');brackets=np.clip(brackets,1,len(tt)-1)
  assert np.max(tt[brackets]-tt[brackets-1])<100000000
  pe=ra.apply(p)+pa-g
  with warnings.catch_warnings(record=True) as ws:
   ae=(r.inv()*(ra*q)).as_euler('xyz',degrees=True)
  assert not ws,'Euler singularity requires explicit treatment'
  e=np.column_stack([pe,ae]);assert np.isfinite(e).all()
  data[k]=(t,e)
  meta['estimators'][k]={'count':len(t),'excluded_at_boundaries':int(len(valid)-valid.sum()),'rotation_xyzw':ra.as_quat().tolist(),'translation_m':pa.tolist(),'max_abs':np.max(np.abs(e),axis=0).tolist(),'rmse':np.sqrt(np.mean(e**2,axis=0)).tolist()}
 return data,meta

def plot(label,data,meta,pdf):
 fig,axes=plt.subplots(6,1,figsize=(14,17),sharex=True)
 fig.subplots_adjust(hspace=.35,left=.095,right=.97,top=.935,bottom=.065)
 start=meta['anchor_ns'];end=max(t[-1] for t,e in data.values());duration=(end-start)*1e-9
 meta['display']={}
 for j,(ax,name) in enumerate(zip(axes,NAMES)):
  full=np.concatenate([e[:,j] for t,e in data.values()]);lo=min(0.,float(full.min()));hi=max(0.,float(full.max()))
  pad=max(.002 if j<3 else .02,(hi-lo)*.06)
  limits=(lo-pad,hi+pad)
  meta['display'][name]={'visible_ranges':[limits],'omitted_sample_counts':{k:0 for k in data}}
  for k,(t,e) in data.items():
   y=e[:,j].copy()
   # Only disconnect the Euler representation wrap; no y-axis omission.
   if j>=3:y[np.r_[False,np.abs(np.diff(y))>180]]=np.nan
   ax.plot((t-start)*1e-9,y,color=COLORS[k],lw=1.05,alpha=.90,label=k.upper())
  ax.set_ylim(*limits);ax.set_xlim(0,duration);ax.grid(True,alpha=.22);ax.axhline(0,color='#444',lw=.65,alpha=.6)
  ax.yaxis.set_major_locator(MaxNLocator(5));ax.xaxis.set_major_locator(MaxNLocator(9));ax.tick_params(labelsize=9,labelbottom=True)
  ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
  ax.set_title(name,loc='left',fontsize=11,fontweight='bold',pad=6)
 axes[0].legend(loc='upper right',ncol=2,framealpha=.9)
 axes[-1].set_xlabel('Time since common tracking start [s]',fontsize=11)
 fig.suptitle(f'{label}  |  LIO / VIO error against ground truth',fontsize=18,x=.095,ha='left',y=.977)
 revision='VIO recomputed  |  ' if meta.get('vio_variant')=='gyro_matched_depth10' else ''
 fig.text(.095,.952,revision+'Tracking only  |  One initial position/orientation alignment  |  Full y-axis range',fontsize=11,color='#555')
 fig.text(.095,.021,'XYZ: truth world frame. Roll/pitch/yaw: relative rotation in truth body frame.\nOriginal timestamps retained; truth interpolated at estimate times. No y-axis breaks or clipping.',fontsize=9,color='#555')
 fig.savefig(OUT/(label+'.png'),dpi=160);fig.savefig(OUT/(label+'.pdf'));pdf.savefig(fig);plt.close(fig)

def main():
 OUT.mkdir(exist_ok=True);manifest=json.loads((SOURCE/'manifest.json').read_text());summary={}
 assert manifest['status']=='complete','Refusing to plot an incomplete campaign'
 provenance={'source_directory':str(SOURCE),'variant':manifest.get('variant','original'),'settings':manifest.get('settings'),'source_format':manifest.get('format','Original CSV and MCAP')}
 with PdfPages(OUT/'all_flights.pdf') as pdf:
  for run in manifest['runs']:
   label=run['label'];data,meta=errors(SOURCE/label);meta['source_run']=run['source_run'];meta['source_window']=run['window'];meta['vio_variant']=manifest.get('variant','original');plot(label,data,meta,pdf);summary[label]=meta
   for k,(t,e) in data.items():
    with (OUT/(label+'_'+k+'_errors.csv')).open('w') as f:
     w=csv.writer(f);w.writerow(['stamp_ns','elapsed_s','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg'])
     for stamp,row in zip(t,e):w.writerow([int(stamp),(int(stamp)-meta['anchor_ns'])*1e-9,*row])
   print(label,{k:meta['estimators'][k]['max_abs'][:3] for k in COLORS},flush=True)
 (OUT/'evaluation.json').write_text(json.dumps(summary,indent=2)+'\n')
 (OUT/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
 (OUT/'generation.log').write_text(json.dumps({'status':'complete',**provenance,'flights':list(summary)},indent=2)+'\n')
 note='<p>VIO再計算: 模擬ジャイロの誤差モデルに整合し、特徴点の初期距離を10 mへ変更。全14飛行で同じ設定。LIOと真値は前回と同じデータです。<a href="provenance.json">データ出典・設定</a></p>' if manifest.get('variant')=='gyro_matched_depth10' else ''
 (OUT/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>ZaB LIO / VIO errors</title><style>body{max-width:1400px;margin:30px auto;font:16px sans-serif;background:#eee}img{width:100%;background:white}nav a{margin-right:14px}section{margin:30px 0}</style><h1>ZaB — LIO / VIO 六軸誤差</h1><p>青: LIO、橙: VIO。追従開始時に一度だけ位置・向きを整合。各図は独立した1飛行。縦軸は省略せず、全誤差範囲を表示。</p>'+note+'<p><a href="all_flights.pdf">全14飛行 PDF</a> · <a href="README.txt">比較方法・制約</a></p><nav>'+''.join(f'<a href="#{k}">{k}</a>' for k in summary)+'</nav>'+''.join(f'<section id="{k}"><h2>{k} <a href="{k}.pdf">PDF</a></h2><img loading="lazy" src="{k}.png"></section>' for k in summary))
 (OUT/'README.txt').write_text('''全14飛行の真値に対するLIO/VIO六軸誤差
index.htmlで一覧、all_flights.pdfで全14ページ、各飛行PNG/PDFでも閲覧できます。
青=LIO、橙=VIO。各飛行内はファイル分割に関係なく1図に統合しています。
14件は別々の離陸・時計・推定器初期化を持つため結合していません。

各飛行の真値の時間範囲内にあるLIO/VIO共通の最初の取得時刻で、位置・姿勢を一度だけ真値に合わせます。
以後その固定変換を使用。全軌道フィッティング、スケール補正、途中の再整合はありません。
初期位置・姿勢誤差はこの整合により除去されるため、図は開始後のずれを示します。
真値は推定の元timestampへ位置を線形補間、姿勢を球面補間(SLERP)します。外挿しません。
共通開始より前・真値終端より後の境界サンプルだけを除外し、件数をevaluation.jsonへ記録しています。
保存元CSV/rosbagは変更していません。誤差CSVも元の整数stamp_nsを保持します。
位置誤差=整合した推定位置−真値位置（真値世界XYZ軸、m）。
姿勢誤差=真値姿勢の逆×整合した推定姿勢、をxyz順のEuler角へ変換（真値機体軸、deg）。
Euler角の±180度折り返しをまたぐ線だけ描画時に切っています。

縦軸は軸ごとに全データが収まる範囲へ調整。軸の省略・分割は行いません。
表示範囲はevaluation.jsonへ記載。全軸の省略点数は0です。
統計やCSVの値をクリップ・間引きしていません。
run1_middleの末尾VIO2時刻は元の後処理結果から未出力です。補作していません。
''')
 with (OUT/'README.txt').open('a') as f:
  f.write('\n入力データ: '+str(SOURCE)+'\nVIO設定: '+str(provenance['variant'])+'\n')
  if provenance['settings']:f.write(provenance['settings']+'\n新VIOはCSVで保存。以前のresults内MCAPは旧設定の結果として保持し、書き換えていません。\n')
if __name__=='__main__':
 candidate=ROOT/'logs/zab-offline/vio_recomputed'
 if (candidate/'manifest.json').exists() and json.loads((candidate/'manifest.json').read_text()).get('status')=='complete':SOURCE=candidate
 parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,default=SOURCE);parser.add_argument('--output',type=Path,default=OUT)
 args=parser.parse_args();SOURCE=args.source;OUT=args.output;main()
