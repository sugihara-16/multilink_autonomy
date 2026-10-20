#!/usr/bin/env python3
"""Full tracking error comparison, fixed three-camera ROVIO, DRPM on/off."""
import json,re
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from run_three_camera_no_drpm import OUT,REF,LABELS,base

def main():
 out=OUT/'plots';out.mkdir(parents=True,exist_ok=True);results={};rows=[];images=[]
 with PdfPages(out/'comparison.pdf') as pdf:
  for label in LABELS:
   d=OUT/'fusion'/label;m=json.loads((d/'metrics.json').read_text());results[label]=m
   summary=json.loads((d/'summary.json').read_text());assert summary['returncode']==0
   for file,h in summary['artifacts'].items():assert base.sha(file)==h
   assert json.loads((OUT/'inputs'/label/'bag_match.json').read_text())['byte_identical']
   assert base.sha(OUT/'rovio'/label/'result_vio.csv')==base.sha(REF/'rovio'/label/'result_vio.csv')
   assert (d/'truth_tracking.csv').read_bytes()==(REF/'fusion'/label/'truth_tracking.csv').read_bytes()
   fig,axs=plt.subplots(6,1,figsize=(13,15),sharex=True)
   for path,name,color in [(d/'off_full_errors.csv','MIMOSA + ROVIO: DRPM OFF','#0072B2'),(REF/'fusion'/label/'drpm_full_errors.csv','MIMOSA + ROVIO: DRPM ON','#D55E00'),(REF/'fusion'/label/'rovio_full_errors.csv','ROVIO alone (3 cameras)','#009E73')]:
    a=np.genfromtxt(path,delimiter=',',names=True)
    assert int(a['evaluation_stamp_ns'][0])==m['alignment_stamp_ns'] and int(a['evaluation_stamp_ns'][-1])==m['full_grid_end_ns']
    for j,ax in enumerate(axs):ax.plot((a['evaluation_stamp_ns']-m['alignment_stamp_ns'])*1e-9,a[a.dtype.names[j+1]],color=color,label=name,lw=1)
   for ax,name in zip(axs,['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']):ax.set_ylabel(name);ax.grid(alpha=.3)
   axs[0].legend();axs[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | same three-camera ROVIO, DRPM ON vs OFF');fig.tight_layout();fig.savefig(out/f'{label}.png',dpi=115);pdf.savefig(fig);plt.close(fig);images.append(f'<h2>{label}</h2><img src="{label}.png">')
   for key in ['off','on','rovio']:
    v=m[key];rows.append(f'<tr><td>{label}</td><td>{key}</td><td>{v["position_rmse_m"]:.4f}</td><td>{v["rotation_rmse_deg"]:.4f}</td><td>{v["position_max_m"]:.4f}</td></tr>')
 note='同じ全追従区間、link1/link3/link4の3カメラ。前回のROVIO出力をそのまま使用。IMU・LiDAR・ROVIOをまとめた入力bagのSHA256はON測定時と一致。推定器バイナリと融合共分散は同じで、変更はDRPM有効フラグとログ保存先のみ。MIMOSA既存の退化投影処理は両方OFF。開始時だけ位置姿勢を整合、元整数nsの共通10Hzで評価。途中再整合・スケール変更・外挿・縦軸省略なし。位置RMSEは3次元、姿勢RMSEは相対回転角。ROVIO単体も参考表示。採用DRPMは独立点群位置合わせの結果を姿勢制約として融合し、OFFは通常MIMOSAの点と面の制約を使う。したがって通常MIMOSA対DRPM導入版の比較であり、同じ解法の確率抑制だけを外した比較ではない。各条件1試行。'
 (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto}img{max-width:100%}td,th{padding:8px;border:1px solid #bbb}table{border-collapse:collapse}</style><h1>3カメラ MIMOSA＋ROVIO：DRPM ON/OFF</h1><p>'+note+'</p><a href="comparison.pdf">PDF</a><table><tr><th>run</th><th>方式</th><th>位置RMSE[m]</th><th>姿勢RMSE[deg]</th><th>最大位置誤差[m]</th></tr>'+''.join(rows)+'</table>'+''.join(images))
 for file in re.findall(r'(?:href|src)="([^"]+)"',(out/'index.html').read_text()):assert (out/file).is_file()
 data=(out/'comparison.pdf').read_bytes();assert len(re.findall(rb'/Type\s*/Page\b',data))==3 and data.rstrip().endswith(b'%%EOF')
 base.save(OUT/'comparison.json',results);base.save(OUT/'validation.json',dict(status='complete',checks={k:v['checks'] for k,v in results.items()},pdf_pages=3,cleanup_complete=False))
 for label,m in results.items():print(label,{k:(m[k]['position_rmse_m'],m[k]['rotation_rmse_deg'],m[k]['position_max_m']) for k in ['off','on','rovio']})
if __name__=='__main__':main()
