#!/usr/bin/env python3
"""Full tracking comparison of native translation projection and untreated MIMOSA."""
import json,re
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from run_corrected_mono_native_degen import OUT,REF,CONTROL,base
LABELS=base.LABELS

def main():
 out=OUT/'plots';out.mkdir(parents=True,exist_ok=True);results={};rows=[];images=[]
 with PdfPages(out/'comparison.pdf') as pdf:
  for label in LABELS:
   d=OUT/'fusion'/label;m=json.loads((d/'metrics.json').read_text());results[label]=m
   summary=json.loads((d/'summary.json').read_text());assert summary['returncode']==0
   for file,h in summary['artifacts'].items():assert base.sha(file)==h
   assert json.loads((OUT/'inputs'/label/'bag_match.json').read_text())['byte_identical']
   assert json.loads((OUT/'inputs'/label/'bag_match.json').read_text())['sha256']==json.loads((CONTROL/'fusion'/label/'summary.json').read_text())['input_sha256']
   assert base.sha(OUT/'rovio'/label/'result_vio.csv')==base.sha(REF/'rovio'/label/'result_vio.csv')
   assert (d/'truth_tracking.csv').read_bytes()==(REF/'fusion'/label/'truth_tracking.csv').read_bytes()
   fig,axs=plt.subplots(6,1,figsize=(13,15),sharex=True)
   for path,name,color in [(d/'native_full_errors.csv','Native translation handling ON','#0072B2'),(CONTROL/'fusion'/label/'off_full_errors.csv','No degeneracy handling','#D55E00'),(REF/'fusion'/label/'drpm_full_errors.csv','Adopted DRPM','#CC79A7'),(REF/'fusion'/label/'rovio_full_errors.csv','Corrected ROVIO alone (link3)','#009E73')]:
    a=np.genfromtxt(path,delimiter=',',names=True)
    assert int(a['evaluation_stamp_ns'][0])==m['alignment_stamp_ns'] and int(a['evaluation_stamp_ns'][-1])==m['full_grid_end_ns']
    for j,ax in enumerate(axs):ax.plot((a['evaluation_stamp_ns']-m['alignment_stamp_ns'])*1e-9,a[a.dtype.names[j+1]],color=color,label=name,lw=1,linestyle='--' if name=='No degeneracy handling' else '-')
   for ax,name in zip(axs,['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']):ax.set_ylabel(name);ax.grid(alpha=.3)
   axs[0].legend();axs[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | single-camera ROVIO: native translation handling');fig.tight_layout();fig.savefig(out/f'{label}.png',dpi=115);pdf.savefig(fig);plt.close(fig);images.append(f'<h2>{label}</h2><p>Native handling calls: {m["native_handling_linearization_count"]}; output identical to untreated: {m["output_identical_to_untreated"]}</p><img src="{label}.png">')
   for key in ['native','untreated','drpm','rovio']:
    v=m[key];rows.append(f'<tr><td>{label}</td><td>{key}</td><td>{v["position_rmse_m"]:.4f}</td><td>{v["rotation_rmse_deg"]:.4f}</td><td>{v["position_max_m"]:.4f}</td></tr>')
 note='同じ全追従区間、link3の単眼。初期加速度バイアス分散をセンサ仕様から修正した前回のROVIO出力をそのまま使用。IMU・LiDAR・ROVIOをまとめた入力bagのSHA256は前回の標準処理OFF・DRPM ONの両測定と一致。推定器バイナリと融合共分散は同じで、比較元はDRPM OFF・標準退化処理OFF。変更は標準退化処理の有効フラグとログ保存先のみ。並進しきい値40、回転しきい値0を維持し、現在の実装が適用する並進側の処理のみを有効化。開始時だけ位置姿勢を整合、元整数nsの共通10Hzで評価。途中再整合・スケール変更・外挿・縦軸省略なし。位置RMSEは3次元、姿勢RMSEは相対回転角。ROVIO単体も参考表示。今回と標準処理OFFは同じMIMOSAの点と面の制約を使う。DRPMは過去の参考結果で、独立点群位置合わせによる姿勢制約を使う別方式。退化処理回数は内部計算の呼出し回数であり、時刻やスキャン数ではない。標準処理ONとOFFが一致する場合、青実線と橙破線は重なっている。各条件1試行。'
 (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto}img{max-width:100%}td,th{padding:8px;border:1px solid #bbb}table{border-collapse:collapse}</style><h1>単眼 MIMOSA＋ROVIO：標準の並進退化処理ON</h1><p>'+note+'</p><a href="comparison.pdf">PDF</a><table><tr><th>run</th><th>方式</th><th>位置RMSE[m]</th><th>姿勢RMSE[deg]</th><th>最大位置誤差[m]</th></tr>'+''.join(rows)+'</table>'+''.join(images))
 for file in re.findall(r'(?:href|src)="([^"]+)"',(out/'index.html').read_text()):assert (out/file).is_file()
 assert json.loads((OUT/'cleanup.json').read_text())['owned_masters_stopped']
 assert not list((OUT/'inputs').glob('*.bag')) and not list((OUT/'inputs').glob('*/*.npz'))
 data=(out/'comparison.pdf').read_bytes();assert len(re.findall(rb'/Type\s*/Page\b',data))==3 and data.rstrip().endswith(b'%%EOF')
 base.save(OUT/'comparison.json',results);base.save(OUT/'validation.json',dict(status='complete',checks={k:v['checks'] for k,v in results.items()},pdf_pages=3,cleanup_complete=True))
 base.save(OUT/'runtime.json',dict(status='complete',plots=str(out/'index.html')))
 for label,m in results.items():print(label,{k:(m[k]['position_rmse_m'],m[k]['rotation_rmse_deg'],m[k]['position_max_m']) for k in ['native','untreated','drpm','rovio']})
if __name__=='__main__':main()
