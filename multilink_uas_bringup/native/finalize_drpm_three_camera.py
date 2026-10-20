#!/usr/bin/env python3
"""Compare fixed-weight three-camera DRPM against previous mono and official FAST."""
import csv,json,re,subprocess
from pathlib import Path
import numpy as np,yaml
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from run_drpm_three_camera import OUT,OLD,LABELS,ROOT,BASE,base

def main():
 out=OUT/'plots';out.mkdir(parents=True,exist_ok=True);result={};checks={};images=[];rows=[]
 with PdfPages(out/'comparison.pdf') as pdf:
  for label in LABELS:
   d=OUT/'fusion'/label;m=json.loads((d/'metrics.json').read_text());old=json.loads((OLD/'fusion'/label/'metrics.json').read_text());vio=json.loads((OUT/'rovio'/label/'summary.json').read_text())
   assert m['window']==old['window'] and m['alignment_stamp_ns']==old['alignment_stamp_ns'] and m['full_grid_end_ns']==old['full_grid_end_ns']
   assert m['checks']['estimator_artifacts']==old['checks']['estimator_artifacts']
   newcfg=yaml.safe_load((d/'config.yaml').read_text());oldcfg=yaml.safe_load((OLD/'fusion'/label/'config.yaml').read_text());newcfg['logs_directory']=oldcfg['logs_directory'];assert newcfg==oldcfg
   assert (d/'truth_tracking.csv').read_bytes()==(OLD/'fusion'/label/'truth_tracking.csv').read_bytes()
   assert vio['all_camera_groups_complete'] and vio['only_camera_blocks_differ_from_mono'] and all(x>0 for x in vio['feature_observations'].values())
   audit=json.loads((OUT/'inputs'/label/'input_audit.json').read_text());bag=json.loads((OUT/'inputs'/label/'bag_summary.json').read_text());assert bag['counts']['/comparison/imu']==audit['streams']['imu_t']['count'];assert bag['counts']['/comparison/points']==audit['streams']['cloud_t']['count']
   filechecks={}
   for name in ['odometry_tracking.csv','rovio_tracking.csv','truth_tracking.csv']:
    p=d/name;ts=[int(r['stamp_ns']) for r in csv.DictReader(p.open())];assert (np.diff(ts)>0).all();filechecks[name]=dict(samples=len(ts),sha256=base.sha(p))
   checks[label]=dict(same_entire_tracking_interval=True,same_truth=True,same_drpm_binary_and_configuration=True,same_fusion_covariance=True,all_three_cameras_used=True,feature_observations=vio['feature_observations'],files=filechecks)
   result[label]=dict(full_mono=old['full'],full_three=m['full'],common_mono=old['common'],common_three=m['common'],registration=m['registration'])
   for mode in ['full','common','three_camera_detail']:
    if mode=='three_camera_detail':
     lines=[(d/'drpm_full_errors.csv','DRPM + ROVIO: 3 cameras','#D55E00'),(d/'rovio_full_errors.csv','ROVIO alone: 3 cameras','#009E73')]
    else:
     lines=[(OLD/'fusion'/label/f'drpm_{mode}_errors.csv','DRPM + ROVIO: link3','#0072B2'),(d/f'drpm_{mode}_errors.csv','DRPM + ROVIO: link1/link3/link4','#D55E00')]
    if mode=='common':lines.append((d/'fast_official_common_errors.csv','Official FAST-LIVO2: link3','#777777'))
    fig,axs=plt.subplots(6,1,figsize=(13,15),sharex=True)
    for path,name,color in lines:
     a=np.genfromtxt(path,delimiter=',',names=True)
     for j,ax in enumerate(axs):ax.plot((a['evaluation_stamp_ns']-m['alignment_stamp_ns'])*1e-9,a[a.dtype.names[j+1]],label=name,color=color,lw=1)
    for ax,name in zip(axs,['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']):ax.set_ylabel(name);ax.grid(alpha=.3)
    axs[0].legend();axs[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | '+{'full':'Full tracking: 1 vs 3 cameras','common':'Common interval with official FAST-LIVO2','three_camera_detail':'Full tracking: three-camera results in detail'}[mode]);fig.tight_layout();fig.savefig(out/f'{label}_{mode}.png',dpi=115);pdf.savefig(fig);plt.close(fig);images.append(f'<h2>{label}: {mode}</h2><img src="{label}_{mode}.png">')
   for name,data in [('DRPM + mono',old['full']['drpm']),('DRPM + 3 cameras',m['full']['drpm']),('ROVIO mono',old['full']['rovio']),('ROVIO 3 cameras',m['full']['rovio'])]:
    rows.append(f'<tr><td>{label}</td><td>{name}</td><td>{data["position_rmse_m"]:.4f}</td><td>{data["rotation_rmse_deg"]:.4f}</td><td>{data["position_max_m"]:.4f}</td><td>{data["first_sustained_100m_s"]}</td></tr>')
 note='前回と同じ3飛行・全追従区間。3台はlink1/link3/link4、単眼はlink3。DRPM公式更新＋距離方向ノイズ、融合の固定共分散、ROVIOの設定と合計特徴点上限25を維持。画像は各カメラの既存校正で白黒480×360へ補正。単眼用に過去データで校正した融合重みをそのまま使う比較であり、3台向けに再校正した最適性能ではない。離陸前から初期化し、評価は追従区間のみ。開始時に一度だけ位置姿勢を整合。時刻は元の整数ns、評価は共通10Hz。途中再整合、スケール調整、外挿、縦軸省略なし。位置RMSEは3次元、姿勢RMSEは相対回転角。100m超の位置誤差が5秒継続する区間の開始時刻を発散時刻として記載。FASTとの比較は前回FAST出力がある短い共通区間のみ。各runに3カメラ構成の融合結果とROVIO単体だけを描く詳細図も掲載し、旧構成の大誤差に新結果が埋もれないようにした。どの図も全データを描き、縦軸は図ごとに自動設定する。'
 (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto}img{max-width:100%}td,th{padding:8px;border:1px solid #bbb}table{border-collapse:collapse}</style><h1>DRPM＋vision：カメラ1台対3台</h1><p>'+note+'</p><a href="comparison.pdf">PDF</a><table><tr><th>run</th><th>方式（全追従区間）</th><th>位置RMSE[m]</th><th>姿勢RMSE[deg]</th><th>最大位置誤差[m]</th><th>発散時刻[s]</th></tr>'+''.join(rows)+'</table>'+''.join(images))
 for name in re.findall(r'(?:href|src)="([^"]+)"',(out/'index.html').read_text()):assert (out/name).exists()
 pdf=(out/'comparison.pdf').read_bytes();assert len(re.findall(rb'/Type\s*/Page\b',pdf))==9 and pdf.rstrip().endswith(b'%%EOF')
 official=subprocess.check_output(['git','-C',str(base.WS/'drpm'),'show','b39a540c946f4059f5e3dfde625216c437eaf158:src/degeneracy.h']);assert official==(base.WS/'src/mimosa/include/mimosa/lidar/official_drpm/degeneracy.h').read_bytes()
 base.save(OUT/'comparison.json',result);base.save(OUT/'validation.json',dict(status='complete',checks=checks,official_drpm_header_identical=True,pdf_pages=9,cleanup_complete=False))
 for label,x in result.items():
  a=x['full_mono']['drpm'];b=x['full_three']['drpm'];v=x['full_three']['rovio'];print(label,'MONO',a['position_rmse_m'],'THREE',b['position_rmse_m'],b['rotation_rmse_deg'],'MAX',b['position_max_m'],'ONSET',b['first_sustained_100m_s'],'VIO',v['position_rmse_m'],flush=True)
if __name__=='__main__':main()
