"""Compact installation/flight report, including the actual rendered end views."""
import argparse,html,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation,Slerp
from report_articulated_dragon import read_csv


def report(root):
    out=root/'report';out.mkdir(exist_ok=True)
    run=root/'active90';validation=json.loads((run/'validation.json').read_text())
    inspection=root/'inspect20_v2'/'inspection'
    images=[];max_angle_error=0.
    for p in sorted(inspection.glob('??_*.png')):
        meta=json.loads((inspection/(p.name[:2]+'.json')).read_text())
        max_angle_error=max(max_angle_error,float(np.max(abs(np.array(meta['command_deg'])-meta['measured_deg']))))
        images.append(f'<figure><img src="../{p.relative_to(root)}"><figcaption>{html.escape(p.stem)}</figcaption></figure>')
    audit=json.loads((run/'occlusion_audit.json').read_text())
    truth=read_csv(run/'truth.csv');truth[:,0]*=1e-9
    truth=truth[np.r_[True,np.diff(truth[:,0])>0]]
    start,end=validation['evaluation_interval_s']
    fig,ax=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for name in ('rovio','mimosa'):
        a=read_csv(run/(name+'.csv'));a[:,0]*=1e-9;a=a[(a[:,0]>=start)&(a[:,0]<=end)]
        pos=np.column_stack([np.interp(a[:,0],truth[:,0],truth[:,i]) for i in (1,2,3)])
        rot=Slerp(truth[:,0],Rotation.from_quat(truth[:,4:8]))([a[0,0]])[0]*Rotation.from_quat(a[0,4:8]).inv()
        error=rot.apply(a[:,1:4]-a[0,1:4])+pos[0]-pos
        for i in range(3):ax[i].plot(a[:,0]-start,error[:,i],label=name.upper());ax[i].set_ylabel('XYZ'[i]+' error [m]');ax[i].grid()
    ax[0].legend();ax[-1].set_xlabel('Tracking time [s]');fig.tight_layout();fig.savefig(out/'position_error.png',dpi=150);plt.close(fig)
    a=read_csv(run/'gimbal.csv');a=a[(a[:,0]>=start)&(a[:,0]<=end)]
    fig,ax=plt.subplots(figsize=(10,3))
    ax.plot(a[:,0]-start,np.rad2deg(a[:,1]),label='Yaw');ax.plot(a[:,0]-start,np.rad2deg(a[:,2]),label='Pitch')
    ax.set(xlabel='Tracking time [s]',ylabel='Measured angle [deg]');ax.grid();ax.legend();fig.tight_layout();fig.savefig(out/'gimbal.png',dpi=150);plt.close(fig)
    (out/'summary.json').write_text(json.dumps(dict(validation=validation,occlusion=audit,inspection_images=len(images),max_inspection_angle_error_deg=max_angle_error),indent=2))
    rows=''.join(f'<tr><td>{name.upper()}</td><td>{v["position_rmse_m"]:.3f} m</td><td>{v["rotation_rmse_deg"]:.3f}°</td></tr>' for name,v in validation['estimators'].items())
    text=f'''<!doctype html><meta charset="utf-8"><title>DRAGON gimbal validation</title>
<style>body{{font:16px sans-serif;max-width:1150px;margin:35px auto;line-height:1.6}}table{{border-collapse:collapse}}td,th{{padding:8px 18px;border:1px solid #ccc}}img{{max-width:100%}}.views{{display:grid;grid-template-columns:repeat(3,1fr)}}figure{{margin:8px}}figcaption{{font-size:13px}}</style>
<h1>DRAGON：固定四角形＋link4ジンバル</h1>
<p><strong>配置・自己遮蔽・飛行動作は確認済み。90mの精度検証はROVIOが発散し、融合後も大きな誤差が残った。</strong></p>
<p>LiDAR＋IMUをlink1、単眼カメラをlink4。センサ設定・ROVIO＋MIMOSAは現行構成を継承、DRPM OFF。
カメラ中心はlink4座標(0.10, −0.60, 0)m。元の配置から外側へ52cm移し、高さと初期方向を維持。
yaw±50°、pitch±90°、正のpitchは下向き。支持棒の重量・たわみはモデル化していない理想ジンバル。</p>
<h2>自己遮蔽の確認</h2><p>機体visualの囲い＋15mm、ローター可動範囲を含む境界、画角余裕3°で{audit['checked_views']}姿勢を検査。
視野との重なり{audit['possible_occlusion_views']}件。下の画像はホバリング中に取得した実際のGazebo画像で、マスクやロボット非表示なし。
角度指令と実測の最大差{max_angle_error:.3f}°。この検査は固定四角形の配置についてのもの。</p><div class="views">{''.join(images)}</div>
<h2>90m飛行・1試行</h2><p>weathered24301、0.5m/s。離陸・初期化・40秒のカメラyaw観測を除外し、追従開始姿勢だけを合わせて評価。尺度補正なし。
実移動{validation['active_travel_m']:.3f}m、LiDAR搭載リンクの姿勢変動最大{validation['lidar_carrier_attitude_excursion_deg']:.3f}°。</p>
<table><tr><th>推定器</th><th>位置RMSE</th><th>回転RMSE</th></tr>{rows}</table>
<p>準備運動終了時のROVIO位置誤差は約0.17mだったが、40秒のyaw観測後には約4.75m、特徴点距離の中央値は約844mになった。
飛行開始前から推定が不安定であり、この試行を正常に初期化できたジンバル方式の性能とは扱わない。自己遮蔽の解消と推定器の安定性は別の結果。</p>
<p>カメラの取り付け位置と初期観測が関節偏向実験と異なるため、過去の関節偏向結果との差を駆動機構だけの効果とは扱わない。</p>
<img src="position_error.png"><img src="gimbal.png">
<h2>起動</h2><pre>ros2 launch multilink_uas_bringup dragon_gimbal.launch.py</pre>
<p>Gazebo GUIあり。90m飛行・着陸後に自動終了。途中はCtrl+C。取り付け確認は inspect:=true distance:=0。</p>'''
    (out/'index.html').write_text(text)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();report(a.root)
