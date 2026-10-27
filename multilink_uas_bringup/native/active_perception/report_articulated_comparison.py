"""One-run DRAGON fixed/active comparison, no scale fit or selected intervals."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_articulated_dragon import read_csv


def series(path,name):
    events={e['stage']:e['time'] for e in json.loads((path/'flight_events.json').read_text())}
    protocol=json.loads((path/'protocol.json').read_text());start=events['active'];end=min(events['landing'],start+protocol['distance_m']/protocol['speed_mps']+2)
    truth=read_csv(path/'truth.csv');truth[:,0]*=1e-9;truth=truth[np.r_[True,np.diff(truth[:,0])>0]]
    a=read_csv(path/(name+'.csv'));a[:,0]*=1e-9;a=a[np.r_[True,np.diff(a[:,0])>0]]
    t=np.arange(start+.1,end-.01,.1);valid=(t>=a[0,0])&(t<=a[-1,0])&(t>=truth[0,0])&(t<=truth[-1,0]);t=t[valid]
    pos=np.column_stack([np.interp(t,truth[:,0],truth[:,i]) for i in (1,2,3)]);rot=Slerp(truth[:,0],Rotation.from_quat(truth[:,4:8]))(t)
    p=np.column_stack([np.interp(t,a[:,0],a[:,i]) for i in (1,2,3)]);r=Slerp(a[:,0],Rotation.from_quat(a[:,4:8]))(t)
    align=rot[0]*r[0].inv();p=align.apply(p-p[0])+pos[0];r=align*r
    error=p-pos;angular=(rot.inv()*r).as_rotvec()*180/np.pi
    m=(t-start>=10)&(t-start<=170);x=error[m,0];local=x[100:]-x[:-100] if len(x)>100 else np.array([])
    return t-start,error,angular,dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
        position_end_m=float(np.linalg.norm(error[-1])),rotation_rmse_deg=float(np.sqrt(np.mean(np.sum(angular**2,axis=1)))),
        axial_10s_displacement_rmse_m=float(np.sqrt(np.mean(local**2))) if len(local) else None,samples=len(t),
        coverage_s=float(t[-1]-t[0]),max_output_gap_s=float(np.diff(a[(a[:,0]>=start)&(a[:,0]<=end),0]).max()))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('--active',default='active90');parser.add_argument('--fixed',default='fixed90');a=parser.parse_args()
    out=a.root/'report';out.mkdir(exist_ok=True);results={};data={}
    for mode,name in [('active',a.active),('fixed',a.fixed)]:
        folder=a.root/name
        if not (folder/'flight_result.json').exists():continue
        results[mode]={'path':str(folder),'estimators':{},'integration':json.loads((folder/'validation.json').read_text())}
        for estimator in ['rovio','mimosa']:
            t,e,r,metrics=series(folder,estimator);data[mode,estimator]=(t,e,r);results[mode]['estimators'][estimator]=metrics
    with PdfPages(out/'comparison.pdf') as pdf:
        for estimator in ['rovio','mimosa']:
            fig,axs=plt.subplots(3,2,figsize=(13,9),sharex=True)
            for mode in results:
                t,e,r=data[mode,estimator]
                for i in range(3):axs[i,0].plot(t,e[:,i],label=mode);axs[i,1].plot(t,r[:,i],label=mode)
            for i in range(3):
                axs[i,0].set_ylabel('XYZ'[i]+' error [m]');axs[i,1].set_ylabel(['Roll','Pitch','Yaw'][i]+' rotvec error [deg]')
            for ax in axs.flat:ax.grid(alpha=.3)
            axs[0,0].legend();axs[-1,0].set_xlabel('Tracking time [s]');axs[-1,1].set_xlabel('Tracking time [s]')
            fig.suptitle(estimator.upper()+' — DRAGON, layout 24301, 90 m / 0.5 m/s');fig.tight_layout();pdf.savefig(fig);fig.savefig(out/(estimator+'.png'),dpi=140);plt.close(fig)
        fig,axs=plt.subplots(3,1,figsize=(13,10),sharex=True)
        for mode,v in results.items():
            folder=Path(v['path']);f=read_csv(folder/'flight.csv');events={e['stage']:e['time'] for e in json.loads((folder/'flight_events.json').read_text())};m=(f[:,0]>=events['active'])&(f[:,0]<=events['active']+182)
            for i in range(6):axs[i%2].plot(f[m,0]-events['active'],np.rad2deg(f[m,4+i]),label=mode+' joint'+str(i//2+1))
            import sys
            sys.path.insert(0,str(Path(__file__).resolve().parent.parent));from dragon_articulated import Kinematics
            k=Kinematics(folder/'dragon_slam.urdf');qs=f[m,4:][::5];opt=np.array([k.camera(q)[:3,2] for q in qs]);az=np.unwrap(np.arctan2(opt[:,1],opt[:,0]));el=np.arcsin(np.clip(opt[:,2],-1,1))
            axs[2].plot((f[m,0]-events['active'])[::5],np.rad2deg(az),label=mode+' camera azimuth');axs[2].plot((f[m,0]-events['active'])[::5],np.rad2deg(el),label=mode+' camera elevation')
        for ax,title in zip(axs,['Body joint pitch','Body joint yaw','Camera optical axis relative to LiDAR/IMU link']):ax.set_ylabel('Angle [deg]');ax.set_title(title);ax.grid(alpha=.3);ax.legend(ncol=3)
        axs[-1].set_xlabel('Tracking time [s]');fig.tight_layout();pdf.savefig(fig);fig.savefig(out/'joints.png',dpi=140);plt.close(fig)
    historical=Path('/root/uas_native/logs/information-holdout-20261026/report/summary.json')
    gimbal=json.loads(historical.read_text())['trials']['layout24301_lidar_conditioned_seed20262801']['raw']['full_fused']['active']
    report=dict(results=results,gimbal_historical_reference=gimbal,reference_path=str(historical),
                interpretation='Single independent DRAGON flight per arm. Historical UniPilot gimbal is context, not a controlled platform-only ablation. Different camera mount, carrier dynamics, preparation and planning score. Sensor physical settings and estimator/fusion settings inherited.',
                alignment='First tracking pose only, no scale fit, full 90m trajectory. Ten-second axial displacement errors use cruising 10..170s.')
    (out/'summary.json').write_text(json.dumps(report,indent=2))
    html='''<!doctype html><meta charset="utf-8"><title>DRAGON coordinated view search</title><style>body{font:16px sans-serif;max-width:1250px;margin:30px auto;line-height:1.7}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px}img{max-width:100%}</style><h1>DRAGON：複数関節による視野選択</h1><p>人工トンネル24301の90m区間、0.5m/s。単眼ROVIO＋MIMOSA、DRPM OFF。固定・偏向とも同じ初期観測の後に評価。各1飛行であり、反復平均ではない。</p><p>関節の可動範囲から複数関節の候補と経路を作成し、観測済み特徴地図と現在LiDAR情報を使って視野を選ぶ。初期の約180°観測動作は固定・偏向に共通で、比較対象から除外する。地図の特徴点独立近似による採点で、ジンバルv10の相関付き局所速度採点とは同一実装ではない。真値は飛行制御と事後評価だけ。</p><p><a href="comparison.pdf">PDF</a> / <a href="summary.json">数値</a></p><table><tr><th>方式</th><th>推定器</th><th>位置RMSE [m]</th><th>終端誤差 [m]</th><th>回転RMSE [deg]</th><th>10秒X移動誤差 [m]</th></tr>'''
    for mode,v in results.items():
        for estimator,m in v['estimators'].items():html+=f'<tr><td>{mode}</td><td>{estimator}</td>'+''.join(f'<td>{m[k]:.3f}</td>' for k in ['position_rmse_m','position_end_m','rotation_rmse_deg','axial_10s_displacement_rmse_m'])+'</tr>'
    html+='</table><p>参考：既存UniPilotジンバルの同じ配置・90mの最初の試行ではMIMOSA位置RMSE '+f'{gimbal["mimosa"]["position_rmse_m"]:.3f}m。機体、カメラ取り付け、初期観測と視野探索方式が異なるため、関節機構だけの優劣を表す比較ではない。</p>'
    if 'active' in results:
        integ=results['active']['integration']
        html+=f"<p>追従中の実動作：関節変化幅は最大{max(integ['joint_span_deg']):.2f}°、カメラ光軸の向きは初期から最大{integ.get('camera_optical_axis_excursion_deg',float('nan')):.2f}°。画像の回転も含むカメラ姿勢全体の変化は{integ['camera_rotation_excursion_deg']:.2f}°。LiDAR側の実姿勢は最大{integ['lidar_carrier_attitude_excursion_deg']:.2f}°変動した。</p>"
    audit_path=out/'final_audit.json'
    if audit_path.exists():
        audit=json.loads(audit_path.read_text());counts=audit['lidar_information_state_counts'];dt=audit['planning_intervals_s']
        html+=f"<p>計画判断の内訳：LiDAR情報が新鮮{counts.get('current',0)}回、情報不足で視覚による観測継続{counts.get('unavailable_visual_recovery',0)}回。常時LiDARの退化方向が分かったという意味ではない。計画更新間隔は中央値{dt['median']:.2f}秒、最大{dt['max']:.2f}秒。候補/経路評価の計算が律速で、公称タイマー周期とは区別する。</p><p><a href='final_audit.json'>取得・設定・終了の確認結果</a></p>"
    for img in ['mimosa','rovio','joints']:html+=f'<img src="{img}.png">'
    (out/'index.html').write_text(html);print(json.dumps(report,indent=2))

if __name__=='__main__':main()
