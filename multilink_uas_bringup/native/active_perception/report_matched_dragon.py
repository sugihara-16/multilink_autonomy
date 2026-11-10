"""Read-only, identical first-pose SE3 scoring for historical and new flights."""
import argparse,csv,gzip,hashlib,json,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import yaml
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
NATIVE=HERE.parent

def table(path):
    if not path.exists():path=Path(str(path)+'.gz')
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt') as f:return np.genfromtxt(f,delimiter=',',names=True)

def poses(path):
    a=table(path);t=a['stamp_ns']*1e-9
    m=np.r_[True,np.diff(t)>0]
    return t[m],np.column_stack([a[k] for k in ('x','y','z')])[m],np.column_stack([a[k] for k in ('qx','qy','qz','qw')])[m]

def errors(path,method,start,end):
    tt,pt,qt=poses(path/'truth.csv');t,p,q=poses(path/(method+'.csv'))
    m=(t>=max(start,tt[0]))&(t<=min(end,tt[-1]));t,p,q=t[m],p[m],q[m]
    truth=np.column_stack([np.interp(t,tt,pt[:,i]) for i in range(3)])
    rt=Slerp(tt,Rotation.from_quat(qt))(t);r=rt[0]*Rotation.from_quat(q[0]).inv()
    err=r.apply(p-p[0])+truth[0]-truth
    angles=(rt.inv()*r*Rotation.from_quat(q)).as_rotvec()*180/np.pi
    return dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(err**2,axis=1)))),
                position_max_m=float(np.linalg.norm(err,axis=1).max()),
                rotation_rmse_deg=float(np.sqrt(np.mean(np.sum(angles**2,axis=1)))),
                estimated_displacement_m=float(np.linalg.norm(p[-1]-p[0])),
                true_displacement_m=float(np.linalg.norm(truth[-1]-truth[0])),
                samples=len(t),max_gap_s=float(np.diff(t).max())),(t-start,err)

def flight(path,kind):
    if kind=='unipilot':
        a=table(path/'route_tracking.csv');i=np.flatnonzero(a['route_t']>0)[0];start=float(a['sim_t'][i]-a['route_t'][i]);end=start+182
        d=table(path/'vio_diagnostic.csv')
    else:
        ev={x['stage']:x['time'] for x in json.loads((path/'flight_events.json').read_text())}
        start=ev['active'];end=min(ev['landing'],start+182);d=table(path/'vio.csv')
    rows={};curves={}
    for method in ('rovio','mimosa'):rows[method],curves[method]=errors(path,method,start,end)
    m=(d['stamp_ns']*1e-9>=start-5)&(d['stamp_ns']*1e-9<=start)
    rows.update(start_s=start,end_s=end,path=str(path),kind=kind,
                preflight_features_median=float(np.median(d['features'][m])),
                preflight_depth_median_m=float(np.median(d['median_depth'][m])))
    factors=table(path/'mimosa/velocity_factors.csv')
    m=(factors['stamp_ns']*1e-9>=start+10)&(factors['stamp_ns']*1e-9<=end-10)
    rows['accepted_visual_velocity']=dict(count=int(m.sum()),interval_s=[start+10,end-10],
        median_body_x_mps=float(np.median(factors['vx'][m])),median_logged_x_variance=float(np.median(factors['c00'][m])))
    return rows,curves

def audit(path,ref):
    a=yaml.safe_load((path/'mimosa.yaml').read_text());b=yaml.safe_load((ref/'mimosa.yaml').read_text())
    a.pop('logs_directory');b.pop('logs_directory')
    lidar_transform=a['lidar'].pop('T_B_S');b['lidar'].pop('T_B_S')
    pose=np.fromstring(ET.parse(path/'dragon_slam.urdf').getroot().findtext(".//sensor[@name='end_lidar']/pose"),sep=' ')
    actual_transform=np.r_[pose[:3],Rotation.from_euler('xyz',pose[3:]).as_quat()]
    def without_camera(text):return re.sub(r'\nCamera0\s*\{.*?\}', '',text,flags=re.S)
    sha=json.loads((path/'protocol.json').read_text())['sha256']
    ref_sha=json.loads((ref/'protocol.json').read_text())['binary_sha256']
    binary=next(v for k,v in sha.items() if k.endswith('/rovio-joint-geometry/rovio_gimbal'))
    fusion_binary=next(v for k,v in sha.items() if k.endswith('/mimosa_batched_node'))
    fusion_library=next(v for k,v in sha.items() if k.endswith('/mimosa-batched-vision/libmimosa_lib.so'))
    runtime=yaml.safe_load((path/'rovio_runtime.yaml').read_text())
    ref_runtime=yaml.safe_load((ref/'rovio_runtime.yaml').read_text())
    checks=dict(planner_settings=yaml.safe_load((path/'planner.yaml').read_text())==yaml.safe_load((ref/'planner.yaml').read_text()),
                fusion_settings=a==b,
                actual_lidar_calibration=np.allclose(lidar_transform,actual_transform,atol=1e-12,rtol=0),
                camera_intrinsics=yaml.safe_load((path/'cam0.yaml').read_text())==yaml.safe_load((ref/'cam0.yaml').read_text()),
                rovio_noise_and_filter=without_camera((path/'rovio.info').read_text())==without_camera((ref/'rovio.info').read_text()),
                rovio_imu_model=runtime['rovio_gimbal']==ref_runtime['rovio_gimbal'],
                rovio_binary=binary==ref_sha['rovio_gimbal'],
                fusion_binary=fusion_binary==ref_sha['mimosa_node'],
                fusion_library=fusion_library==ref_sha['libmimosa_lib.so'],
                world_and_scheduling=(path/'world_serial.sdf').read_bytes()==(ref/'world_serial.sdf').read_bytes(),
                no_preflight_sweep='articulation' not in [e['stage'] for e in json.loads((path/'flight_events.json').read_text())],
                body_clearance=json.loads((path/'occlusion_audit.json').read_text())['passed'])
    completion=json.loads((path/'completion.json').read_text())
    checks['all_exit_zero']=all(v==0 for v in completion['processes'].values())
    live={k:pid for k,pid in json.loads((path/'owners.json').read_text()).items() if Path('/proc',str(pid)).exists()}
    checks['owned_processes_stopped']=not live
    return dict(checks=checks,live_pids=live,acquisition=json.loads((path/'validation.json').read_text()),
                remaining_differences=json.loads((path/'matching.json').read_text())['residual_differences'])

def report(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=root/'report';out.mkdir(exist_ok=True)
    reference=NATIVE/'logs/information-holdout-20261026'
    rows={};curves={};audits={}
    old=NATIVE/'logs/dragon-gimbal-20261027/active90'
    rows['Previous DRAGON'],curves['Previous DRAGON']=flight(old,'dragon')
    pilot=root/'pilot90'
    if pilot.exists():rows['Setup pilot'],_=flight(pilot,'dragon')
    preclearance=root/'preclearance90'
    if preclearance.exists():rows['Before LiDAR clearance correction'],_=flight(preclearance,'dragon')
    dropped=root/'incomplete_attempts/transport_missing_imu90'
    if dropped.exists():
        label='Clearance pilot: 2 IMU samples missing (excluded)'
        rows[label],_=flight(dropped,'dragon')
    for p in sorted(root.glob('active_seed*')):
        if not (p/'validation.json').exists():continue
        seed=int(p.name.split('seed')[-1]);key=f'layout24301_lidar_conditioned_seed{seed}'
        ref=reference/key/'campaign'/(key+'_active')
        rows[f'UniPilot {seed}'],curves[f'UniPilot {seed}']=flight(ref,'unipilot')
        label=f'Matched DRAGON {seed}'
        rows[label],curves[label]=flight(p,'dragon');audits[label]=audit(p,ref)
    aggregates={}
    for label,prefix in [('unipilot','UniPilot '),('matched_dragon','Matched DRAGON ')]:
        selected=[r for k,r in rows.items() if k.startswith(prefix)]
        aggregates[label]={m:dict(count=len(selected),mean_position_rmse_m=float(np.mean([r[m]['position_rmse_m'] for r in selected])),
                                  std_position_rmse_m=float(np.std([r[m]['position_rmse_m'] for r in selected],ddof=1)) if len(selected)>1 else None)
                           for m in ('rovio','mimosa')} if selected else {}
    result=dict(results=rows,audits=audits,aggregates=aggregates,
        evaluation='90m at 0.5m/s, weathered layout24301; first tracking pose SE3 alignment, no scale fitting. Historical UniPilot is independently simulated, not identical sensor data.',
        validity='Do not discard failed accuracy results. Incomplete setup attempts kept separately. Operational checks are not accuracy success.')
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for label,cs in curves.items():
        if label=='Previous DRAGON':continue
        for ax,method in zip(axes,('rovio','mimosa')):
            t,e=cs[method];ax.plot(t,np.linalg.norm(e,axis=1),label=label,lw=1)
    for ax,title in zip(axes,('ROVIO','ROVIO + MIMOSA')):
        ax.set(title=title,ylabel='Position error [m]');ax.grid(alpha=.3);ax.legend(fontsize=8)
    axes[-1].set_xlabel('Tracking elapsed [s]');fig.tight_layout();fig.savefig(out/'position_error.png',dpi=150);fig.savefig(out/'comparison.pdf');plt.close(fig)
    html='''<!doctype html><meta charset="utf-8"><title>DRAGON: UniPilot条件との整合</title><style>body{font:16px sans-serif;max-width:1100px;margin:32px auto;line-height:1.7}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}img{max-width:100%}</style><h1>DRAGON＋ジンバル：UniPilotとの条件整合</h1><p>同じ模様24301、90m、0.5m/s。追従開始姿勢だけで座標を合わせ、尺度補正なし。別飛行のため同じseedでも同一センサ列ではない。</p><p>センサ・ROVIO実行バイナリ・融合設定・視野選択処理・準備運動を整合。前向き、2回目の準備中は視線保持、追加旋回なし。カメラ中心は機体遮蔽を避け前方80cm（UniPilotは18cm）。機体・飛行制御・実運動は異なる。</p><table><tr><th>条件</th><th>ROVIO位置RMSE [m]</th><th>融合位置RMSE [m]</th><th>開始前5秒の特徴数中央値</th><th>特徴距離中央値 [m]</th></tr>'''
    for label,r in rows.items():html+=f'<tr><td>{label}</td><td>{r["rovio"]["position_rmse_m"]:.4f}</td><td>{r["mimosa"]["position_rmse_m"]:.4f}</td><td>{r["preflight_features_median"]:.1f}</td><td>{r["preflight_depth_median_m"]:.2f}</td></tr>'
    html+='</table><p>LiDARは発光点が部品内に入らないよう25cm上げ、実配置を推定器に設定した。準備区間の有効点数は約11,200→約15,700点（UniPilot約16,400点）。センサ自体のノイズ・走査角・距離閾値は不変。</p><h2>本比較の平均</h2><table><tr><th>機体</th><th>試行数</th><th>ROVIO RMSE平均 [m]</th><th>融合 RMSE平均 [m]</th></tr>'
    for key,v in aggregates.items():
        if v:html+=f'<tr><td>{key}</td><td>{v["rovio"]["count"]}</td><td>{v["rovio"]["mean_position_rmse_m"]:.4f}</td><td>{v["mimosa"]["mean_position_rmse_m"]:.4f}</td></tr>'
    html+='</table><p>前回DRAGONの極端な誤差は表に表示。下図は条件整合後とUniPilotの全範囲を表示。</p><img src="position_error.png"><p><a href="comparison.pdf">PDF</a> / <a href="summary.json">数値・一致検査・残る差</a></p>'
    (out/'index.html').write_text(html)
    print(json.dumps({k:{m:v[m]['position_rmse_m'] for m in ('rovio','mimosa')} for k,v in rows.items()},indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);report(p.parse_args().root)
