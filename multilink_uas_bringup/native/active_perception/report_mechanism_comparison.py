"""Symmetric joint/gimbal trial evaluation; full-route, no scale correction."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
import yaml
from report_articulated_comparison import series
from report_dragon_fusion_fix import registration

def fusion(path):
    c=yaml.safe_load((path/'mimosa.yaml').read_text());c.pop('logs_directory',None);return c

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root
    protocol=json.loads((root/'comparison_protocol.json').read_text());out=root/'report';out.mkdir(exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    rows={};curves={};audits={}
    for seed in protocol['seeds']:
        paths={m:root/f'{m}_seed{seed}' for m in ('joint','gimbal')}
        for mechanism,path in paths.items():
            key=f'{mechanism}_seed{seed}'
            valid=json.loads((path/'validation.json').read_text());events={e['stage']:e['time'] for e in json.loads((path/'flight_events.json').read_text())}
            rows[key]=dict(estimators={},motion=valid,path=str(path))
            for estimator in ('mimosa','rovio'):
                t,e,r,metrics=series(path,estimator);rows[key]['estimators'][estimator]=metrics;curves[key,estimator]=(t,e,r)
            reg=registration(path/'mimosa/drpm_registration.csv',events['active'],events['active']+182)
            done=json.loads((path/'completion.json').read_text());loaded=json.loads((path/'loaded_fusion_library.json').read_text());local=json.loads((path/'protocol.json').read_text())
            checks=dict(acquisition=valid['passed'],lidar_retained=reg['passed'],completed=done['flight_completed'],
                processes_exit_zero=all(v==0 for v in done['processes'].values()),
                owned_processes_stopped=not any(Path('/proc',str(pid)).exists() for pid in json.loads((path/'owners.json').read_text()).values()),
                corrected_library=bool(loaded['maps']) and all('/mimosa-batched-drpm-support-view/libmimosa_lib.so' in line for line in loaded['maps']),
                frozen_sources=all(protocol['sha256'].get(k)==v for k,v in local['sha256'].items() if k in protocol['sha256']),
                no_extra_sweep='articulation' not in events)
            audits[key]=dict(checks=checks,registration=reg,passed=all(checks.values()))
        joint,gimbal=paths['joint'],paths['gimbal']
        jm,gm=[json.loads((p/'articulated_manifest.json').read_text()) for p in (joint,gimbal)]
        same_files=['rovio.info','rovio_runtime.yaml','cam0.yaml','planner.yaml','world_serial.sdf']
        checks={f:same for f in same_files if (same:=(joint/f).read_bytes()==(gimbal/f).read_bytes()) is not None}
        checks.update(fusion_settings=fusion(joint)==fusion(gimbal),initial_camera=np.allclose(jm['T_FC_camera'],gm['T_FC_camera'],atol=1e-12),lidar_pose=jm['lidar_pose_fc']==gm['lidar_pose_fc'])
        jp,gp=[json.loads((p/'protocol.json').read_text()) for p in (joint,gimbal)]
        checks['same_binaries']=all(jp['sha256'][k]==gp['sha256'][k] for k in jp['sha256'] if k.endswith(('/rovio_articulated','/mimosa_batched_node','/libmimosa_lib.so')))
        audits[f'pair_{seed}']=dict(checks=checks,passed=all(checks.values()))
    means={m:{e:float(np.mean([rows[f'{m}_seed{s}']['estimators'][e]['position_rmse_m'] for s in protocol['seeds']])) for e in ('mimosa','rovio')} for m in ('joint','gimbal')}
    data_audits={}
    for key,row in rows.items():
        path=Path(row['path']);original=audits[key]['checks']
        checks={k:v for k,v in original.items() if k not in ('acquisition','processes_exit_zero')}
        checks.update({k:v for k,v in row['motion']['checks'].items() if k!='joint_slew'})
        done=json.loads((path/'completion.json').read_text());bad={k:v for k,v in done['processes'].items() if v!=0}
        post_tracking_shutdown=False
        if bad=={'actuator':1} and (path/'gimbal.csv').exists():
            samples=np.loadtxt(path/'gimbal.csv',delimiter=',',skiprows=1,ndmin=2)
            end=row['motion']['evaluation_interval_s'][1]
            post_tracking_shutdown=bool(samples[-1,0]>end+5 and 'Unable to convert call argument' in (path/'actuator.log').read_text())
        checks['processes_healthy_through_evaluation']=not bad or post_tracking_shutdown
        data_audits[key]=dict(passed=all(checks.values()),checks=checks,post_tracking_shutdown_error=post_tracking_shutdown,
            motion_limit_passed=row['motion']['checks']['joint_slew'])
    data_passed=all(v['passed'] for v in data_audits.values()) and all(v['passed'] for k,v in audits.items() if k.startswith('pair_'))
    v10_path=Path('/root/uas_native/logs/dragon-fusion-fix-20261110/report/summary.json')
    v10=json.loads(v10_path.read_text())['aggregates']['DRAGON corrected']['mimosa']['mean_position_rmse_m']
    result=dict(data_valid=data_passed,data_audits=data_audits,existing_v10_reference=dict(mean_position_rmse_m=v10,path=str(v10_path),scope='Historical v10 and rotation-only ROVIO; a different viewing policy/adapter from this symmetric comparison'),analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),results=rows,mean_position_rmse_m=means,audits=audits,passed=all(x['passed'] for x in audits.values()),protocol=protocol,
                evaluation='182 s full tracking, initial SE3 alignment only; 10 Hz common timestamps, no scale fitting or clipped intervals')
    (out/'summary.json').write_text(json.dumps(result,indent=2))
    with PdfPages(out/'comparison.pdf') as pdf:
        for seed in protocol['seeds']:
            for estimator in ('mimosa','rovio'):
                fig,axs=plt.subplots(3,2,figsize=(13,9),sharex=True)
                for mechanism in ('joint','gimbal'):
                    t,e,r=curves[f'{mechanism}_seed{seed}',estimator]
                    for i in range(3):axs[i,0].plot(t,e[:,i],label=mechanism);axs[i,1].plot(t,r[:,i],label=mechanism)
                for i in range(3):axs[i,0].set_ylabel('XYZ'[i]+' error [m]');axs[i,1].set_ylabel(['Roll','Pitch','Yaw'][i]+' rotvec error [deg]')
                for ax in axs.flat:ax.grid(alpha=.3)
                axs[0,0].legend();axs[-1,0].set_xlabel('Tracking time [s]');axs[-1,1].set_xlabel('Tracking time [s]')
                fig.suptitle(f'DRAGON {estimator.upper()} | seed {seed} | 90 m, 0.5 m/s');fig.tight_layout();pdf.savefig(fig);fig.savefig(out/f'{seed}_{estimator}.png',dpi=130);plt.close(fig)
    html='''<!doctype html><meta charset="utf-8"><title>DRAGON ジンバル対関節</title><style>body{font:16px sans-serif;line-height:1.7;max-width:1200px;margin:30px auto}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #aaa}img{width:100%}</style><h1>DRAGON：ジンバル版と関節版の対称比較</h1><p>模様24301、90m、0.5m/s、各3試行。センサ、初期カメラ姿勢・位置、LiDAR配置、ROVIOの位置・姿勢変換対応版、修正済みDRPM、視野の採点、準備運動を共通化。探索候補と動作制約、実際の機体運動は各機構に従う。真値は飛行制御と事後評価だけに使用。</p><p>今回の共通視野選択は関節版の地図＋速度予測方式。以前のジンバルv10の4.08mとは実装条件が異なるため、過去値を対照として混ぜていない。同じ指定seedでも観測ノイズ列が一致するわけではない。</p><p><a href="comparison.pdf">6軸比較PDF</a> / <a href="summary.json">数値と検査結果</a></p><table><tr><th>seed</th><th>機構</th><th>融合位置RMSE [m]</th><th>VIO位置RMSE [m]</th><th>融合終端誤差 [m]</th><th>光軸最大偏向 [deg]</th></tr>'''
    for seed in protocol['seeds']:
        for mechanism in ('joint','gimbal'):
            row=rows[f'{mechanism}_seed{seed}'];es=row['estimators']
            html+=f"<tr><td>{seed}</td><td>{mechanism}</td><td>{es['mimosa']['position_rmse_m']:.3f}</td><td>{es['rovio']['position_rmse_m']:.3f}</td><td>{es['mimosa']['position_end_m']:.3f}</td><td>{row['motion']['camera_optical_axis_excursion_deg']:.2f}</td></tr>"
    html+='</table><p>取得データ・設定整合の検査: '+str(data_passed)+'。全検査成功（速度・終了コードも含む）: '+str(result['passed'])+'</p>'
    html+='<p>関節版2回目は実測速度12.597°/s。18,181区間中1区間、10msだけ既存12°/sの監査基準を超過した。閾値は変更せず不合格を保持し、正常取得された90m全体を評価している。ジンバル1回目のactuatorは評価区間終了から5秒以上後まで記録を継続したが、終了時のROS購読破棄で終了コード1になった。この例外も保持。</p>'
    html+=f'<p>既存の採用済みジンバルv10構成の過去3回平均は{v10:.3f}m。今回の38.882mは、その構成そのものの再現値ではなく、両機構の視野選択・SE3対応推定器・準備時の視線保持まで共通化した比較値である。共通化した構成が既存ジンバル構成より悪化していることも結果として残す。</p><pre>'+json.dumps(means,indent=2)+'</pre>'
    for seed in protocol['seeds']:
        for estimator in ('mimosa','rovio'):html+=f'<img src="{seed}_{estimator}.png">'
    (out/'index.html').write_text(html)
    print(json.dumps(dict(means=means,audits_passed=result['passed']),indent=2))
    if not data_passed:raise SystemExit('Data/configuration audit failed; see summary.json')

if __name__=='__main__':main()
