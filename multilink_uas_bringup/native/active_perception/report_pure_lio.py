#!/usr/bin/env python3
"""Audit and evaluate one same-flight pure LIO versus selected visual fusion."""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import tempfile
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from report_fusion_sidecar import tum, ordered
from report_estimator_validation import evaluate
from report_gimbal_campaign import summarize
from report_routes import csv_path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    root=args.output;observers=root/'observers'
    manifest=json.loads((observers/'manifest.json').read_text())
    flight=Path(manifest['flight'])
    route=flight.name.removesuffix('_active')
    protocol=json.loads((flight/'protocol.json').read_text())
    seed=protocol['seed'];duration=protocol['route_end']
    paired=summarize(root/'campaign')[flight.name]
    (root/'paired_reference_audit.json').write_text(json.dumps(paired,indent=2)+'\n')
    checks=dict(reference_valid=paired['valid'],observers_stopped=manifest.get('stopped',False),
                observers_completed=manifest['status']=='reference_ended',
                observer_processes_zero=all(v==0 for v in manifest['processes'].values()))
    main_pose=tum(flight/'mimosa/graph_manager_odometry.tum')
    duplicate=tum(observers/'duplicate_active/mimosa/graph_manager_odometry.tum')
    common,i,j=np.intersect1d(np.rint(main_pose[:,0]*1e9).astype(np.int64),
                             np.rint(duplicate[:,0]*1e9).astype(np.int64),return_indices=True)
    checks['duplicate_poses_exactly_equal']=bool(len(common)>1 and np.array_equal(main_pose[i,1:],duplicate[j,1:]))
    checks['duplicate_covers_reference']=len(common)==len(main_pose)
    lower,upper=0,2**63-1
    reference_rows,reference_audit=ordered(flight/'mimosa/ordered_inputs.csv',lower,upper)
    duplicate_rows,duplicate_audit=ordered(observers/'duplicate_active/mimosa/ordered_inputs.csv',lower,upper)
    pure_rows,pure_audit=ordered(observers/'pure_lio/mimosa/ordered_inputs.csv',lower,upper)
    def signature(rows):
        return [tuple(r[k] for k in ['stamp_ns','source','imu_count','imu_upto_ns']) for r in rows]
    checks['duplicate_all_inputs_exact']=signature(reference_rows)==signature(duplicate_rows)
    def lidar_signature(rows):
        return [(r['stamp_ns'],r['imu_upto_ns']) for r in rows if r['source']=='lidar']
    checks['pure_lio_same_lidar_stamps_and_imu_prefix']=lidar_signature(reference_rows)==lidar_signature(pure_rows)
    checks['pure_lio_no_vio_events']=bool(pure_rows) and all(r['source']=='lidar' for r in pure_rows)
    checks['pure_lio_continuous_inputs']=all(pure_audit[k] for k in ['chronological','imu_monotonic','imu_brackets_events']) and pure_audit['sources']['lidar']['continuous']
    def cumulative_imu_at_scans(rows):
        total=0;result=[]
        for row in rows:
            total+=int(row['imu_count'])
            if row['source']=='lidar':result.append((row['stamp_ns'],total))
        return result
    checks['pure_lio_same_cumulative_imu_at_each_scan']=cumulative_imu_at_scans(reference_rows)==cumulative_imu_at_scans(pure_rows)
    # Preserve the all-lifetime equality checks separately. Pure LIO need not
    # wait for a VIO watermark and can consume a final scan after the fused
    # node has stopped. Accept that only if the complete reference is an exact
    # prefix, including cumulative IMU counts, and extras follow evaluation.
    lifetime_checks={key:checks.pop(key) for key in
        ['pure_lio_same_lidar_stamps_and_imu_prefix','pure_lio_same_cumulative_imu_at_each_scan']}
    reference_scans=lidar_signature(reference_rows);pure_scans=lidar_signature(pure_rows)
    checks['pure_lio_complete_reference_scan_prefix_exact']=pure_scans[:len(reference_scans)]==reference_scans
    checks['pure_lio_complete_reference_imu_prefix_exact']=cumulative_imu_at_scans(pure_rows)[:len(reference_scans)]==cumulative_imu_at_scans(reference_rows)
    reference=yaml.safe_load((flight/'mimosa.yaml').read_text())
    pure=yaml.safe_load((observers/'pure_lio/mimosa.yaml').read_text())
    reference.pop('logs_directory');pure.pop('logs_directory')
    reference['odometry']['manager']['enabled']=False
    checks['only_external_odometry_disabled']=reference==pure
    for filename in ['velocity_factors.csv','tilt_factors.csv']:
        p=csv_path(observers/'pure_lio/mimosa'/filename)
        if p.exists():
            import gzip
            with (gzip.open(p,'rt') if p.suffix=='.gz' else p.open()) as f:
                count=sum(1 for _ in csv.DictReader(f))
        else:count=0
        checks['no_'+filename.removesuffix('.csv')]=count==0
    expected=json.loads((flight/'protocol.json').read_text())['binary_sha256']
    loaded=json.loads((observers/'loaded_libraries.json').read_text())
    checks['same_executable']=all(v['binary_sha256']==expected['mimosa_node'] for v in manifest['variants'].values())
    checks['same_loaded_library']=all(set(v.values())=={expected['libmimosa_lib.so']} for v in loaded.values())
    checks['external_odometry_disconnected']=manifest['variants']['pure_lio']['remaps']['odometry/manager/odometry_in'].endswith('/unused_external_odometry')
    selected=json.loads((Path(__file__).resolve().parent/'selected_gimbal.json').read_text())
    checks['selected_sources_unchanged']=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in selected['files'].items())
    metrics={};errors={}
    for label,path,name in [('fusion_fixed',flight/'fixed','mimosa'),('fusion_active',flight,'mimosa'),
                            ('rovio_fixed',flight/'fixed','rovio'),('rovio_active',flight,'rovio')]:
        metrics[label],grid,errors[label],stamps=evaluate(path,name)
    data=tum(observers/'pure_lio/mimosa/graph_manager_odometry.tum')
    with tempfile.TemporaryDirectory(dir=root,prefix='evaluation_') as temporary:
        proxy=Path(temporary)
        (proxy/'protocol.json').symlink_to(flight/'protocol.json')
        for name in ['truth.csv','route_tracking.csv']:
            source=csv_path(flight/name);(proxy/source.name).symlink_to(source)
        data[:,0]=np.rint(data[:,0]*1e9)
        np.savetxt(proxy/'mimosa.csv',data,delimiter=',',header='stamp_ns,x,y,z,qx,qy,qz,qw',comments='',fmt=['%.0f']+['%.17g']*7)
        metrics['pure_lio'],grid,errors['pure_lio'],stamps=evaluate(proxy,'mimosa')
    checks['pure_lio_full_continuous_output']=metrics['pure_lio']['coverage']==1 and metrics['pure_lio']['max_output_gap_s']<=.10001
    extra=pure_scans[len(reference_scans):]
    checks['extra_pure_lio_scans_only_after_evaluation']=all(int(row[0])>int(stamps[-1]) for row in extra)
    result=dict(valid=all(checks.values()),checks=checks,metrics=metrics,
        scope=f'One shared physical flight, {route}, seed {seed}. Pure LIO disables ALL ROVIO factors from initialization onward. Selected LiDAR four-DoF registration, IMU model, library, noise and priors retained. The pure estimate does not control the camera or vehicle. Tracking-only initial rigid alignment; no scale fit.',
        audits=dict(reference=reference_audit,duplicate=duplicate_audit,pure_lio=pure_audit),
        duplicate_samples=len(common),route=route,seed=seed,
        full_lifetime_stream_checks=lifetime_checks,
        terminal_input_difference=dict(reference_scan_count=len(reference_scans),pure_scan_count=len(pure_scans),
            extra_scan_stamps_ns=[int(row[0]) for row in extra],evaluation_end_stamp_ns=int(stamps[-1]),
            meaning='Lifetime equality is retained as measured; only a verified extra tail after evaluation is irrelevant to the accuracy comparison. No evaluation samples or timestamps were removed.'))
    (root/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axs=plt.subplots(3,2,figsize=(13,9),sharex=True)
    labels={'pure_lio':'MIMOSA LIO only','fusion_fixed':'MIMOSA + ROVIO, fixed','fusion_active':'MIMOSA + ROVIO, gimbal'}
    for key,color in [('pure_lio','#333333'),('fusion_fixed','#2474b5'),('fusion_active','#df6619')]:
        for n,ax in enumerate([*axs[:,0],*axs[:,1]]):
            ax.plot(grid,errors[key][:,n],color=color,lw=.9,label=labels[key]);ax.grid(alpha=.3)
            ax.set_ylabel(['x [m]','y [m]','z [m]','roll [deg]','pitch [deg]','yaw [deg]'][n])
        np.savetxt(root/(key+'_errors.csv'),np.column_stack([grid,stamps,errors[key]]),delimiter=',',
                   header='route_t,stamp_ns,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg',comments='',fmt=['%.6f','%.0f']+['%.12g']*6)
    axs[0,0].legend();axs[-1,0].set_xlabel('Trajectory time [s]');axs[-1,1].set_xlabel('Trajectory time [s]')
    fig.suptitle(route+': same-flight pure LIO / fixed visual fusion / gimbal visual fusion')
    fig.tight_layout();fig.savefig(root/'comparison.png',dpi=135);fig.savefig(root/'comparison.pdf');plt.close(fig)
    names=dict(pure_lio='MIMOSA単体',fusion_fixed='MIMOSA＋ROVIO・固定',fusion_active='MIMOSA＋ROVIO・偏向',rovio_fixed='ROVIO単体・固定',rovio_active='ROVIO単体・偏向')
    rows=''.join(f'<tr><td>{names[k]}</td><td>{metrics[k]["position_rmse_m"]:.4f}</td><td>{metrics[k]["rotation_rmse_deg"]:.4f}</td><td>{metrics[k]["position_max_m"]:.4f}</td></tr>' for k in names)
    page=f'''<!doctype html><meta charset="utf-8"><title>{route} MIMOSA単体比較</title>
<style>body{{font:17px sans-serif;line-height:1.7;max-width:1100px;margin:30px auto}}table{{border-collapse:collapse}}td,th{{border:1px solid #ccc;padding:8px}}img{{width:100%}}</style>
<h1>{route}：MIMOSA単体と視覚融合の比較</h1><p>精度比較に必要な入力・出力検査合格：{result['valid']}</p>
<p>同じ1回の飛行・LiDAR・IMUを使用。単体では初期化からROVIOの速度・傾き拘束を両方無効化。LiDARの並進3成分＋方位の登録方式、IMU・雑音・初期不確かさは採用構成のまま。DRPM OFF。追従{duration:g}秒、始点の位置・姿勢だけ整合、尺度補正なし。過去試行との同一入力比較ではなく、今回の同時比較。</p>
<table><tr><th>構成</th><th>位置RMSE [m]</th><th>回転RMSE [deg]</th><th>最大位置誤差 [m]</th></tr>{rows}</table>
<p><a href="comparison.pdf">六軸PDF</a> / <a href="comparison.json">数値・監査結果</a></p><img src="comparison.png">
<p>終了時の記録件数は完全一致ではありません。純LIOは融合側の全入力を同じIMU累積数で処理した後、評価終了より後の追加{len(extra)}スキャンを処理しました。全評価区間と融合側の全記録区間は一致し、評価時刻・評価点を削除していません。厳密な全寿命一致の不合格記録もJSONに保持しています。</p>
<p>実装：{html.escape(result['scope'])}</p>'''
    (root/'index.html').write_text(page)
    print(json.dumps(dict(valid=result['valid'],failed=[k for k,v in checks.items() if not v],metrics=metrics),indent=2))


if __name__=='__main__':main()
