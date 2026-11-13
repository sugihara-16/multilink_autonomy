#!/usr/bin/env python3
"""Recomputable paired, layout-level analysis of six-camera UniPilot flights."""
import argparse, itertools, json, re
from pathlib import Path
import numpy as np
import yaml
from compact_record_io import record_path
from practical_comparison_metrics import evaluate
from report_practical_comparison import read_pose
from report_fusion_sidecar import ordered, coincident_input_signature

ORDER=['yaw_m50','front','yaw_p50','pitch_p90','pitch_m90','active']
LABEL=['固定 −50°','固定 正面','固定 ＋50°','固定 下','固定 上','ジンバル']


def read(p):return json.loads(p.read_text())
def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')


def trial(root):
    paths=dict(active=root,front=root/'fixed',**{n:root/'views'/n for n in ORDER if n not in ('active','front')})
    result=dict(checks={},views={});checks=result['checks']
    try:
        runtime=read(root/'runtime.json');route=read(root/'route_status.json')
        result['runtime']=runtime
        checks['physical_route_complete']=route['stage']=='complete' and route['failure'] is None and route['route_time_s']>=182
        checks['clean_shutdown']=runtime['stopped'] and runtime['status']=='route_complete' and all(v==0 for v in runtime['processes'].values()) and runtime['launch_shutdown']['all_clean'] and runtime['gazebo_shutdown']['all_clean']
        tracking=np.genfromtxt(record_path(root/'route_tracking.csv'),delimiter=',',names=True)
        moving=tracking[tracking['route_t']>0];start=float(moving['sim_t'][0]-moving['route_t'][0])
        result['tracking_start_s']=start
        truth=read_pose(root/'truth.csv')
        result['physical_tracking_max_m']=float(np.max(moving['tracking_error_m'])) if 'tracking_error_m' in moving.dtype.names else None
        audit=read(root/'array_input_audit.json');inputs=read(root/'input_audit.json')
        result['input_audit']=inputs
        result['raw_upstream_continuity']={n:all(s[k]==0 for k in ('missing_periods','duplicates','reordered')) for n,s in inputs.items()}
        for name,sample in inputs.items():
            # native_slam_inputs discards the first 2s of IMU BEFORE forwarding
            # to either estimator, irrespective of the additional hover gate.
            # Preserve raw defects, but do not call discarded input an estimator gap.
            if name=='imu':
                period=1000000
                gaps=sample.get('gaps',[])
                recorded_missing=sum((b-a)//period-1 for a,b in gaps)
                harmless=recorded_missing==sample['missing_periods'] and all(b<=sample['first_stamp_ns']+2000000000 for a,b in gaps)
                checks[name+'_continuous']=sample['count']>0 and sample['duplicates']==0 and sample['reordered']==0 and harmless
            else:
                checks[name+'_continuous']=sample['count']>0 and all(sample[k]==0 for k in ('missing_periods','duplicates','reordered'))
            checks[name+'_covers_tracking']=sample['first_stamp_ns']*1e-9<=start and sample['last_stamp_ns']*1e-9>=start+182
        shared=read(root/'shared_initialization.json')
        checks['active_images_continuous']=shared['missing_periods']==0 and shared['first_stamp_ns']*1e-9<=start and shared['last_stamp_ns']*1e-9>=start+182
        libraries=read(root/'loaded_array_libraries.json')
        checks['six_correct_loaded_fusion_libraries']=len(libraries)==6 and all(r['paths'] and all('/mimosa-batched-vision/libmimosa_lib.so' in p for p in r['paths']) for r in libraries)
        base=yaml.safe_load((root/'fixed/mimosa.yaml').read_text());base.pop('logs_directory')
        normalize=lambda t:re.sub(r'(qCM_[xyzw]\s+)[^;\s]+;',r'\1POSE;',t)
        reference_info=normalize((root/'fixed/rovio.info').read_text())
        upper=round((start+181.9)*1e9)
        ref,_=ordered(root/'fixed/mimosa/ordered_inputs.csv',0,upper)
        reference_sig=coincident_input_signature(ref)
        expected=np.arange(int(np.ceil((start+.1)*20-1e-6))*50000000,int(np.floor((start+181.9)*20+1e-6))*50000000+1,50000000,dtype=np.int64)
        for name,directory in paths.items():
            view=result['views'][name]=dict(metrics={},checks={});vc=view['checks']
            for estimator in ('mimosa','rovio'):
                try:
                    _,_,_,metrics=evaluate(truth,read_pose(directory/(estimator+'.csv')),start,182,max_gap_s=.100001 if estimator=='mimosa' else .050001)
                    view['metrics'][estimator]=metrics
                except (OSError,ValueError,IndexError) as error:view[estimator+'_error']=str(error)
            vc['full_intervals']=len(view['metrics'])==2
            config=yaml.safe_load((directory/'mimosa.yaml').read_text());config.pop('logs_directory')
            vc['same_fusion_parameters']=config==base
            vc['same_filter_except_orientation']=normalize((directory/'rovio.info').read_text())==reference_info
            vc['same_intrinsics']=(directory/'cam0.yaml').read_bytes()==(root/'fixed/cam0.yaml').read_bytes()
            vc['drpm_off']=not read(directory/'protocol.json')['drpm_fusion']
            events,event_audit=ordered(directory/'mimosa/ordered_inputs.csv',0,upper)
            vc['same_event_and_imu_prefix']=coincident_input_signature(events)==reference_sig
            vc['ordered_inputs']=all(event_audit[k] for k in ('chronological','imu_monotonic','imu_brackets_events'))
            view['event_audit']=event_audit
            if name!='active':
                a=audit[name];frames=np.array(a['image_stamps'],dtype=np.int64);poses=set(a['extrinsic_stamps'])
                vc['correct_fixed_extrinsic']=a['rotation_error_max_rad']<1e-8 and a['position_error_max_m']<1e-8
                vc['all_tracking_images']=np.array_equal(frames[(frames>=expected[0])&(frames<=expected[-1])],expected)
                vc['all_tracking_image_poses']=all(int(x) in poses for x in expected)
                view['rotation_error_max_rad']=a['rotation_error_max_rad'];view['position_error_max_m']=a['position_error_max_m']
            view['passed']=all(vc.values())
        checks['all_six_views_pass']=len(result['views'])==6 and all(v['passed'] for v in result['views'].values())
    except (OSError,ValueError,KeyError,IndexError,TypeError) as error:
        result['error']=str(error);checks['complete_records']=False
    result['checks']={k:bool(v) for k,v in checks.items()};result['passed']=bool(checks) and all(checks.values())
    return result


def contrast(difference):
    d=np.array(difference,dtype=float);n=len(d)
    signs=np.array(list(itertools.product((-1.,1.),repeat=n)))
    perm=signs@d/n;observed=float(d.mean());eps=1e-12*max(1.,float(abs(d).max()))
    rng=np.random.default_rng(61113)
    boots=d[rng.integers(0,n,size=(100000,n))].mean(1)
    return dict(layout_differences_m=d.tolist(),mean_fixed_minus_gimbal_m=observed,
        bootstrap_95_percentile_m=np.quantile(boots,[.025,.975]).tolist(),
        p_one_sided=float(np.mean(perm>=observed-eps)),p_two_sided=float(np.mean(abs(perm)>=abs(observed)-eps)),
        gimbal_better_layouts=int(np.sum(d>0)),layouts=n)


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root
    protocol=read(root/'protocol.json');rows={t['name']:dict(task=t,**trial(root/t['name'])) for t in protocol['tasks']}
    layouts=sorted(set(t['layout'] for t in protocol['tasks']));means={};per_layout={};stats={}
    for estimator in ('mimosa','rovio'):
        means[estimator]={};per_layout[estimator]={}
        for name in ORDER:
            values=[r['views'].get(name,{}).get('metrics',{}).get(estimator,{}).get('position_rmse_m') for r in rows.values()]
            means[estimator][name]=float(np.mean(values)) if all(v is not None for v in values) else None
        for layout in layouts:
            subset=[r for r in rows.values() if r['task']['layout']==layout]
            per_layout[estimator][str(layout)]={}
            for name in ORDER:
                vals=[r['views'].get(name,{}).get('metrics',{}).get(estimator,{}).get('position_rmse_m') for r in subset]
                per_layout[estimator][str(layout)][name]=float(np.mean(vals)) if all(v is not None for v in vals) else None
        stats[estimator]={}
        if all(v is not None for v in means[estimator].values()):
            matrix=np.array([[per_layout[estimator][str(k)][name] for name in ORDER] for k in layouts])
            for j,name in enumerate(ORDER[:-1]):stats[estimator][name]=contrast(matrix[:,j]-matrix[:,-1])
            sort=sorted(ORDER[:-1],key=lambda n:stats[estimator][n]['p_one_sided']);last=0.
            for j,name in enumerate(sort):
                last=max(last,min(1.,(5-j)*stats[estimator][name]['p_one_sided']))
                stats[estimator][name]['p_one_sided_holm_five']=last
            stats[estimator]['all_five_intersection_union_p']=max(stats[estimator][n]['p_one_sided'] for n in ORDER[:-1])
            stats[estimator]['fixed_average']=contrast(matrix[:,:5].mean(1)-matrix[:,-1])
            stats[estimator]['retrospective_best_fixed_per_layout']=contrast(matrix[:,:5].min(1)-matrix[:,-1])
    result=dict(protocol=protocol,results=rows,mean_position_rmse_m=means,per_layout_mean_position_rmse_m=per_layout,
        statistics=stats,all_trials_passed=all(r['passed'] for r in rows.values()),
        complete_camera_intervals=sum(len(v['metrics'])==2 for r in rows.values() for v in r['views'].values()),
        expected_camera_intervals=6*len(rows),freeze_audit=read(root/'freeze_audit.json') if (root/'freeze_audit.json').exists() else None)
    out=root/'report';out.mkdir(exist_ok=True);write(out/'summary.json',result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(11,5))
    for j,name in enumerate(ORDER):
        vals=[per_layout['mimosa'][str(k)][name] for k in layouts]
        ax.plot(np.arange(len(layouts))+1,vals,'o-',label=name)
    ax.set(xlabel='Texture layout',ylabel='Mean position RMSE [m]',xticks=np.arange(len(layouts))+1)
    ax.grid(alpha=.25);ax.legend(ncol=3);fig.tight_layout();fig.savefig(out/'layout_means.png',dpi=150);plt.close(fig)
    f=lambda v:'—' if v is None else f'{v:.4f}'
    page='<!doctype html><meta charset="utf-8"><title>UniPilot 固定5方向 vs ジンバル</title><style>body{font:16px sans-serif;max-width:1200px;margin:30px auto;line-height:1.7}table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:7px}img{max-width:100%}</style>'
    page+=f'<h1>UniPilot：固定5方向とジンバルの同時比較</h1><p>{len(layouts)}配置×各{len(rows)//len(layouts)}回、計{len(rows)}飛行。形状が一定の100mアーチ型トンネルで、汚れ・ひびが多い面が位置によって変化する。各飛行で6台の仮想カメラを同時撮影し、それぞれ独立した単眼推定器で処理。複数カメラの融合なし。90m、0.5m/s、全182秒の位置誤差を評価。</p>'
    page+='<p>RMSEは位置ずれの二乗平均平方根。小さいほど良い。表は各試行のRMSEを平均した値。共通の慣性・距離データ、同一の画像仕様と推定器設定、DRPM OFF。最初の位置・姿勢だけを整列し、尺度の補正なし。準備運動は評価から除外。</p>'
    page+=f'<p>全区間を取得したカメラ系統：{result["complete_camera_intervals"]}/{result["expected_camera_intervals"]}。全取得・公平性・終了検査合格：{result["all_trials_passed"]}。失敗も除外せず記録。</p>'
    page+='<table><tr><th>カメラ</th><th>融合位置誤差 [m]</th><th>視覚・慣性位置誤差 [m]</th></tr>'
    for n,label in zip(ORDER,LABEL):page+=f'<tr><td>{label}</td><td>{f(means["mimosa"][n])}</td><td>{f(means["rovio"][n])}</td></tr>'
    page+=f'</table><h2>配置を単位とした比較</h2><p>各配置の反復平均を先に求め、{len(layouts)}配置を分析単位にする。同じ飛行の対応を維持した符号反転検定。帰無仮説の下で配置間の差が独立で符号対称という仮定が必要。片側は「固定の誤差がジンバルより大きい」を試験前に指定。5方向それぞれの判定にはHolm補正を併記。95%区間は配置の対応を保つブートストラップ百分位区間であり、検定の逆算ではない。</p>'
    page+='<table><tr><th>比較対象</th><th>固定−ジンバル [m]</th><th>95%区間 [m]</th><th>片側p</th><th>両側p</th><th>片側p・Holm</th><th>ジンバル良好な配置</th></tr>'
    for n,label in zip(ORDER[:-1],LABEL[:-1]):
        s=stats['mimosa'].get(n)
        if s:page+=f'<tr><td>{label}</td><td>{f(s["mean_fixed_minus_gimbal_m"])}</td><td>{s["bootstrap_95_percentile_m"][0]:.3f} ～ {s["bootstrap_95_percentile_m"][1]:.3f}</td><td>{s["p_one_sided"]:.6g}</td><td>{s["p_two_sided"]:.6g}</td><td>{s["p_one_sided_holm_five"]:.6g}</td><td>{s["gimbal_better_layouts"]}/{s["layouts"]}</td></tr>'
    page+='</table>'
    if stats['mimosa']:
        page+=f'<p>「平均で全5固定方向より良い」という同時主張の片側検定（intersection-union）：p={stats["mimosa"]["all_five_intersection_union_p"]:.6g}。</p>'
    page+='<img src="layout_means.png"><h2>配置ごとの3回平均 [m]</h2><table><tr><th>配置</th>'+''.join('<th>'+x+'</th>' for x in LABEL)+'</tr>'
    for k in layouts:page+='<tr><td>'+str(k+1)+'</td>'+''.join('<td>'+f(per_layout['mimosa'][str(k)][n])+'</td>' for n in ORDER)+'</tr>'
    page+='</table><h2>全試行 [m]</h2><table><tr><th>試行</th>'+''.join('<th>'+x+'</th>' for x in LABEL)+'<th>検査</th></tr>'
    for name,r in rows.items():
        page+='<tr><td>'+name+'</td>'+''.join('<td>'+f(r['views'].get(n,{}).get('metrics',{}).get('mimosa',{}).get('position_rmse_m'))+'</td>' for n in ORDER)+'<td>'+str(r['passed'])+'</td></tr>'
    page+='</table><p>この結論の対象は同じ生成方法による模様配置と単純形状のトンネル。任意の実環境での優位性や実時間処理性能を保証しない。真値は飛行制御と評価だけに使用。全処理の完了を待ってシミュレーション時刻を進める。</p><p><a href="summary.json">全数値・仮定・監査</a> / <a href="../protocol.json">事前固定した試験条件</a></p>'
    (out/'index.html').write_text(page)
    print(json.dumps(dict(means=means,passed=result['all_trials_passed'],complete=result['complete_camera_intervals'],expected=result['expected_camera_intervals']),indent=2),flush=True)


if __name__=='__main__':main()
