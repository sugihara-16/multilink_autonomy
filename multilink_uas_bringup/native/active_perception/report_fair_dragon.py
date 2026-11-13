#!/usr/bin/env python3
"""All-attempt, layout-paired report; usable on losslessly compressed records."""
import argparse,gzip,html,itertools,json,sys
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from compact_record_io import record_path,open_record
from practical_comparison_metrics import evaluate
from report_practical_comparison import read_pose
from report_dragon_fusion_fix import registration
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics

def read(p):return json.loads(p.read_text())
def write(p,x):p.write_text(json.dumps(x,indent=2)+'\n')
def lines(p):
    with open_record(p) as f:return [json.loads(l) for l in f if l.strip()]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    parser.add_argument('--control-envelope',action='store_true')
    a=parser.parse_args();root=a.root;protocol=read(root/'protocol.json')
    arms=protocol.get('arms',list(dict.fromkeys(t['arm'] for t in protocol['tasks'])))
    colors={'gimbal':'tab:blue','joint':'tab:orange','gimbal_v10':'tab:green'}
    labels={'gimbal':'Gimbal / shared policy','joint':'Body joints','gimbal_v10':'Gimbal / established v10'}
    rows=[];curves={};reference=None;configuration={}
    for task in protocol['tasks']:
        p=root/task['name'];row=dict(**task,estimators={},checks={});checks=row['checks']
        for key in ('completion','validation','input_audit','encoder_audit'):
            if (p/(key+'.json')).exists():row[key]=read(p/(key+'.json'))
        try:
            start=next(e['time'] for e in read(p/'flight_events.json') if e['stage']=='active');row['start_s']=start
            truth=read_pose(p/'truth.csv')
            for est in ('mimosa','rovio'):
                t,error,angles,metrics=evaluate(truth,read_pose(p/(est+'.csv')),start)
                metrics['axis_rmse_m']=np.sqrt(np.mean(error**2,axis=0)).tolist()
                row['estimators'][est]=metrics
                curves[task['name'],est]=(t,np.linalg.norm(error,axis=1))
            checks['full_interval']=True
            active=truth[(truth[:,0]>=start)&(truth[:,0]<=start+182)]
            rpy=Rotation.from_quat(active[:,4:8]).as_euler('xyz',degrees=True)
            row['max_abs_body_tilt_deg']=float(abs(rpy[:,:2]).max())
            row['max_altitude_error_m']=float(abs(active[:,3]-2).max())
            row['max_lateral_error_m']=float(abs(active[:,2]).max())
            c=row['completion'];checks['physical_completion']=bool(c['flight_completed'])
            checks['all_children_exit_zero']=all(v==0 for v in c['processes'].values())
            checks.update(row['validation']['checks'])
            inputs=row['input_audit']
            for sensor in ('imu','lidar'):
                d=inputs[sensor]
                checks[sensor+'_continuous']=all(d[k]==0 for k in ('missing_periods','duplicates','reordered'))
                checks[sensor+'_coverage']=d['first_stamp_ns']*1e-9<=start and d['last_stamp_ns']*1e-9>=start+182
            enc=row['encoder_audit'];checks['image_coverage']=enc['first_ns']*1e-9<=start and enc['last_ns']*1e-9>=start+182
            libs=read(p/'loaded_fusion_libraries.json') if (p/'loaded_fusion_libraries.json').exists() else []
            checks['correct_loaded_library']=bool(libs) and all('/mimosa-batched-drpm-support-view/libmimosa_lib.so' in f for d in libs for f in d['paths'])
            row['registration']=registration(p/'mimosa/drpm_registration.csv',start,start+182)
            checks['lidar_updates']=row['registration']['passed']
            manifest=read(p/'protocol.json')
            checks['symmetric_mode']=manifest['mechanism_comparison'] and not manifest['unipilot_matched'] and manifest['fusion_profile']=='drpm_isotropic20_view'
            cfg=yaml.safe_load((p/'mimosa.yaml').read_text());cfg.pop('logs_directory',None)
            common=dict(rovio=(p/'rovio.info').read_text(),intrinsics=yaml.safe_load((p/'cam0.yaml').read_text()),fusion=cfg,
                        runtime=yaml.safe_load((p/'rovio_runtime.yaml').read_text()),planner=yaml.safe_load((p/'planner.yaml').read_text()))
            if reference is None:reference=common
            configuration[task['name']]={k:v==reference[k] for k,v in common.items()}
            checks['common_configuration']=all(configuration[task['name']].values())
            kin=Kinematics(p/'dragon_slam.urdf')
            q=np.loadtxt(record_path(p/'joint_measurements.csv'),delimiter=',',skiprows=1,ndmin=2)
            q=q[(q[:,0]>=start)&(q[:,0]<=start+182)]
            sample=q[np.unique(np.searchsorted(q[:,0],np.arange(q[0,0],q[-1,0],.1)).clip(0,len(q)-1))]
            Ts=np.array([kin.camera(x[1:]) for x in sample])
            row['camera_translation_span_m']=np.ptp(Ts[:,:3,3],axis=0).tolist()
            row['camera_rotation_excursion_deg']=float(np.rad2deg((Rotation.from_matrix(Ts[0,:3,:3]).inv()*Rotation.from_matrix(Ts[:,:3,:3])).magnitude()).max())
            if a.control_envelope and task['arm']=='joint':
                envelopes=[kin.control.evaluate(x[1:]) for x in sample]
                row['control_envelope']=dict(samples=len(sample),invalid=sum(not e['valid'] for e in envelopes),
                    full_shape_invalid=sum(not kin.valid(x[1:]) for x in sample),
                    max_thrust_fraction=max(e.get('max_thrust_fraction',0) for e in envelopes))
                checks['measured_control_envelope']=row['control_envelope']['invalid']==row['control_envelope']['full_shape_invalid']==0
            vio=np.genfromtxt(record_path(p/'vio.csv'),delimiter=',',names=True)
            vio=vio[(vio['stamp_ns']*1e-9>=start)&(vio['stamp_ns']*1e-9<=start+182)]
            row['visual_diagnostics']=dict(median_features=float(np.median(vio['features'])),fraction_fewer_than_four_features=float(np.mean(vio['features']<4)),
                median_depth_m=float(np.median(vio['median_depth'])),max_median_depth_m=float(np.max(vio['median_depth'])))
            v10=task['arm']=='gimbal_v10'
            decisions=[d for d in lines(p/('decisions.jsonl' if v10 else 'joint_decisions.jsonl')) if start<=(d['stamp_ns']*1e-9 if v10 else d['time'])<=start+182]
            row['view_decisions']=dict(selected=int(sum(np.linalg.norm(np.array(d['target'])-np.array(d['encoder']))>np.deg2rad(.1) if v10 and d.get('encoder') is not None else d['reason']=='select_global_view' for d in decisions)),
                reasons={reason:sum(d['reason']==reason for d in decisions) for reason in sorted(set(d['reason'] for d in decisions))})
        except (OSError,ValueError,KeyError,IndexError,StopIteration) as error:
            row['record_error']=str(error);checks['complete_records']=False
        row['passed']=bool(checks) and all(checks.values());rows.append(row)
    layout=[]
    for k,w in enumerate(protocol['worlds']):
        x=dict(layout=k,world=w,arms={})
        for arm in arms:
            rr=[r for r in rows if r['layout']==k and r['arm']==arm]
            x['arms'][arm]=dict(completed=sum(r['checks'].get('full_interval',False) for r in rr),planned=len(rr),estimators={})
            for est in ('mimosa','rovio'):
                values=[r['estimators'].get(est,{}).get('position_rmse_m') for r in rr]
                x['arms'][arm]['estimators'][est]=dict(values=values,mean=float(np.mean(values)) if all(v is not None for v in values) else None,
                    sd=float(np.std(values,ddof=1)) if len(values)>1 and all(v is not None for v in values) else None)
        layout.append(x)
    aggregate={}
    for est in ('mimosa','rovio'):
        means={arm:[l['arms'][arm]['estimators'][est]['mean'] for l in layout] for arm in arms}
        if any(v is None for vv in means.values() for v in vv):aggregate[est]={'incomplete':True};continue
        comparisons={}
        for baseline in [arm for arm in arms if arm!='joint']:
            delta=np.array(means['joint'])-means[baseline];n=len(delta)
            rng=np.random.default_rng(845122)
            bootstrap=np.mean(delta[rng.integers(0,n,(100000,n))],axis=1)
            null=np.array([np.mean(delta*np.array(s)) for s in itertools.product([-1,1],repeat=n)]) if n<=16 else None
            comparisons[baseline]=dict(difference_joint_minus_gimbal_m=float(delta.mean()),layout_differences_m=delta.tolist(),
                joint_better_layouts=int(sum(delta<0)),gimbal_better_layouts=int(sum(delta>0)),
                layout_bootstrap_95_percent_interval_m=np.quantile(bootstrap,[.025,.975]).tolist() if n>1 else None,
                exact_paired_layout_sign_flip_p=None if null is None or n<2 else float(np.mean(abs(null)>=abs(delta.mean())-1e-12)))
        aggregate[est]=dict(mean_rmse_m={k:float(np.mean(v)) for k,v in means.items()},comparisons_to_joint=comparisons,
            primary_baseline='gimbal_v10' if 'gimbal_v10' in arms else 'gimbal',
            uncertainty_unit='Whole layouts, preserving three repeats together; bootstrap interval and sign-flip symmetry/exchangeability assumption. Shared-policy comparison is a secondary diagnostic, not a baseline chosen after results.')
    state=read(root/'status.json');freeze=read(root/'freeze_audit.json') if (root/'freeze_audit.json').exists() else None
    result=dict(protocol=protocol,stage=state['status'],freeze=freeze,rows=rows,layouts=layout,aggregate=aggregate,
        trials=len(rows),full_intervals=sum(r['checks'].get('full_interval',False) for r in rows),all_checks_passed=sum(r['passed'] for r in rows),configuration=configuration,
        scope='Synthetic constant-section arch tunnel, changing texture distribution; matched DRAGON sensing/estimation/preparation. Primary compares established gimbal control to guarded joint control. Flight control uses truth; not estimated-pose autonomous navigation or a proof about all possible implementations.')
    out=root/'report';out.mkdir(exist_ok=True);write(out/'summary.json',result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(11,5))
    for arm,offset in zip(arms,np.linspace(-.2,.2,len(arms))):
        color=colors[arm]
        for k,l in enumerate(layout):
            vals=[v for v in l['arms'][arm]['estimators']['mimosa']['values'] if v is not None]
            if vals:ax.scatter(np.full(len(vals),k+1+offset),vals,color=color,alpha=.5,s=22)
        ax.plot(np.arange(len(layout))+1+offset,[l['arms'][arm]['estimators']['mimosa']['mean'] for l in layout],'o-',color=color,label=labels[arm])
    ax.set(xlabel='Texture layout',ylabel='Full-route position RMSE [m]',title=f'{len(protocol["seeds"])} run(s) per layout; dots = every run, line = arithmetic mean')
    ax.grid(alpha=.3);ax.legend();fig.tight_layout();fig.savefig(out/'comparison.png',dpi=140)
    ax.set_yscale('log');ax.set_ylabel('Full-route position RMSE [m], logarithmic scale');fig.tight_layout();fig.savefig(out/'comparison_log.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(len(layout),2,figsize=(13,3*len(layout)),squeeze=False)
    for k in range(len(layout)):
        for col,est in enumerate(('mimosa','rovio')):
            ax=axes[k,col]
            for r in rows:
                key=(r['name'],est)
                if r['layout']!=k or key not in curves:continue
                t,e=curves[key];ax.plot(t,e,color=colors[r['arm']],alpha=.65,lw=1)
            ax.set(title=f'Layout {k+1}: {est}',xlabel='Evaluation time [s]',ylabel='Position error [m]');ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(out/'errors.png',dpi=110);plt.close(fig)
    page='<!doctype html><meta charset="utf-8"><title>DRAGON fair comparison</title><style>body{font:16px sans-serif;max-width:1150px;margin:30px auto;line-height:1.6}td,th{padding:7px;border:1px solid #aaa}table{border-collapse:collapse}img{max-width:100%}pre{white-space:pre-wrap}</style>'
    page+='<h1>DRAGON ジンバルと関節：同一条件の比較</h1><p>一定断面のトンネルで、模様の多い方向が位置によって変わる環境。反復数と全試行を以下に表示。初期位置・姿勢だけ整列し、尺度の事後補正はしない。未完走を短い区間の精度として採点しない。</p>'
    if protocol['stage']!='held_out':page+='<p><strong>これは開発・診断用の予備試験。本比較の優劣を判定するデータには含めない。</strong></p>'
    page+=f'<p>全区間取得 {result["full_intervals"]}/{len(rows)}、全検査成功 {result["all_checks_passed"]}/{len(rows)}。段階: {html.escape(protocol["stage"])}。</p>'
    page+='<p>同じ機体質量・初期カメラ位置・センサー・推定器・準備手順を使用。主比較は実績のあるv10ジンバルと安全制約付き関節手法。副対照の有無は取得前の試験計画に明記する。関節は速度4度/秒・加速度8度/秒²、共通手法ジンバルは2軸合成14度/秒。機構の可動範囲の差は残す。飛行制御は真値参照で、推定位置を使う自律航行の検証ではない。</p>'
    page+='<p>修正：到達に長時間かかる視野へ即座に移れると仮定する評価を廃止。2秒以内に停止までできる指令だけを採点し、その間の機体移動による見え方も確認する。カメラの回転・並進を補正する推定処理は両方式共通。</p>'
    page+='<p>位置RMSEは、時刻ごとの位置誤差を二乗平均して平方根を取った値。小さいほど良い。各配置の3回平均を同じ重みで集計。信頼区間は試行を独立な環境と扱わず、配置単位で再抽出して算出。</p>'
    if protocol.get('preparation'):
        page+='<p>全方式で準備の前半に視野を選び、後半は選んだ視野を保持して同じ機体の準備運動を行う。従来のジンバルだけに準備中の視野選択が許されていた不一致を解消した。関節と共通手法ジンバルは、追跡中の特徴を90%以上維持できる移動だけを選ぶ。</p>'
    else:page+='<p>この過去の開発段階では、共有plannerは準備中の視野選択を行っていない。最終の対称準備条件と区別する。</p>'
    page+='<pre>'+html.escape(json.dumps(aggregate,indent=2))+'</pre><table><tr><th>配置</th>'+''.join('<th>'+labels[arm]+' 全値 → 平均 [m]</th>' for arm in arms)+'</tr>'
    for l in layout:
        page+=f'<tr><td>{l["layout"]+1}</td>'
        for arm in arms:
            v=l['arms'][arm]['estimators']['mimosa'];page+='<td>'+', '.join('未完了' if x is None else f'{x:.3f}' for x in v['values'])+' → '+('—' if v['mean'] is None else f'{v["mean"]:.3f}')+'</td>'
        page+='</tr>'
    page+='</table><p><a href="summary.json">全数値・検査・条件</a> / <a href="../protocol.json">取得前の試験計画</a></p><img src="comparison.png"><p>全値を同じ比率で見比べる対数軸（上の通常軸と同じ全試行）。</p><img src="comparison_log.png"><h2>全試行の誤差</h2><p>緑：従来v10ジンバル、青：共通手法ジンバル、橙：関節。左：融合後、右：画像＋慣性推定。</p><img src="errors.png">'
    failures={r['name']:[k for k,v in r['checks'].items() if not v] for r in rows if not r['passed']}
    page+='<h2>検査不合格・未完了</h2><pre>'+html.escape(json.dumps(failures,indent=2))+'</pre>'
    (out/'index.html').write_text(page)
    print(json.dumps(dict(aggregate=aggregate,full_intervals=result['full_intervals'],all_checks_passed=result['all_checks_passed'],failures=failures),indent=2))

if __name__=='__main__':main()
