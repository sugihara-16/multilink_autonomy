#!/usr/bin/env python3
"""Predeclared 3-layout x 5-repeat, scoring-only ablation; no adaptive tuning."""
import argparse, collections, csv, gzip, hashlib, json, shutil, signal, subprocess, sys, time
from pathlib import Path
import numpy as np
import yaml
from run_lio_failure_comparisons import HERE, INPUTS, read, run_one, verify_selected
from run_zab_additional_conditions import save
from weathered_trial_acceptance import trial_acceptance
import report_switching_tunnel_sparse as sparse_report
sparse_report.ORDER=["front","active"]
evaluate=sparse_report.evaluate
from report_information_ablation import MODES, FIELDS, TITLES
from report_routes import load

LAYOUTS=[24301,24302,24303]
SEEDS=[20262801,20262802,20262803,20262804,20262805]

def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def schedule():
    result={}
    # Interleave layouts and alternate which arm runs first: 8 versus 7 pairs.
    for repeat,seed in enumerate(SEEDS):
        for i,layout in enumerate(LAYOUTS):
            for mode in (MODES if (repeat+i)%2==0 else MODES[::-1]):
                key=f'layout{layout}_{mode}_seed{seed}'
                result[key]=dict(layout=layout,mode=mode,seed=seed,repeat=repeat+1)
    return result

def stats(values):
    finite=[float(v) for v in values if v is not None and np.isfinite(v)]
    complete=len(finite)==len(values) and len(values)>0
    return dict(values=values,n=len(values),available_n=len(finite),mean=float(np.mean(finite)) if complete else None,
                median=float(np.median(finite)) if complete else None,std=float(np.std(finite,ddof=1)) if complete and len(finite)>1 else None)

def prepare(root,assets):
    root.mkdir(parents=True,exist_ok=True)
    path=root/'campaign.json'
    if path.exists():return read(path)
    configs=root/'configs';configs.mkdir()
    cfg=yaml.safe_load((HERE/'motion_view_v10.yaml').read_text())
    for mode in MODES:(configs/f'{mode}.yaml').write_text(yaml.safe_dump(dict(cfg,view_information_mode=mode)))
    entries={};geometry={};textures={}
    for layout in LAYOUTS:
        p=assets/str(layout);entries[str(layout)]=read(p/'catalog.json')[f'weathered{layout}']
        geometry[str(layout)]={f.name:digest(f) for f in sorted((p/'meshes').glob('*.dae'))}
        textures[str(layout)]={f.name:digest(f) for f in sorted((p/'textures').glob('*.png'))}
        env=read(p/'environment.json')
        assert env['axial_normal_component_max']==0 and env['width_m']==5 and env['length_m']==100
    assert all(v==geometry[str(LAYOUTS[0])] for v in geometry.values())
    assert len({json.dumps(v,sort_keys=True) for v in textures.values()})==3
    reference=Path('/root/multilink_autonomy_ws/assets/weathered_tunnel_20261024')
    assert geometry[str(LAYOUTS[0])]=={f.name:digest(f) for f in sorted((reference/'meshes').glob('*.dae'))}
    files=set(read(HERE/'selected_gimbal.json')['files'])
    files.update(str(f) for f in HERE.rglob('*') if f.is_file() and f.suffix in ['.py','.yaml','.json','.hpp','.cpp','.info'] and '__pycache__' not in f.parts)
    files.update(str(f) for f in assets.rglob('*') if f.is_file())
    files.update(str(f) for f in configs.glob('*.yaml'))
    trials=schedule()
    m=dict(status='prepared',created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),assets=str(assets),entries=entries,
           order=list(trials),trials=trials,runs={},source_sha256={f:digest(f) for f in sorted(files)},
           asset_audit=dict(geometry_identical=True,geometry_matches_previous=True,texture_distinct=True,geometry=geometry,textures=textures),
           design=dict(layouts=LAYOUTS,seeds=SEEDS,segments_s=[[10.,170.]],distance_m=90,speed_mps=.5,drpm=False,
             primary='Actual accepted visual axial velocity RMS and 1 s-bin availability. Complete RMS only if coverage is 100%.',
             secondary=FIELDS[2:],
             difference='Only view scoring H/U/weak inputs differ. LiDAR-conditioned: actual values; visual-only: zero H, identity U, equal translation weights. Same finite prior, visual forecast, retention/recovery, motion limits, preparation, estimators and fusion.',
             interpretation='Visual-only is a view-scoring ablation, not vision-only estimation or IGLOV reproduction. Both arms still use LiDAR fusion and MIMOSA state/time synchronization.',
             comparison_unit='3 new texture layouts, 5 separate physical flights per arm per layout. Matched requested seeds do not imply identical sensor streams. 5 fixed monocular views and pure LIO are same-flight diagnostic controls.',
             analysis='All 30 trials; per-layout mean, sample standard deviation, median, paired requested-seed differences and wins. Overall means weight layouts equally. Describe these 3 layouts, not 15 independent environments. No optional stopping or posthoc parameter/seed selection.',
             acceptance='All acquisition checks required; only old planner_continuous false due to held-angle commands is allowed and separately reported, as in the preceding protocol. No accuracy-based exclusion. Infrastructure failures retained for inspection.',
             truth='Ground truth used for common body control and evaluation only; no truth or texture-distribution metadata for view selection.',
             storage='No GUI, raw rosbag, continuous images or point clouds. Compressed numeric logs; stop below 8 GiB Windows free.'))
    snapshot=root/'implementation';snapshot.mkdir()
    for name in ['run_information_holdout.py','view_information_ablation.py','ablation_planner_node.py','run_information_ablation_array.py','report_switching_tunnel_sparse.py','weathered_trial_acceptance.py','build_weathered_tunnel.py']:
        shutil.copy2(HERE/name,snapshot/name)
    save(path,m);return m

def check(m):
    changed=[p for p,h in m['source_sha256'].items() if not Path(p).exists() or digest(p)!=h]
    if changed:raise RuntimeError('Frozen inputs changed: '+str(changed))
    if shutil.disk_usage('/mnt/c').free<8*2**30:raise RuntimeError('Windows free space below 8 GiB')
    verify_selected()

def collect(root,key,m):
    record,samples=evaluate(root,key,m['design']['segments_s'])
    metrics=dict(record['methods']['active'])
    metrics['full_fused_position_rmse_m']=record['full_fused']['active']['mimosa']['position_rmse_m']
    result=dict(**m['trials'][key],metrics=metrics,raw=record)
    save(root/key/'holdout_metrics.json',result)
    with gzip.open(root/key/'velocity_samples.csv.gz','wt') as f:
        w=csv.DictWriter(f,fieldnames=['trial','method','section','elapsed_s','axial_velocity_error_mps','error_xyz_mps']);w.writeheader();w.writerows(samples)
    return result

def report(root,m):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    out=root/'report';out.mkdir(exist_ok=True)
    trials={k:read(root/k/'holdout_metrics.json') for k in m['order'] if m['runs'].get(k,{}).get('status')=='complete'}
    aggregates={};pairs={}
    for layout in LAYOUTS:
        aggregates[str(layout)]={mode:{field:stats([t['metrics'][field] for t in trials.values() if t['layout']==layout and t['mode']==mode]) for field in FIELDS} for mode in MODES}
        pairs[str(layout)]={}
        for field in FIELDS:
            rows=[]
            for seed in SEEDS:
                a=trials.get(f'layout{layout}_lidar_conditioned_seed{seed}');b=trials.get(f'layout{layout}_visual_only_seed{seed}')
                if a and b:
                    x,y=a['metrics'][field],b['metrics'][field]
                    rows.append(dict(seed=seed,lidar=x,visual=y,difference=None if x is None or y is None else x-y))
            pairs[str(layout)][field]=rows
    overall={mode:{f:stats([aggregates[str(l)][mode][f]['mean'] for l in LAYOUTS]) for f in FIELDS} for mode in MODES}
    result=dict(status=m['status'],design=m['design'],trials=trials,aggregates=aggregates,overall_equal_layouts=overall,comparison=pairs)
    save(out/'summary.json',result)
    def fmt(v):return '—' if v is None else f'{v:.5f}'
    html='''<!doctype html><meta charset="utf-8"><title>未使用3配置での視野評価比較</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;line-height:1.7}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:7px}img{max-width:100%}</style><h1>視覚のみ vs LiDAR情報を使う偏向：未使用3配置</h1><p>形状は前回と同一。新規の模様配置24301/24302/24303、各方式5回、90m・0.5m/s。途中の調整や精度による試行選別なし。方式間は別飛行であり、同じ要求seedでも同じセンサ列とは限らない。</p><p>違いは偏向の評価にLiDAR行列を使うかだけ。両方式ともROVIO+MIMOSA、DRPM OFF、視覚の予測・保持・復帰・ジンバル制限・融合重みは共通。視覚のみでもLiDAR融合は動作する。</p><p>主指標は巡航10〜170秒の実際の視覚速度拘束の軸誤差と利用率。補助にVIO/融合の10秒軸移動誤差、全90m融合位置誤差。尺度補正なし。平均±標本標準偏差、中央値と全試行を表示。元のplanner_continuous判定と取得合格は区別する。</p><p><a href="comparison.pdf">PDF</a> / <a href="summary.json">全数値</a> / <a href="../campaign.json">事前条件・進捗</a> / <a href="final_audit.json">終了時監査</a></p>'''
    html+=f'<p>状態: {m["status"]}、完了 {len(trials)}/30</p>'
    with PdfPages(out/'comparison.pdf') as pdf:
        for layout in LAYOUTS:
            html+=f'<h2>配置 {layout}</h2><table><tr><th>方式</th>'+''.join(f'<th>{t}</th>' for t in TITLES)+'</tr>'
            for mode in MODES:
                html+=f'<tr><td>{mode}</td>'+''.join(f'<td>{fmt(aggregates[str(layout)][mode][f]["mean"])} ± {fmt(aggregates[str(layout)][mode][f]["std"])}<br>中央値 {fmt(aggregates[str(layout)][mode][f]["median"])}</td>' for f in FIELDS)+'</tr>'
            html+='</table>'
            fig,axs=plt.subplots(2,3,figsize=(16,9))
            for ax,field,title in zip(axs.ravel(),FIELDS,TITLES):
                for j,mode in enumerate(MODES):
                    vs=aggregates[str(layout)][mode][field]['values'];valid=[(i,v) for i,v in enumerate(vs) if v is not None]
                    if valid:
                        ax.scatter([j+(i-2)*.035 for i,v in valid],[v for i,v in valid]);mean=aggregates[str(layout)][mode][field]['mean']
                        if mean is not None:ax.plot([j-.2,j+.2],[mean]*2)
                for pair in pairs[str(layout)][field]:
                    if pair['difference'] is not None:ax.plot([0,1],[pair['lidar'],pair['visual']],color='gray',alpha=.25)
                ax.set_xticks([0,1],['LiDAR-conditioned','Visual-only']);ax.set_title(title);ax.grid(alpha=.3)
                if field=='coverage':ax.set_ylim(0,1.05)
            axs[-1,-1].axis('off');fig.suptitle(f'Unseen texture layout {layout}: all trials, lines pair requested seeds');fig.tight_layout();pdf.savefig(fig);fig.savefig(out/f'metrics_{layout}.png',dpi=130);plt.close(fig)
            html+=f'<img src="metrics_{layout}.png">'
            fig,axs=plt.subplots(5,2,figsize=(15,15),squeeze=False)
            for i,seed in enumerate(SEEDS):
                for mode in MODES:
                    key=f'layout{layout}_{mode}_seed{seed}'
                    if key not in trials:continue
                    enc=load(root/key/'campaign'/(key+'_active')/'encoders.csv');t0=trials[key]['raw']['tracking_start_s'];mask=(enc['t']>=t0)&(enc['t']<=t0+182)
                    for j,angle in enumerate(['yaw','pitch']):axs[i,j].plot(enc['t'][mask]-t0,np.degrees(enc[angle][mask]),label=mode)
                for j,angle in enumerate(['yaw','pitch']):axs[i,j].set(title=f'{layout} repeat {i+1}: {angle}',xlabel='Tracking time [s]',ylabel='Angle [deg]');axs[i,j].grid(alpha=.3)
                if axs[i,0].lines:axs[i,0].legend()
            fig.tight_layout();pdf.savefig(fig);fig.savefig(out/f'angles_{layout}.png',dpi=110);plt.close(fig)
            html+=f'<details><summary>実際の偏向角（pitch正は下、負は上）</summary><img src="angles_{layout}.png"></details>'
    html+='<h2>全試行</h2><table><tr><th>試行</th>'+''.join(f'<th>{t}</th>' for t in TITLES)+'</tr>'
    for key,t in trials.items():html+='<tr><td>'+key+'</td>'+''.join('<td>'+fmt(t['metrics'][f])+'</td>' for f in FIELDS)+'</tr>'
    html+='</table><h2>取得検査</h2>'
    for key in trials:html+=f'<details><summary>{key}</summary><pre>'+json.dumps(m['runs'][key]['acceptance'],indent=2)+'</pre></details>'
    (out/'index.html').write_text(html)
    print('REPORT',out/'index.html',flush=True)

def audit(root,m):
    assert m['status']=='complete' and len(m['runs'])==30
    check(m);cfg=[yaml.safe_load((root/'configs'/f'{mode}.yaml').read_text()) for mode in MODES]
    differences=[k for k in set(cfg[0])|set(cfg[1]) if cfg[0].get(k)!=cfg[1].get(k)]
    assert differences==['view_information_mode']
    pids=[];runs={}
    for key in m['order']:
        f=root/key/'campaign'/(key+'_active');mode=m['trials'][key]['mode']
        accept=trial_acceptance(root/key)
        path=f/'decisions.jsonl.gz';op=gzip.open
        if not path.exists():path=f/'decisions.jsonl';op=open
        with op(path,'rt') as stream:rows=[json.loads(line) for line in stream]
        labels=collections.Counter(v.get('view_information_mode') for v in rows)
        assert labels[mode]>100 and set(labels)<={None,mode},(key,labels)
        assert all(v.get('reason') in ['fixed_baseline','hold_view_for_common_preparation'] for v in rows if v.get('view_information_mode') is None)
        outer=read(root/key/'completion.json');runtime=read(f/'runtime.json')
        assert outer['stopped'] and runtime['stopped'] and runtime['status']=='route_complete'
        assert all(v==0 for v in outer['processes'].values()) and all(v==0 for v in runtime['processes'].values())
        pids.extend(read(root/key/'owners.json').values())
        for item in runtime.get('launch_shutdown',{}).get('children',[]):pids.append(item['pid']);assert item['exit_code']==0
        runs[key]=dict(acceptance=accept,mode_labels={str(k):v for k,v in labels.items()},exit_codes=dict(outer=outer['processes'],inner=runtime['processes']))
    alive=[p for p in set(pids) if Path(f'/proc/{p}').exists()];assert not alive,alive
    save(root/'report/final_audit.json',dict(source_entries_checked=len(m['source_sha256']),configuration_differences=differences,runs=runs,
         all_acquisitions_valid=True,all_recorded_exit_codes_zero=True,recorded_pids_absent=len(set(pids)),host_free_bytes=shutil.disk_usage('/mnt/c').free,trial_wall_s=sum(v['wall_s'] for v in m['runs'].values())))

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--assets',type=Path,default=Path('/root/multilink_autonomy_ws/assets/weathered_holdout_20261026'))
    p.add_argument('--prepare-only',action='store_true');p.add_argument('--report-only',action='store_true');a=p.parse_args();root=a.output.resolve();m=prepare(root,a.assets.resolve());path=root/'campaign.json'
    if a.report_only:
        report(root,m)
        if m['status']=='complete':audit(root,m)
        return
    check(m)
    if a.prepare_only:print('PREPARED',len(m['order']),'trials',len(m['source_sha256']),'frozen inputs');return
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    m['status']='running';save(path,m)
    try:
        for key in m['order']:
            if key in m['runs']:
                if m['runs'][key]['status']=='complete':continue
                raise RuntimeError('Inspect unfinished attempt before resume: '+key)
            check(m);trial=m['trials'][key];template=read(INPUTS/'templates/paired_flight.json');template[1]=str(HERE/'run_information_holdout_pair.py')
            template[template.index('--planner-config')+1]=str(root/'configs'/f'{trial["mode"]}.yaml')
            m['runs'][key]=dict(status='running',**trial);save(path,m);print('START',key,flush=True)
            result=run_one(root,key,m['entries'][str(trial['layout'])],template,trial['seed'],record_geometry=True)
            subprocess.run([sys.executable,str(HERE/'report_information_holdout_pair.py'),'--trial',str(root/key)],check=True)
            accept=trial_acceptance(root/key);geo=read(root/key/'observers/common_lidar.jsonl.status.json')
            if geo.get('samples',0)<100:raise RuntimeError('Missing geometry')
            metrics=collect(root,key,m)
            result.update(status='complete',**trial,acceptance=accept,geometry=geo)
            m['runs'][key]=result;check(m);save(path,m);print('FINISHED',key,json.dumps(metrics['metrics']),flush=True)
        m['status']='complete';check(m)
    except BaseException as exc:
        m['status']='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed';m['error']=repr(exc);raise
    finally:save(path,m)
    report(root,m);audit(root,m)

if __name__=='__main__':main()
