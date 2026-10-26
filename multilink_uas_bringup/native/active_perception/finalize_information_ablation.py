#!/usr/bin/env python3
"""Read-only acquisition audit and presentation; numeric scoring stays frozen."""
import argparse,collections,gzip,hashlib,json,os,shutil
from pathlib import Path
import numpy as np
import yaml
from report_routes import load
from weathered_trial_acceptance import trial_acceptance


def main():
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);a=parser.parse_args();r=a.root;out=r/'report'
    m=json.loads((r/'campaign.json').read_text());s=json.loads((out/'summary.json').read_text())
    assert m['status']=='complete' and len(m['runs'])==6
    changed=[p for p,d in m['source_sha256'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=d]
    assert not changed,changed
    configs={mode:yaml.safe_load((r/'configs'/f'{mode}.yaml').read_text()) for mode in ['lidar_conditioned','visual_only']}
    differences=[k for k in set().union(*(c.keys() for c in configs.values())) if configs['lidar_conditioned'].get(k)!=configs['visual_only'].get(k)]
    assert differences==['view_information_mode'],differences
    diagnostics={};pids=[];exits={};acceptance={}
    for key in m['order']:
        mode=key.rsplit('_seed',1)[0];f=r/key/'campaign'/(key+'_active');acceptance[key]=trial_acceptance(r/key)
        p=f/'decisions.jsonl.gz';rows=list(map(json.loads,gzip.open(p,'rt'))) if p.exists() else list(map(json.loads,(f/'decisions.jsonl').read_text().splitlines()))
        labels=collections.Counter(v.get('view_information_mode') for v in rows)
        assert labels[mode]>100 and set(labels)<=set([None,mode]),(key,labels)
        assert all(v.get('reason') in ['fixed_baseline','hold_view_for_common_preparation'] for v in rows if v.get('view_information_mode') is None)
        t0=s['trials'][key]['raw']['tracking_start_s'];track=[v for v in rows if t0+10<=v['stamp_ns']*1e-9<t0+170]
        diagnostics[key]=dict(mode_labels={str(k):v for k,v in labels.items()},tracking_reasons=dict(collections.Counter(v['reason'] for v in track)),tracking_scoring_kinds=dict(collections.Counter(v.get('selection_score_kind','no_score') for v in track)))
        outer=json.loads((r/key/'completion.json').read_text());runtime=json.loads((f/'runtime.json').read_text())
        assert outer['stopped'] and runtime['stopped'] and runtime['status']=='route_complete'
        assert all(v==0 for v in outer['processes'].values()) and all(v==0 for v in runtime['processes'].values())
        exits[key]=dict(outer=outer['processes'],inner=runtime['processes'])
        pids.extend(json.loads((r/key/'owners.json').read_text()).values())
        for item in runtime.get('launch_shutdown',{}).get('children',[]):
            pids.append(item['pid']);assert item['exit_code']==0
    alive=[p for p in set(pids) if Path(f'/proc/{p}').exists()];assert not alive,alive
    audit=dict(all_acquisitions_valid=all(v['acquisition_valid'] for v in acceptance.values()),original_strict_valid={k:v['original_strict_valid'] for k,v in acceptance.items()},acceptance=acceptance,
        source_entries_checked=len(m['source_sha256']),source_changes=changed,configuration_differences=differences,all_recorded_exit_codes_zero=True,recorded_pids_absent=len(set(pids)),alive_recorded_pids=alive,exit_codes=exits,
        host_free_bytes=shutil.disk_usage('/mnt/c').free,trial_wall_s=sum(v['wall_s'] for v in m['runs'].values()),audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    findings={}
    for field in ['complete_axial_velocity_rms_mps','vio_local_10s_axial_rms_m','fused_local_10s_axial_rms_m','full_fused_position_rmse_m']:
        li=s['aggregates']['lidar_conditioned'][field];vi=s['aggregates']['visual_only'][field]
        findings[field]=dict(lidar_mean=li['mean'],visual_mean=vi['mean'],mean_reduction_percent=100*(1-li['mean']/vi['mean']),lidar_median=float(np.median(li['values'])),visual_median=float(np.median(vi['values'])),lidar_wins=sum(p['difference']<0 for p in s['comparison'][field]),n=3)
    angles={}
    for seed in m['design']['seeds']:
        pair=[]
        for mode in ['lidar_conditioned','visual_only']:
            key=f'{mode}_seed{seed}';f=r/key/'campaign'/(key+'_active');enc=load(f/'encoders.csv');t0=s['trials'][key]['raw']['tracking_start_s'];ts=t0+np.arange(10,170,.2)
            pair.append(np.column_stack([np.interp(ts,enc['t'],enc[n]) for n in ['yaw','pitch']]))
        diff=np.degrees(pair[0]-pair[1]);angles[str(seed)]=dict(yaw_rms_difference_deg=float(np.sqrt(np.mean(diff[:,0]**2))),pitch_rms_difference_deg=float(np.sqrt(np.mean(diff[:,1]**2))))
    for name,data in [('final_audit',audit),('decision_audit',diagnostics),('findings',dict(metrics=findings,angle_differences=angles,note='Descriptive three-trial comparison in one environment, not a statistical significance claim. Means are strongly affected by visual-only trial 3. Mode-label audit reads decisions.jsonl; the original numeric reporter looked for planner.jsonl, leaving its auxiliary decision dictionaries empty. Numeric scoring is unchanged.'))]:
        (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    html=(out/'index.html').read_text();marker='<section id="ablation-findings">'
    if marker in html:html=html[:html.index(marker)]
    html+='''<section id="ablation-findings"><h2>比較の結論と確認</h2><p>3回平均はLiDAR情報ありが良く、視覚の軸速度誤差14.0%、VIOの10秒軸移動13.8%、融合の10秒軸移動20.0%、全位置15.8%の減少。各指標で3組中2組が改善。全試行で勝つ結果ではない。平均差には視覚のみ3回目の悪化が強く影響し、視覚速度の中央値はLiDARあり0.310、視覚のみ0.305m/sで逆順位。1環境各3回の記述的比較であり、統計的有意とは呼ばない。</p><p>向きの大きな推移は両方式で似ているが、切替時刻や途中の選択に差がある。視覚の拘束自体にも平均改善があるため、融合位置の改善だけを見た比較ではない。ただし今回、融合誤差相殺の因果的な分離は実施していない。</p><p>6本とも90m完走、速度拘束利用率100%。設定差はview_information_modeだけ。取得の検査は全て合格し、元のplanner_continuous=falseは全試行で保持。角度指令間隔の最大は1.82〜4.27秒で、元の厳密判定を合格に書き換えていない。全記録終了コード0、記録PIDの消滅を確認。</p><p><a href="final_audit.json">最終監査</a> / <a href="decision_audit.json">実行された評価方式・判断の内訳</a> / <a href="findings.json">平均・中央値・各回の勝敗</a></p></section>'''
    (out/'index.html').write_text(html)
    print(json.dumps(dict(findings=findings,source_entries=audit['source_entries_checked'],pids=audit['recorded_pids_absent'],wall_minutes=audit['trial_wall_s']/60),indent=2))

if __name__=='__main__':main()
