#!/usr/bin/env python3
"""Aggregate frozen held-out comparisons without treating trials as independent frames."""
import argparse,html,json,hashlib
from pathlib import Path
import numpy as np


def comparison_entries(results, required=False):
    """Keep flight reports distinct from diagnostic metadata in the same root."""
    if not isinstance(results, dict):
        if required:
            raise ValueError('Declared validation results must be a flight mapping')
        return []
    entries = []
    for name, row in results.items():
        canonical = (isinstance(name, str) and name.endswith('_active')
                     and isinstance(row, dict) and isinstance(row.get('valid'), bool)
                     and isinstance(row.get('checks'), dict))
        if canonical:
            entries.append((name, row))
        elif required:
            raise ValueError('Malformed declared validation flight: '+str(name))
    return entries


def frozen_evidence(base,frozen):
    """Check the saved flight inputs, not whatever source happens to exist now."""
    protocol=json.loads((base/'protocol.json').read_text());checks={}
    for filename,expected in frozen['files'].items():
        source=Path(filename)
        if source.name in ('native_fusion_calibration.json','native_fusion_ar1_calibration.json','native_fusion_window1s_calibration.json','native_fusion_translation_window1s_calibration.json','native_fusion_displacement_gravity_calibration.json','native_fusion_velocity_gravity_calibration.json'):target=base/'fusion_calibration.json'
        elif '/active_perception/' in filename:
            target=base/'implementation'/filename.split('/active_perception/',1)[1]
        elif source.name in ('unipilot_gimbal.py','unipilot_gimbal_sim.launch.py','rovio_initialization.py'):
            target=base/'implementation'/source.name
        elif source.name in ('mimosa_ordered_node','mimosa_tilt_node','mimosa_gravity_translation_node','mimosa_velocity_node','mimosa_batched_node'):
            checks[source.name]=protocol['binary_sha256']['mimosa_node']==expected;continue
        elif any(k in str(source) for k in ('rovio-joint-information','rovio-joint-geometry')):
            if source.name=='rovio_gimbal':
                checks['joint_rovio_binary']=protocol['binary_sha256']['rovio_gimbal']==expected;continue
            target=base/'implementation'/('joint_rovio_build.json' if source.name=='manifest.json' else 'joint_rovio_generated.cpp')
        elif 'mimosa-batched-vision' in str(source):
            if source.name=='libmimosa_lib.so':
                checks['batched_fusion_library']=protocol['binary_sha256']['libmimosa_lib.so']==expected;continue
            target=base/'implementation/batched_fusion'/source.name
        elif source.name=='libmimosa_lib.so' and 'mimosa-fixed-gravity' in str(source):
            checks['fixed_gravity_library']=protocol['binary_sha256']['libmimosa_lib.so']==expected;continue
        elif source.name=='libmimosa_lib.so' and 'mimosa-sensor-bias' in str(source):
            checks['sensor_bias_library']=protocol['binary_sha256']['libmimosa_lib.so']==expected;continue
        elif source.name in ('libgz-sim8.so.8.15.0','libno_crash_dump.so'):
            checks[source.name]=protocol['binary_sha256'].get(source.name)==expected;continue
        else:raise ValueError('No saved evidence mapping for '+filename)
        actual=hashlib.sha256(target.read_bytes()).hexdigest() if target.exists() else None
        checks[filename]=actual==expected
        # A development-only startup correction may atomically publish the
        # same generated model. Independent frozen trials do not carry this
        # exception and must use the exact final source hash.
        eq=frozen.get('bootstrap_equivalence') if source.name=='unipilot_gimbal.py' else None
        if not checks[filename] and eq:
            checks[filename]=(expected==eq['new_source_sha256'] and actual==eq['old_source_sha256']
                and all((base/name).exists() and hashlib.sha256((base/name).read_bytes()).hexdigest()==sha
                        for name,sha in eq['flight_output_sha256'].items()))
    for name,expected in frozen.get('binary_sha256',{}).items():
        checks['binary/'+name]=protocol['binary_sha256'].get(name)==expected
    for name,expected in frozen.get('protocol_fields',{}).items():
        checks['protocol/'+name]=name in protocol and protocol[name]==expected
    if 'upstream' in frozen:checks['upstream']=protocol.get('upstream')==frozen['upstream']
    return checks


def assess(results,criteria,expected_routes=None,expected_seeds=None):
    # Six independent physical flights, never thousands of correlated frames.
    ratios=[r['active_fixed_position_ratio']['mimosa'] for r in results]
    per_route={name:float(np.mean([r['active_fixed_position_ratio']['mimosa'] for r in results if r['route']==name]))
               for name in sorted({r['route'] for r in results})}
    identities=[(r['route'],r['seed']) for r in results]
    expected=({(r,s) for r in expected_routes for s in expected_seeds}
              if expected_routes is not None and expected_seeds is not None else None)
    complete=(len(results)==6 and all(r['valid'] for r in results)
              and len(set(identities))==6
              and all(sum(r['route']==name for r in results)==2 for name in per_route)
              and len(per_route)==3 and (expected is None or set(identities)==expected))
    checks={'six_valid_pairs':complete,
            'at_least_five_wins':sum(x<1 for x in ratios)>=criteria['min_wins'],
            'each_route_mean_ratio':bool(per_route) and all(x<=criteria['per_route_mean_ratio_max'] for x in per_route.values()),
            'overall_mean_ratio':bool(ratios) and float(np.mean(ratios))<=criteria['all_pair_mean_ratio_max'],
            'worst_pair_ratio':bool(ratios) and max(ratios)<=criteria['max_single_pair_ratio']}
    return dict(complete=complete,passed=all(checks.values()),checks=checks,pairs=len(results),
                wins=sum(x<1 for x in ratios),mean_ratio=float(np.mean(ratios)) if ratios else None,
                worst_ratio=max(ratios) if ratios else None,per_route_mean_ratio=per_route)


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path)
    p.add_argument('--candidate',default='v6');a=p.parse_args()
    frozen_path=a.root/f'frozen_candidate_{a.candidate}.json'
    frozen=json.loads(frozen_path.read_text())
    campaigns=frozen.get('validation_campaigns',[f'validation-{a.candidate}-seed{s-20261000}' for s in frozen['validation_seeds']])
    records=[];history=[]
    for campaign in sorted(a.root.iterdir()):
        if not campaign.is_dir() or not (campaign/'results.json').exists():continue
        results=json.loads((campaign/'results.json').read_text())
        for name,r in comparison_entries(results,required=campaign.name in campaigns):
            row=dict(r,campaign=campaign.name,route=name.removesuffix('_active'),report=str(campaign.relative_to(a.root)/'index.html'))
            history.append(row)
            if campaign.name in campaigns and 'metrics' in row:
                row['frozen_candidate_checks']=frozen_evidence(campaign/name,frozen)
                row['valid']=row['valid'] and all(row['frozen_candidate_checks'].values())
                records.append(row)
    result=assess(records,frozen['criterion'],frozen['routes'],frozen['validation_seeds'])
    result.update(candidate=frozen['candidate'],seeds=frozen['validation_seeds'],
                  scope='Three routes, two held-out seeds each. Same physical flight and common initialization per pair. Tracking-only first-pose alignment; no scale correction. No truth used by estimator or planner.',
                  comparisons=records)
    early=a.root/f'{a.candidate}_early_rejection.json'
    if early.exists():result.update(rejected_early=True,early_rejection=json.loads(early.read_text()),passed=False)
    (a.root/'validation_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (a.root/f'validation_summary_{a.candidate}.json').write_text(json.dumps(result,indent=2)+'\n')
    state='事前の改善基準を満たしました' if result['passed'] else ('事前の改善基準には未到達です' if result['complete'] else '検証継続中：安定した改善はまだ確認していません')
    if early.exists():state='候補'+a.candidate+'は不採用。次の制御方式を開発中です。'
    def table(rows):
        body=[]
        for row in rows:
            if 'metrics' not in row:
                body.append('<tr><td>'+html.escape(row['campaign']+'/'+row['route'])+'</td><td colspan="7">未完了・監査不合格。元記録を保持。</td></tr>');continue
            f=row['metrics']['fixed'];g=row['metrics']['active'];ratio=row['active_fixed_position_ratio']['mimosa']
            cells=[f'<a href="{html.escape(row["report"])}">{html.escape(row["campaign"]+" / "+row["route"])}</a>',str(row['valid']),f'{f["mimosa"]["position_rmse_m"]:.3f}',f'{g["mimosa"]["position_rmse_m"]:.3f}',f'{100*(1-ratio):+.1f}%',f'{f["mimosa"]["rotation_rmse_deg"]:.3f} → {g["mimosa"]["rotation_rmse_deg"]:.3f}',f'{f["rovio"]["position_rmse_m"]:.3f} → {g["rovio"]["position_rmse_m"]:.3f}',f'{f["rovio"]["rotation_rmse_deg"]:.3f} → {g["rovio"]["rotation_rmse_deg"]:.3f}']
            body.append('<tr>'+''.join('<td>'+v+'</td>' for v in cells)+'</tr>')
        return '<table><tr><th>試行</th><th>監査</th><th>固定・融合位置 m</th><th>可動・融合位置 m</th><th>位置誤差低減</th><th>融合回転 deg</th><th>ROVIO位置 m</th><th>ROVIO回転 deg</th></tr>'+''.join(body)+'</table>'
    text='''<!doctype html><meta charset="utf-8"><title>Gimbal improvement validation</title>
<style>body{font-family:sans-serif;margin:30px auto;max-width:1400px;line-height:1.6}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:7px;border:1px solid #ccc;text-align:right}td:first-child,th:first-child{text-align:left}.state{font-size:1.3em;font-weight:bold}</style>
<h1>ジンバルによる推定精度の改善検証</h1>'''
    text+='<p class="state">'+state+'</p><p>ROVIO＋MIMOSA、DRPM融合'+('ON' if frozen.get('drpm_fusion',False) else 'OFF')+'。位置・回転は二乗平均平方根誤差。各組で同じ実飛行・IMU・推定器設定を使用。初期状態を共通化し、IMU／LiDAR／画像由来姿勢を取得時刻順に融合します。評価前の26秒の準備往復も共通で、精度集計は従来の軌道追従区間だけです。開始時の位置・姿勢を揃え、距離の尺度は補正しません。</p>'
    text+='<h2>設定固定後の反復検証</h2><p>3経路×未使用の乱数条件2つ。完了組数 '+str(result['pairs'])+'/6。事前基準：6組中5組以上改善、各経路の平均誤差比≤0.9、全組の平均比≤0.8、最悪の比≤1.5。未完了のまま成功と判定しません。</p>'+table(records)
    text+='<p><a href="validation_summary.json">検証値・判定</a> / <a href="'+html.escape(frozen_path.name)+'">固定した設定とコードのSHA256</a> / <a href="ordered-imu-null-a1-seed17/null_reproducibility.json">同一入力で推定が一致する対照</a></p>'
    focused=text.replace('href="validation_summary.json"','href="validation_summary_'+html.escape(a.candidate)+'.json"')
    focused+='<p>各試行のリンクから六軸誤差の図とPDFを確認できます。ROVIO単体の誤差は別列に記載しています。融合精度の改善を、視覚推定単体の改善とは区別してください。</p>'
    (a.root/('validation_'+a.candidate+'.html')).write_text(focused)
    text+='<h2>開発中の全試行</h2><p>条件が異なる試行の絶対誤差を直接比較しません。同じ行の固定→可動だけが対照です。nullはカメラを動かさない再現性診断、pitch20は別の初期視野であり、動的偏向の成果には数えません。入力欠落・途中停止の試行も保持しています。</p>'+table([r for r in history if r not in records])
    (a.root/'index.html').write_text(text)
    print(json.dumps({k:v for k,v in result.items() if k!='comparisons'},indent=2))
if __name__=='__main__':main()
