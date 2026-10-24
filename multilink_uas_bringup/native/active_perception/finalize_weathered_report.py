#!/usr/bin/env python3
"""Present completed weathered-tunnel results without changing numeric metrics."""
import argparse,json,re,hashlib
from pathlib import Path
import numpy as np
from report_direction_repeats import ORDER,LABEL


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();out=a.root/'report'
    s=json.loads((out/'summary.json').read_text());assert s['status']=='complete'
    html='<section id="findings"><h2>比較結果</h2>';findings={}
    titles={'complete_axial_velocity_rms_mps':'VIO速度の軸方向誤差 [m/s]（全区間利用のみ）','vio_local_10s_axial_rms_m':'VIOの10秒軸方向移動誤差 [m]','fused_local_10s_axial_rms_m':'融合後の10秒軸方向移動誤差 [m]','full_fused_position_rmse_m':'融合後の全90m位置誤差 [m]'}
    for name,c in s['conditions'].items():
        rows={};m=c['methods'];html+=f'<h3>{name}</h3><table><tr><th>指標</th><th>固定5方向平均</th><th>事後最良固定</th><th>偏向</th><th>偏向が固定平均より良い回数</th><th>全固定より良い回数</th></tr>'
        for field,title in titles.items():
            arrays=[m[n][field]['values'] for n in ORDER[:6]]
            if any(v is None for aa in arrays for v in aa):
                rows[field]={'complete':False};html+=f'<tr><td>{title}</td><td colspan="5">全区間の拘束がない方向を含むため平均順位は算出しない。利用率と利用時誤差は下表。</td></tr>';continue
            fixed=np.asarray(arrays[:5]);active=np.asarray(arrays[5]);bestidx=int(np.argmin(fixed.mean(1)))
            row=dict(complete=True,fixed_mean=float(fixed.mean()),best_fixed=ORDER[bestidx],best_fixed_mean=float(fixed.mean(1)[bestidx]),active_mean=float(active.mean()),active_values=active.tolist(),fixed_mean_values=fixed.mean(0).tolist(),wins_fixed_mean=int((active<fixed.mean(0)).sum()),wins_all_fixed=int((active<fixed.min(0)).sum()),n=len(active))
            rows[field]=row
            html+=f'<tr><td>{title}</td><td>{row["fixed_mean"]:.4f}</td><td>{row["best_fixed_mean"]:.4f} ({LABEL[row["best_fixed"]]})</td><td>{row["active_mean"]:.4f}</td><td>{row["wins_fixed_mean"]}/{row["n"]}</td><td>{row["wins_all_fixed"]}/{row["n"]}</td></tr>'
        html+='</table><p>各値は独立飛行ごとのRMSを計算してから平均。固定5方向平均は5方向等重み。最良固定は各指標について結果から選び、一つの向きを全長で使った参考値。</p>'
        findings[name]=rows
    campaign=json.loads((a.root/'campaign.json').read_text())
    html+='<h3>取得条件と偏向の停止判断</h3><p>進行管理はカメラが新しい角度を選んだ場合だけでなく、入力条件を満たさず角度を保持して処理を終えた場合も完了として受け取る。推定式・センサ・角度選択規則は変更していない。全3回にこの待ち合わせを適用した。</p><p>修正前の未完了3試行はinvalid_trialsへ保持し、完走の3試行と混ぜていない。初回完走後に監査分類を変更し、planner_continuousだけの不合格は取得の失敗ではなく偏向方式の結果として残した。元の監査falseは書き換えていない。その他のセンサ入力・推定出力の連続性、設定、同一入力、終了検査は必要条件として維持した。</p><table><tr><th>飛行</th><th>角度更新の最大間隔[s]</th><th>元の全検査合格</th><th>取得条件合格</th></tr>'
    for key,run in campaign['runs'].items():
        acc=run['acceptance']
        html+=f"<tr><td>{key}</td><td>{acc['controller_timing']['max_gap_s']:.3f}</td><td>{acc['original_strict_valid']}</td><td>{acc['acquisition_valid']}</td></tr>"
    html+='</table><p>VIO移動誤差の固定平均は、追跡に失敗した方向の大きな値も含む。方向別の表と最良固定を併せて判断する。全区間のVIO速度が利用できない方向を除外して、同じ指標の全方向平均と呼ぶことはしない。</p></section>'
    doc=(out/'index.html').read_text();doc=re.sub(r'<section id="findings">.*?</section>','',doc,flags=re.S);doc=doc.replace('</h1>','</h1>'+html,1)
    (out/'index.html').write_text(doc)
    (out/'findings.json').write_text(json.dumps(dict(conditions=findings,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
    print(json.dumps(findings,indent=2))

if __name__=='__main__':main()
