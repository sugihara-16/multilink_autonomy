#!/usr/bin/env python3
"""Descriptive final presentation; does not modify acquisition or metric definitions."""
import argparse,hashlib,json,shutil
from pathlib import Path
import numpy as np
from run_information_holdout import FIELDS,MODES,LAYOUTS,collect,check


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();r=a.root;out=r/'report'
    m=json.loads((r/'campaign.json').read_text());s=json.loads((out/'summary.json').read_text())
    assert m['status']=='complete' and len(s['trials'])==30
    check(m)
    findings={}
    for field in FIELDS[1:]:
        rows={}
        for layout in LAYOUTS:
            ag=s['aggregates'][str(layout)];li,vi=ag[MODES[0]][field],ag[MODES[1]][field]
            pairs=s['comparison'][str(layout)][field]
            valid=[p for p in pairs if p['difference'] is not None]
            rows[str(layout)]=dict(lidar=li,visual=vi,mean_reduction_percent=100*(1-li['mean']/vi['mean']) if li['mean'] is not None and vi['mean'] else None,
                lidar_wins=sum(p['difference']<0 for p in valid),ties=sum(p['difference']==0 for p in valid),paired_available=len(valid))
        la=s['overall_equal_layouts'][MODES[0]][field]['mean'];va=s['overall_equal_layouts'][MODES[1]][field]['mean']
        findings[field]=dict(layouts=rows,lidar_equal_layout_mean=la,visual_equal_layout_mean=va,
                            mean_reduction_percent=100*(1-la/va) if la is not None and va else None,
                            layouts_mean_improved=sum(v['mean_reduction_percent'] is not None and v['mean_reduction_percent']>0 for v in rows.values()),
                            lidar_wins=sum(v['lidar_wins'] for v in rows.values()),paired_available=sum(v['paired_available'] for v in rows.values()))
    coverage={k:v['metrics']['coverage'] for k,v in s['trials'].items()}
    findings['coverage']=coverage
    findings['scope']='Descriptive, predeclared 3 unseen synthetic texture layouts x 5 separate-flight repeats per arm. All repeated trials included. Not a test of new geometry, new sensors, or real-world generalization. No statistical significance claim; matched requested seeds are not identical sensor streams.'
    findings['execution_amendment']=m.get('protocol_amendment')
    findings['incomplete_attempts']=m.get('attempt_history',[])
    findings['report_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (out/'findings.json').write_text(json.dumps(findings,indent=2)+'\n')
    def f(x):return '—' if x is None else f'{x:.3f}'
    primary=findings[FIELDS[1]];fusion=findings[FIELDS[-1]]
    text='<section id="holdout-findings"><h2>全30回の比較結果</h2>'
    text+=f'<p>主指標の視覚軸速度誤差は、3配置を等しく重みづけした平均で視覚のみ {f(primary["visual_equal_layout_mean"])} → LiDAR情報あり {f(primary["lidar_equal_layout_mean"])} m/s。配置別の平均が改善したのは {primary["layouts_mean_improved"]}/3配置、対応する要求seedの組では {primary["lidar_wins"]}/{primary["paired_available"]}組。</p>'
    text+=f'<p>融合全位置誤差は、視覚のみ {f(fusion["visual_equal_layout_mean"])} → LiDAR情報あり {f(fusion["lidar_equal_layout_mean"])} m。配置別の平均改善は {fusion["layouts_mean_improved"]}/3配置、各組では {fusion["lidar_wins"]}/{fusion["paired_available"]}組。速度拘束の誤差と融合位置の誤差を区別する。</p>'
    text+='<p>各方式の値は5回の平均 ± 標本標準偏差。各回と中央値は下の表・図に掲載。<a href="findings.json">差・中央値・勝敗の全数値</a>。</p><table><tr><th>配置</th><th>視覚のみ 軸速度[m/s]</th><th>LiDARあり 軸速度[m/s]</th><th>速度改善 組数</th><th>視覚のみ 位置[m]</th><th>LiDARあり 位置[m]</th></tr>'
    for layout in LAYOUTS:
        v=primary['layouts'][str(layout)];q=fusion['layouts'][str(layout)]
        cell=lambda a:f(a['mean'])+' ± '+f(a['std'])
        text+=f'<tr><td>{layout}</td><td>{cell(v["visual"])}</td><td>{cell(v["lidar"])}</td><td>{v["lidar_wins"]}/5</td><td>{cell(q["visual"])}</td><td>{cell(q["lidar"])}</td></tr>'
    text+='</table><p>同じ人工トンネル形状の未使用テクスチャ配置での検証であり、異なる形状・実環境への一般化の試験ではない。3配置を15独立環境とは扱わない。方向の評価にLiDAR情報を使う効果の比較であり、固定カメラとの比較ではない。</p>'
    text+='<p>速度拘束の誤差でLiDAR情報ありが良かったのは15組中7組。3配置すべてで速度誤差の最大値と標本標準偏差が小さく、平均改善には大きな誤差の出た試行の差が効いている。24301の速度誤差中央値は視覚のみ0.291、LiDARあり0.331 m/sで逆順位。毎回の改善や統計的有意とは区別する。</p>'
    text+=f'<p>速度拘束利用率100%は {sum(v==1 for v in coverage.values())}/30本。元の角度指令の連続性判定は、保持動作による不合格を消さず取得検査と別記する。全試行の取得・時刻・設定・終了確認は <a href="final_audit.json">監査記録</a>。</p>'
    text+='<p>初回の未完了試行1本は、補助の上向き固定推定器が不正な速度・共分散を出して中断したため別保存。全30本の比較開始前に両方式とも追加4固定方向推定器を外し、正面参照・純LIOを維持した。6カメラの描画、主比較の推定器・偏向・パラメータは変更していない。未完了試行は精度の悪さを理由に除外したものではなく、主比較が途中で打ち切られた記録として残す。</p>'
    for layout in LAYOUTS:
        shutil.copy2(Path(m['assets'])/str(layout)/'distribution.png',out/f'layout_{layout}.png')
        text+=f'<details><summary>配置{layout}の模様分布（制御への入力には使用しない）</summary><img src="layout_{layout}.png"></details>'
    text+='</section>'
    html=(out/'index.html').read_text();start='<section id="holdout-findings">'
    if start in html:
        left,right=html.split(start,1);html=left+right.split('</section>',1)[1]
    anchor=html.index('</h1>')+len('</h1>');html=html[:anchor]+text+html[anchor:];(out/'index.html').write_text(html)
    print(json.dumps({k:{n:v for n,v in d.items() if n!='layouts'} for k,d in findings.items() if k in FIELDS},indent=2))

if __name__=='__main__':main()
