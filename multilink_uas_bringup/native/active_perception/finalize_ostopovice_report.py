#!/usr/bin/env python3
"""Add overview and reproducible usage to completed numeric results."""
import argparse,json,re,hashlib
from pathlib import Path
import numpy as np
from report_direction_repeats import ORDER,LABEL


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();out=a.root/'report'
    s=json.loads((out/'summary.json').read_text());assert s['status']=='complete'
    n=len(s['trials']);counts={};html='<section id="findings"><h2>結果</h2>'
    for field,title in [('vio_local_10s_axial_rms_m','VIOの10秒軸方向移動誤差 [m]'),('full_fused_position_rmse_m','融合後の全90m位置誤差RMSE [m]')]:
        wins_all=sum(c[field+'_comparison']['active_wins_all_fixed'] for c in s['conditions'].values())
        wins_mean=sum(c[field+'_comparison']['active_wins_fixed_mean'] for c in s['conditions'].values())
        counts[field]=dict(physical_flights=n,active_wins_all_fixed=wins_all,active_wins_fixed_mean=wins_mean)
        html+=f'<h3>{title}</h3><p>各飛行内で偏向が全固定より小さい：{wins_all}/{n}回。固定5方向の誤差平均より小さい：{wins_mean}/{n}回。</p><table><tr><th>配置</th><th>固定5方向平均</th><th>最良固定</th><th>偏向</th></tr>'
        for name,c in s['conditions'].items():
            m=c['methods'];best=c[field+'_comparison']['best_fixed_of_mean'];avg=np.mean([m[v][field]['mean'] for v in ORDER[:5]])
            html+=f"<tr><td>{name}</td><td>{avg:.4f}</td><td>{m[best][field]['mean']:.4f} ({LABEL[best]})</td><td>{m['active'][field]['mean']:.4f}</td></tr>"
        html+='</table>'
    html+='<p>実偏向角を見ると、3配置とも序盤の探索後は主に天井方向（pitch=-90度）を選びました。今回のランダム配置では、最適な観測面が距離に沿って何度も切り替わる動作にはなっていません。</p>'
    html+='<p>最良固定は各指標で試験結果から選んだ参考値です。同じ向きがすべての指標で最良とは限りません。固定平均は5方向を等確率で一つ選ぶ場合の今回の誤差平均であり、実際の設置方向の確率を仮定したものではありません。各表は飛行ごとのRMSを計算した後の平均です。</p><p>融合後の順位だけから、視覚拘束の正確さを判断しないでください。実際の軸速度誤差と10秒移動誤差は下表に別々に掲載しています。これらの比較だけで融合の誤差相殺を因果的に分離したものではありません。</p></section>'
    doc=(out/'index.html').read_text();doc=re.sub(r'<section id="findings">.*?</section>','',doc,flags=re.S);doc=doc.replace('</h1>','</h1>'+html,1)
    marker='<!-- generation-usage -->';doc=doc.split(marker)[0]
    doc+=marker+'''<h2>生成環境と再現方法</h2>
<p>元スキャン由来の断面と、実際に生成した落書き位置です。配置は独立な乱数から生成し、性能結果による選別や調整をしていません。</p>
<img src="../../environment_figures/cross_section.png"><img src="../../environment_figures/placements.png">
<p>幅は元の内部幅中央値を5mと仮定した換算です。実測寸法の復元ではありません。形状は元の内壁を採取・繰返し・接続したもので、凹凸や砂利は残ります。全方向が無特徴な理想トンネルではありません。</p>
<p><a href="../../geometry_information.json">生成形状の方向別拘束</a>：面積で重み付けした面法線の行列では最弱方向はほぼトンネル軸、最小/最大固有値比は約0.010です。センサ配置や推定誤差を含む実行中の情報行列とは別です。</p>
<p>環境だけを表示：</p><pre>ros2 launch multilink_uas_bringup ostopovice.launch.py
# 別配置
ros2 launch multilink_uas_bringup ostopovice.launch.py seed:=24102</pre>
<p>新しい保存先へ再生成（同seedなら同じ模様・配置）：</p>
<pre>python3 /root/uas_native/active_perception/generate_ostopovice_tunnel.py \
  --source /mnt/c/Users/j-sug/Downloads/hitler-highway-tunnel-at-ostopovice \
  --output /root/multilink_autonomy_ws/assets/ostopovice_new \
  --length 100 --width 5 --seeds 24101 24102 24103</pre>
<p>新しい乱数に変更すれば別の配置になります。出力先は既存データを上書きしません。新規出力worldは汎用ビューアのworld_file引数でも開けます。生成器はこのFBXの向きと内壁範囲を使用しており、無関係な別モデル用の汎用形状認識器ではありません。</p>
<p><a href="../../../../active_perception/generate_ostopovice_tunnel.py">生成プログラム</a> / <a href="../../frozen.json">事前固定した条件と入力SHA</a></p>'''
    (out/'index.html').write_text(doc)
    (out/'findings.json').write_text(json.dumps(dict(counts=counts,presentation_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
    print(out/'index.html')


if __name__=='__main__':main()
