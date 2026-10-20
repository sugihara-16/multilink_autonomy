#!/usr/bin/env python3
"""Build the short, same-flight directional/crossover diagnostic report."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_routes import load


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root',type=Path)
    a = p.parse_args()
    root = a.root
    tests = {key:json.loads((root/key/'diagnosis.json').read_text())
             for key in ['short120','covariance120']}
    assert all(t['valid'] for t in tests.values())
    names = dict(fixed_mimosa='Fixed camera',active_mimosa='Gimbal',
        pure_lio='LIO only',fixed_velocity_active_tilt='Fixed velocity/cov + gimbal tilt',
        active_velocity_fixed_tilt='Gimbal velocity/cov + fixed tilt',
        active_velocity_fixed_cov='Gimbal velocity + fixed covariance',
        fixed_velocity_active_cov='Fixed velocity + gimbal covariance')
    colors = dict(fixed_mimosa='#2474b5',active_mimosa='#df6619',
        pure_lio='#444444',fixed_velocity_active_tilt='#269659',
        active_velocity_fixed_tilt='#a24faa',
        active_velocity_fixed_cov='#a24faa',fixed_velocity_active_cov='#997000')
    curves = {}
    for test in tests:
        curves[test] = {p.name.removesuffix('_errors.csv'):None
                        for p in (root/test).glob('*_errors.csv')}
    # Explicit field access avoids relying on the CSV reader's column order.
    for test in curves:
        for key in curves[test]:
            d=load(root/test/(key+'_errors.csv'))
            curves[test][key]=np.column_stack([d[k] for k in
                ['route_t','stamp_ns','x_m','y_m','z_m','roll_deg','pitch_deg','yaw_deg']])
    with PdfPages(root/'diagnosis.pdf') as pdf:
        fig,axes=plt.subplots(3,1,figsize=(11,10),sharex=True)
        for key in ['pure_lio','fixed_mimosa','active_mimosa','fixed_velocity_active_tilt','active_velocity_fixed_tilt']:
            d=curves['short120'][key]
            axes[0].plot(d[:,0],np.linalg.norm(d[:,2:5],axis=1),label=names[key],c=colors[key])
        axes[0].set_ylabel('Position error [m]');axes[0].legend(fontsize=8)
        with gzip.open(root/'short120/geometry_scores.json.gz','rt') as stream:
            records=json.load(stream)['rows']
        records=[r for r in records if r['lidar_source']=='pure']
        start=tests['short120']['initial_sim_s']
        for side,color in [('fixed','#2474b5'),('active','#df6619')]:
            axes[1].plot([r['stamp_ns']*1e-9-start for r in records],
                [r[side]['correlated']['weak_log_gain'] for r in records],
                label=side,c=color,lw=.9)
        axes[1].set_ylabel('Weak-direction information gain\n(local geometry proxy)')
        axes[1].legend()
        encoder=load(root/'short120/campaign/run2a_prefix_active/encoders.csv')
        # Inspect supported encoder schema explicitly.
        tk='stamp_ns' if 'stamp_ns' in encoder else 't'
        ts=encoder[tk]*(1e-9 if tk=='stamp_ns' else 1)-start
        pitch=encoder['pitch']
        axes[2].plot(ts,np.rad2deg(pitch),color='#df6619')
        axes[2].set_ylabel('Actual gimbal pitch [deg]')
        axes[2].set_xlabel('Trajectory time [s]')
        for ax in axes:ax.grid(alpha=.3);ax.set_xlim(0,120)
        fig.suptitle('Trial 1: geometry favors gimbal, but fixed fusion is more accurate')
        fig.tight_layout();fig.savefig(root/'geometry.png',dpi=140);pdf.savefig(fig);plt.close(fig)
        fig,ax=plt.subplots(figsize=(11,5))
        for key in ['active_mimosa','fixed_velocity_active_tilt','active_velocity_fixed_cov','fixed_velocity_active_cov']:
            d=curves['covariance120'][key]
            ax.plot(d[:,0],np.linalg.norm(d[:,2:5],axis=1),label=names[key],color=colors[key])
        ax.set(xlabel='Trajectory time [s]',ylabel='Position error [m]',
               title='Trial 2: velocity / covariance crossover, identical gimbal tilt')
        ax.legend(fontsize=9);ax.grid(alpha=.3)
        fig.tight_layout();fig.savefig(root/'crossover.png',dpi=140);pdf.savefig(fig);plt.close(fig)
        for test,keys in [
            ('short120',['pure_lio','fixed_mimosa','active_mimosa','fixed_velocity_active_tilt','active_velocity_fixed_tilt']),
            ('covariance120',['active_mimosa','fixed_velocity_active_tilt','active_velocity_fixed_cov','fixed_velocity_active_cov'])]:
            fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
            for key in keys:
                d=curves[test][key]
                for i,ax in enumerate([*axes[:,0],*axes[:,1]]):
                    ax.plot(d[:,0],d[:,2+i],label=names[key],color=colors[key],lw=.85)
                    ax.set_ylabel(['x [m]','y [m]','z [m]','roll [deg]','pitch [deg]','yaw [deg]'][i])
                    ax.grid(alpha=.3)
            axes[0,0].legend(fontsize=7)
            axes[-1,0].set_xlabel('Trajectory time [s]');axes[-1,1].set_xlabel('Trajectory time [s]')
            fig.suptitle(test+': full six-axis error, no omitted y-axis interval')
            fig.tight_layout();fig.savefig(root/(test+'_six_axes.png'),dpi=140)
            pdf.savefig(fig);plt.close(fig)
    first,second=tests['short120'],tests['covariance120']
    summary=dict(
        conclusion='In the first 120 s, current direction-sensitive geometry does not support fixed-view superiority. Crossover experiments identify velocity values and their covariance as influential. Signed errors and equal-covariance crossover support bias cancellation; this is not proof of the full 240 s cause.',
        trials=tests,
        limitations=[
            'Two fresh physical flights, same requested seed but not identical realized observations; compare interventions within a flight.',
            'Only first 120 s of original 240 s. A second trial reversed fixed/gimbal ranking.',
            'Feature depths are ROVIO estimates. Information is a conditional geometry proxy, not measured true depth or the estimator posterior.',
            'Diagnostic variants do not control the active camera. Crossover covariances are interventions, not valid deployable sensor models.',
            'The first trial verified declared library hashes and exact duplicate trajectory, but did not separately record loaded-library paths. Second trial did.',
            'No parameter tuning or new adopted controller. Shared IMU correlations and nonlinear map feedback were not isolated.'])
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    def metric(d,k):return d['metrics'][k]['position_rmse_m']
    rows=''.join(f'<tr><td>{label}</td><td>{metric(first,key):.3f}</td></tr>' for key,label in [
        ('pure_lio','MIMOSA単体'),('fixed_mimosa','固定カメラ融合'),('active_mimosa','偏向カメラ融合'),
        ('fixed_velocity_active_tilt','固定の速度・信頼度 ＋ 偏向の傾き'),
        ('active_velocity_fixed_tilt','偏向の速度・信頼度 ＋ 固定の傾き')])
    cross=''.join(f'<tr><td>{v}</td><td>{c}</td><td>{metric(second,key):.3f}</td></tr>' for key,v,c in [
        ('fixed_velocity_active_tilt','固定','固定'),
        ('fixed_velocity_active_cov','固定','偏向'),
        ('active_velocity_fixed_cov','偏向','固定'),
        ('active_mimosa','偏向','偏向')])
    page=f'''<!doctype html><meta charset="utf-8"><title>run2a_prefix：偏向悪化の原因調査</title>
<style>body{{font:17px sans-serif;line-height:1.75;max-width:1100px;margin:30px auto;padding:0 20px}}
table{{border-collapse:collapse}}td,th{{border:1px solid #bbb;padding:8px 14px}}img{{width:100%}}li{{margin:.4em 0}}</style>
<h1>run2a_prefix：偏向悪化の原因調査</h1>
<p><strong>最初の120秒では、固定画角の方向別拘束が優れていたという説明を支持する結果は得られませんでした。
速度誤差の相殺と、速度の信頼度による融合重みの変化を示す証拠が強く出ています。</strong></p>
<p>元の240秒の前半を2回飛行。各回で固定・偏向と診断用推定器が同じLiDAR・IMUを共有しました。
各回内の入力・因子交換・通常推定器複製の厳密一致検査はすべて合格。DRPM OFF、採用81ファイルは変更していません。
2回の実観測は同一ではなく、別飛行をまたいだ数値差を介入の効果とは扱いません。</p>
<h2>1回目：傾きより、速度入力の違いが結果を左右した</h2>
<table><tr><th>構成</th><th>位置RMSE [m]</th></tr>{rows}</table>
<p>RMSEは全評価時刻の誤差の二乗平均平方根。始点の位置・姿勢だけ整合し、尺度補正なし。
固定ROVIOの機体前方速度は平均 +0.569m/s、偏向は +0.073m/s の誤差。
MIMOSA単体の終端誤差は進行方向に −29.58m、固定融合 −2.03m、偏向融合 −4.70m。
固定側の速度過大評価が、LiDAR側の過小な移動推定を打ち消す向きです。</p>
<h2>同じLiDARの弱い方向に対する、特徴点配置の比較</h2>
<p>固定・偏向の同時刻の追跡点を、同じLiDAR情報行列と事前条件で評価しました。
特徴点間の誤差の関連まで含めた局所指標では、239時刻中226時刻（94.6%）で偏向側が優勢でした。
同じ弱い並進方向に投影した速度誤差も、固定0.192m/s、偏向0.022m/sでした。
点を完全に既知とする計算、点ごとの不確かさを入れた計算、偏向側LiDAR地図を使った計算でも、偏向優勢の傾向は変わりません。</p>
<p>この指標はROVIOが推定した距離と点配置に依存し、真の特徴点位置の測定ではありません。
元の情報行列による偏向計画も局所的な予測であり、MIMOSAが実際に使う速度・傾きの拘束、
持続する誤差、地図更新まで含めた最終精度そのものではありません。</p>
<p>採用中の偏向計画は、今回の両飛行でLiDARから直接得た特徴点距離が0件でした。
画像を異なる位置から見た差とMIMOSAの移動推定から距離を計算しており、判断時の有効点数は平均約130点／126点です。
この距離計算は機体移動の誤差の影響を受けるため、地図上の予測拘束が正しいかは追加の確認対象です。
ここまでの交換実験はこの距離計算を変更しておらず、それを悪化原因と特定したものではありません。</p>
<img src="geometry.png">
<h2>2回目：速度値と信頼度を別々に交換</h2>
<p>以下はすべて偏向側の傾きを使用。信頼度はROVIOが出す速度の共分散で、値が小さいほど強く融合されます。
交換は原因を切り分ける実験であり、そのまま採用できるセンサモデルではありません。</p>
<table><tr><th>速度値の出所</th><th>信頼度の出所</th><th>位置RMSE [m]</th></tr>{cross}</table>
<p>この飛行でも固定の速度誤差は大きく、前方誤差は固定 +0.228m/s、偏向 +0.118m/s。
それでも同じ信頼度で比較すると固定速度を使う方が融合誤差は小さくなりました。
信頼度だけを交換しても結果は大きく変化します。方向別の視覚情報量だけでは、この結果を説明できません。</p>
<p>通常の固定融合はこの回では{metric(second,'fixed_mimosa'):.3f}m、偏向融合は{metric(second,'active_mimosa'):.3f}mとなり、1回目と順位が逆転しました。
この短区間でも固定優位は安定していません。</p>
<img src="crossover.png">
<h2>判定と残る範囲</h2>
<ul><li>固定画角がLiDARの弱い方向をより強く拘束した：今回の方向別指標では支持されません。</li>
<li>偏向による傾き推定の悪化：1回目の交換試験では、むしろ偏向の傾きを使う方が良好でした。</li>
<li>速度の系統誤差の相殺と融合重み：符号の一致と速度／信頼度交換が支持しています。</li>
<li>欠落や別入力による比較の崩れ：全区間の入力・因子・出力検査に合格し、通常推定器の複製も厳密一致。</li>
<li>共通IMUの誤差の関連、地図更新の非線形な影響：今回単独には切り分けていません。</li></ul>
<p>元の240秒後半と、真の特徴点距離を使った検証は未実施です。
今回の証拠から全経路での原因を断定したり、融合重みを調整して採用したりはしていません。</p>
<p><a href="diagnosis.pdf">図・六軸誤差PDF（4ページ）</a> /
<a href="summary.json">全数値・監査・制約</a> /
<a href="short120/diagnosis.json">1回目の検証</a> /
<a href="covariance120/diagnosis.json">2回目の検証</a></p>
<img src="short120_six_axes.png"><img src="covariance120_six_axes.png">
'''
    (root/'index.html').write_text(page)
    print(root/'index.html')


if __name__=='__main__':
    main()
