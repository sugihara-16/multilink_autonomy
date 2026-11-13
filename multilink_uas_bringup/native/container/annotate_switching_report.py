#!/usr/bin/env python3
"""Display acquisition failures explicitly, alongside complete-run metrics."""
import html
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def csv(path):
    if not path.exists():path=path.with_name(path.name+'.gz')
    return np.loadtxt(path,delimiter=',',skiprows=1,ndmin=2)


def main(root):
    out=root/'report';summary=json.loads((out/'summary.json').read_text());failures=[]
    for name,condition in summary['conditions'].items():
        for key,trial in condition['trials'].items():
            directory=root/name/key
            flight=directory/'flight_result.json'
            record=json.loads(flight.read_text()) if flight.exists() else None
            checks=condition['audit']['trials'][key]
            if checks['passed'] and trial['full_interval_available']:continue
            row=dict(condition=name,trial=key,full_interval=trial['full_interval_available'],
                     failed_checks=[k for k,v in checks['checks'].items() if not v],
                     motion_checks=[k for k,v in checks.get('motion_and_acquisition',{}).items() if not v],
                     flight=record)
            failures.append(row)
            if record and not record['passed'] and 'Lost hover' in str(record['failed']):
                truth=csv(directory/'truth.csv');truth[:,0]*=1e-9
                joints=csv(directory/'joint_measurements.csv')
                events=json.loads((directory/'flight_events.json').read_text())
                start=next(e['time'] for e in events if e['stage']=='active')
                rpy=Rotation.from_quat(truth[:,4:8]).as_euler('xyz')
                fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
                axes[0].plot(truth[:,0]-start,truth[:,3]);axes[0].axhline(2.5,color='k',ls='--',alpha=.4);axes[0].set_ylabel('FC height [m]')
                axes[1].plot(truth[:,0]-start,np.rad2deg(rpy[:,0]),label='roll');axes[1].plot(truth[:,0]-start,np.rad2deg(rpy[:,1]),label='pitch');axes[1].set_ylabel('FC orientation [deg]');axes[1].legend()
                for i in range(1,joints.shape[1]):axes[2].plot(joints[:,0]-start,np.rad2deg(joints[:,i]),label=f'joint {i}')
                axes[2].set_ylabel('Joint angles [deg]');axes[2].legend(ncol=3)
                for ax in axes:ax.grid(alpha=.25)
                axes[-1].set_xlim(max(0,truth[-1,0]-start-60),truth[-1,0]-start)
                axes[-1].set_xlabel('Tracking time [s]')
                fig.suptitle(name+'/'+key+' | interrupted flight, last 60 seconds')
                fig.tight_layout();image=f'{name}_{key}_abort.png';fig.savefig(out/image,dpi=140);plt.close(fig);row['plot']=image
    (out/'failure_details.json').write_text(json.dumps(failures,indent=2))
    marker='<!-- failure details -->';page=(out/'index.html').read_text().split(marker)[0]+marker
    page+='<h2>完走と検査の結果</h2><p>未完走の試行は除外して置き換えていません。3回すべての全区間がそろわない方式・条件は、3回平均を表示しません。</p><ul>'
    for row in failures:
        labels={'clean_shutdown':'終了処理エラー','complete_records':'完走に必要な記録が不足'}
        description=row['flight']['failed'] if row['flight'] and not row['flight']['passed'] else ', '.join(labels.get(k,k) for k in row['motion_checks']+row['failed_checks'])
        if description and 'Lost hover' in description:description='飛行姿勢が崩れ、強制着陸へ移行して中断'
        page+='<li>'+html.escape(row['condition']+'/'+row['trial']+': '+str(description))+'（全区間取得: '+str(row['full_interval'])+'）</li>'
    page+='</ul><p><a href="failure_details.json">失敗と不合格の記録</a></p>'
    page+='<p>ジンバル9試行は採用設定が一致し、入力欠落0件、距離補正23,580回すべて成立。正順の関節3回目は74.43mの誤差を除外せず平均に含めています。<a href="../diagnostics/all_gimbal_checks.json">ジンバルの追加点検</a>。視線の図は機体に対するカメラの向きで、関節版は実測関節角と機体寸法から復元しています。</p>'
    for row in failures:
        if 'plot' in row:page+=f'<img src="{row["plot"]}"><p>姿勢は機体の慣性センサー位置での実姿勢。内部の制御座標系や安全判定値そのものではありません。</p>'
    (out/'index.html').write_text(page)


if __name__=='__main__':main(Path(sys.argv[1]))
