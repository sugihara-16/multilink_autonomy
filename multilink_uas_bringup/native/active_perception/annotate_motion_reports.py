#!/usr/bin/env python3
"""Correct generated report descriptions without changing frozen numerical code."""
import argparse,html,json,re
from pathlib import Path
import yaml

def annotate(root):
    m=json.loads((root/'campaign.json').read_text())
    if m['status']!='complete':raise ValueError('Annotate only completed campaigns')
    files=list((root/'implementation').glob('motion_view_v*.yaml'))
    if len(files)!=1:raise ValueError('Exactly one executed candidate config required')
    cfg=yaml.safe_load(files[0].read_text())
    done=sum(x['status']=='complete' for x in m['runs'].values())
    description=('現在のROVIO追跡点と速度の同時不確かさを使う視野評価。独立の画像特徴地図の未表示点は検索しません。' if cfg.get('fixation') else '画像特徴地図から将来の相対運動の情報を予測する視野評価。')
    if cfg.get('fixation_retinal_candidates'):description+='画像内の特徴移動を抑える候補を追加。'
    if cfg.get('committed_track_recovery'):description+='追跡点不足が続いたら正面復帰を完了し、継続追跡点の回復を確認してから視野選択へ戻る。'
    if cfg.get('common_preparation_cycles',1)==2:description+='全固定と偏向に共通の準備運動を26秒追加し、2回目だけ偏向の実角度を保持。'
    if cfg.get('preview_preparation'):description+='さらに既存経路の冒頭を最大15m往復し、戻り中も偏向視野を保持。固定にも共通の身体運動です。'
    description+=f"完了{done}飛行、段階は{m['stage']}、実行設定は{files[0].name}。各固定方向と偏向は独立した単眼ROVIO＋MIMOSAで、DRPM OFF、推定器・センサ・融合重みは共通です。全方向と失敗結果を保持しています。"
    replacement='<p id="campaign-description">'+html.escape(description)+'</p>'
    p=root/'report/index.html';text=p.read_text()
    pattern=r'<p id="campaign-description">.*?</p>' if 'id="campaign-description"' in text else r'<p>(?:ZaB a1/b1・二ツ小屋、各3新規飛行。|本ページは偏向手法改良の).*?</p>'
    text,count=re.subn(pattern,lambda _:replacement,text,count=1,flags=re.S)
    if count!=1:raise ValueError('Unexpected generated report description')
    text=text.replace('7区間×5方向の探索結果です。','記載した全区間・全方向の結果です。').replace('他の2試行だけで選ぶ評価','他の試行だけで選ぶ評価').replace('真値は採点にだけ使用。','真値は視野選択や推定へ入力せず、共通の飛行制御と事後採点に使用。')
    p.write_text(text)
    (root/'report/description_annotation.json').write_text(json.dumps(dict(config=files[0].name,stage=m['stage'],description=description,numerical_data_changed=False),ensure_ascii=False,indent=2)+'\n')
    print(p)

def main():
    p=argparse.ArgumentParser();p.add_argument('campaigns',nargs='+',type=Path);a=p.parse_args()
    for root in a.campaigns:annotate(root)
if __name__=='__main__':main()
