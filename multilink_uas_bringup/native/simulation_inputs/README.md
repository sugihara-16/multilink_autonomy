# 新規シミュレーション用の入力

過去の `logs/` を削除しても新しいオンライン実験を実行できるよう、
2026-10-20に必要な入力を切り離した。

- `zab_catalog.json`: 14経路の軌道・world・spawn・評価終了時刻。高誤差経路の事前選定値も保持。
- `routes/`: 取得時の経路定義とworld設定。点群・画像・推定軌跡ではない。
- `paired_routes/`: 採用UniPilot比較で使用していた3経路の定義。
- `calibration/`: 採用融合校正値。内容は移動前と同一。
- `templates/`: 過去の測定結果から独立させた起動引数。
- `relocation_manifest.json`: 退避元、退避先、変更していない入力のSHA-256。
- `selected_gimbal_before_relocation.json`: 移動前の採用設定の履歴。ここに残る旧ログパスは由来の記録であり、現在の起動処理では読み込まない。

Gazeboのworld・モデルは従来どおりワークスペースの `assets/`、
実行ファイルは `build/` 等に置かれる。このディレクトリ単独で
別PCへ環境全体を移せるという意味ではない。

採用した偏向方式・推定器・数値設定は変更していない。
`selected_gimbal.json` は校正ファイルのパスと、入力読み込みを移した
`unipilot_campaign.py` のハッシュを更新した。高誤差9経路の新規比較は、
過去のprefix測定結果を再利用せず、prefixも新たに実行する。

## 起動確認

```bash
python3 /root/uas_native/active_perception/test_simulation_inputs.py
python3 /root/uas_native/active_perception/run_selected_gimbal.py \
  --output /root/uas_native/logs/new-selected-run --dry-run
```

実行する場合は `--dry-run` を除く。`--labels run1_tail_a1` で1経路を指定できる。
新規出力先を使い、終了は起動した端末からCtrl+Cで行う。

追加3条件・固定5方向の設定確認:

```bash
python3 /root/uas_native/active_perception/run_zab_additional_conditions.py \
  --output /root/uas_native/logs/new-zab-conditions --prepare-only
python3 /root/uas_native/active_perception/run_view_array_campaign.py \
  --output /root/uas_native/logs/new-fixed-five --prepare-only
```

実行する場合は `--prepare-only` を除く。
過去のrosbagを使うオフライン解析・過去のプロット再生成には、
Driveへ移した該当測定データの復元が必要。
