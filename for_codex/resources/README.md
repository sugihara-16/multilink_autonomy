# LiDAR退化を含むLIO・LIVOの比較候補

調査日: 2026-10-04。指定4論文の方式・実験条件を確認し、追加候補の一次資料と公開実装を調査した。以下は文献とソースに基づく選定であり、今回のZaBデータで各方式の改善を実測した結果ではない。

**指定候補は研究上妥当。ただしDRPMは単独のLIOではなく組込み部品であり、Switch-SLAMとSA-LIVOには公式実装の入手性という課題がある。** まず同じ推定器で視覚・退化対策を有効／無効にした比較を置き、その上で別方式を比較する。追加候補ではFAST-LIO2、LVI-SAM、X-ICP系の公開比較実装を優先する。

## 保存した論文

既存のFAST-LIVO2とSwitch-SLAMの出版社版PDFはそのまま保持した。他はarXivの版番号を固定して取得した。各ファイルの取得元・ページ数・SHA-256は[manifest.json](manifest.json)に記録する。追加論文は比較候補または適用条件の参考資料であり、すべての実装を導入する提案ではない。

Gitには調査メモと出典・ハッシュの一覧を保存する。PDF本体と抽出テキストはローカル資料として保持し、コミット対象から除外する。別のチェックアウトでは、各一覧に記載した取得元から必要なPDFを用意する。

| 区分 | 論文・ローカルPDF | 版／位置づけ | 一次資料 |
|---|---|---|---|
| 指定 | [FAST-LIVO2](FAST-LIVO2_Fast_Direct_LiDARInertialVisual_Odometry.pdf) | IEEE T-RO 41, 2025。既存出版社版 | [DOI](https://doi.org/10.1109/TRO.2024.3502198)・[公式コード](https://github.com/hku-mars/FAST-LIVO2) |
| 指定 | [Switch-SLAM](Switch-SLAM_Switching-Based_LiDAR-Inertial-Visual_SLAM_for_Degenerate_Environments.pdf) | IEEE RA-L 9(8), 2024。既存出版社版 | [DOI](https://doi.org/10.1109/LRA.2024.3421792) |
| 指定 | [SA-LIVO](SA-LIVO_Subspace-Aware_Degeneracy_Handling_arXiv2606.25699v2.pdf) | arXiv v2, 2026-08-06。採録確認なし | [論文](https://arxiv.org/abs/2606.25699v2) |
| 指定 | [Probabilistic Degeneracy Detection / DRPM](DRPM_Probabilistic_Degeneracy_Detection_arXiv2410.10784v2.pdf) | arXiv v2, 2025-02-03。RA-L採録済み原稿 | [論文](https://arxiv.org/abs/2410.10784v2)・[公式コード](https://github.com/ntnu-arl/drpm) |
| 追加 | [FAST-LIO2](FAST-LIO2_arXiv2107.06829v1.pdf) | arXiv v1。標準的なLIO比較対象 | [論文](https://arxiv.org/abs/2107.06829v1)・[公式コード](https://github.com/hku-mars/FAST_LIO) |
| 追加 | [LVI-SAM](LVI-SAM_arXiv2104.10831v2.pdf) | arXiv v2。ICRA 2021 | [論文](https://arxiv.org/abs/2104.10831v2)・[公式コード](https://github.com/TixiaoShan/LVI-SAM) |
| 追加 | [X-ICP](X-ICP_arXiv2211.16335v4.pdf) | arXiv v4。T-RO 2024 | [論文](https://arxiv.org/abs/2211.16335v4)・[著者ページ](https://sites.google.com/leggedrobotics.com/x-icp) |
| 追加 | [Informed, Constrained, Aligned](Informed_Constrained_Aligned_arXiv2408.11809v3.pdf) | arXiv v3。退化対策の比較研究 | [論文](https://arxiv.org/abs/2408.11809v3)・[公式コード](https://github.com/leggedrobotics/perfectlyconstrained) |
| 追加 | [GenZ-ICP](GenZ-ICP_arXiv2411.06766v1.pdf) | arXiv v1。RA-L 2025。IMUを使わないLO | [論文](https://arxiv.org/abs/2411.06766v1)・[公式コード](https://github.com/cocel-postech/genz-icp) |
| 参考 | [RELEAD](RELEAD_arXiv2402.18934v2.pdf) | arXiv v2。ICRA 2024 | [論文](https://arxiv.org/abs/2402.18934v2)・[著者ページ](https://thisparticle.github.io/) |
| 参考 | [COIN-LIO](COIN-LIO_arXiv2310.01235v4.pdf) | arXiv v4。ICRA 2024 | [論文](https://arxiv.org/abs/2310.01235v4)・[公式コード](https://github.com/ethz-asl/COIN-LIO) |
| 参考 | [R3LIVE](R3LIVE_arXiv2109.07982v1.pdf) | arXiv v1。ICRA 2022 | [論文](https://arxiv.org/abs/2109.07982v1)・[公式コード](https://github.com/hku-mars/r3live) |
| 参考 | [GenZ-LIO](GenZ-LIO_arXiv2603.16273v2.pdf) | arXiv v2, 2026-06-15。コード公開予定 | [論文](https://arxiv.org/abs/2603.16273v2) |

## 何を切り分ける比較か

LIOはLiDARとIMUによる運動推定、LIVOはそこにカメラ画像を加えた運動推定。ここでの「退化」は、観測から位置・姿勢の一部の方向を決めにくくなることを指す。例えば、長く同じ断面が続くトンネルでは、前後に移動してもLiDARで見える形があまり変わらない。

比較には二つの狙いがある。

1. **誤ったLiDAR補正を抑える効果。** 形から決められない方向へ無理に位置合わせすると、雑音や誤対応で推定が壊れる。その補正を抑えることで発散を防げるか。
2. **カメラから不足する情報を補う効果。** 形が同じでも、壁の模様が追えれば前後移動を推定できる。その情報を融合すると誤差を減らせるか。

退化対策だけでは、全センサーに存在しない情報を新たに作れない。発散を防ぐこと、移動量を正しく推定すること、長時間のドリフトをなくすことは別々に評価する。

今回の既存結果はMIMOSAの特定設定での失敗を示す。run2a_endでは長手方向の情報不足を強く支持する観測があるが、姿勢破綻やrun2bの巨大な発散まで、環境だけが原因と確定したわけではない。VIOではセンサーの誤差モデルを合わせるだけで改善したため、LIO側も設定整合を確認してから方式の優劣を判断する。

## 指定4候補の評価

### FAST-LIVO2 — 通常LIVOの主比較対象に採用

LiDARで測った3D点と、その点を観測した画像の小領域を同じ地図に持つ。IMUで予測した位置・姿勢をLiDAR、画像の順で更新する。画像の明るさの変化やLiDAR近傍の死角も扱う。方式はIV–VII節、退化環境の実験はIX-C節、長距離ドリフトの制約はXI節に記載。

**「退化に対する工夫が何もない方式」ではない。** 論文自体がLiDAR／視覚の退化への耐性を評価している。今回の分類は「退化方向を明示的に判定して補正量を制限する仕組みを主題としないLIVO基準」が正確。これだけで改善するなら、特別な退化判定なしでも視覚との融合で対処できる場面がある、と判断できる。

公式ROS1実装あり。確認した`LIVMapper.cpp`は画像入力1系統・カメラ外部校正1組で、現状の3カメラをそのまま全部入力できる構造ではない。`img_en`による画像なしのLIO動作も実装されているため、**同じFAST-LIVO2の画像あり／なし**を比較すると視覚の寄与を調べやすい。このLIO動作を別プロジェクトのFAST-LIO2と同一とは扱わない。根拠: [公式ソース](https://github.com/hku-mars/FAST-LIVO2/blob/main/src/LIVMapper.cpp)。

### Switch-SLAM — 目的に合うが、公式コード入手が課題

LiDARとVIOを並行して動かし、LiDARの退化判定に応じて位置合わせの初期予測を切り替える。切り替え境界では補間して急変を抑える。さらに、LiDARの弱い方向を除いた補正と視覚推定を組み合わせる。**最終odometryを単純に二者択一するだけの方式ではない。** III-C〜E節、式(11)〜(16)、Algorithm 1が該当する。

正規化した小さい側の3固有値から判定し、VIO失敗時はLiDAR側へ戻す。論文は環境ごとに調整しない判定を提案するが、固定の統計的仮定、状態履歴、VIO失敗判定の条件は残る。「全くパラメーター不要」とは解釈しない。

Gazeboを含む実験、長い通路、LiDAR／視覚それぞれの失敗がある場面を評価しており、今回に近い。実験は単眼のVINS-Mono系で、GNSSとループ閉じ込みは無効（IV-A節）。論文・著者の公開情報・リポジトリ検索では、著者公式の実行可能コードを確認できなかった。存在しないと断定するものではない。自作する場合は「Switch-SLAM再実装」と明記し、単なるMIMOSA/ROVIO出力切り替えを同論文の再現とは呼ばない。

### SA-LIVO — 方向別の融合を比較する有力候補、再現性確認を先行

LiDARと画像が与える位置・姿勢の情報を合計し、6次元の「どの方向が強く／弱く決まるか」を調べる。両方を合わせても弱い方向は補正を連続的に弱め、IMU予測への依存を増やす。画像がLiDARの不足方向を補えれば、その方向の情報を利用できる。入力の誤対応を除く処理と画像全体の品質判定も別に持つ。v2 VII節、式(31)〜(38)が該当する。

**LiDARの弱い方向だけへ画像を投影する方式、と単純化しない。** v2はLiDARと視覚を合わせた情報の方向を判定する。大きな情報量を持つ誤対応は、この判定だけでは除外できないと論文自身が明記している。弱い幾何形状と、誤った地図対応・画像対応は区別する必要がある。

29系列の評価と構成要素を外した比較がある。FAST-LIVO2に常に勝つという結果ではなく、例えばTable IIのHILTI exp07では同じ0.044 m。比較上の価値は順位より、方向別処理の有無で破綻の仕方が変わるかにある。判定の閾値は座標の単位に依存し、全状態の推定可能性や永続的な安定性を保証するものではない（VII節）。

2026-08-06版では公式コードを採録後公開予定と記載。採録と公式実装公開は確認できない。見つかった[第三者実装](https://github.com/Huashuijingying/SA-LIVO-Reimpl)は、自らv1/v2混在・独自拡張・系列別設定を説明している。これをそのまま「公式SA-LIVOの性能」として比較しない。採用するなら、公式コード取得または論文との差分を固定した再実装の監査が必要。

### DRPM（arXiv:2410.10784）— 適切な退化対策部品。ただし純LIOとして別途評価

正式題名は *Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization*。点の位置と面の向きの誤差から、位置合わせに見えている情報が雑音由来かを確率で判定し、弱い方向の更新を連続的に抑える。点数・環境の大きさ・観測の重みの影響を考慮する点が、固定閾値だけの判定との主な違い。III節、Algorithm 1が該当する。

公式公開物は`degeneracy.h`と使用例であり、完成したLIOパッケージではない。点群位置合わせ・IMU推定・地図更新のどこへ反映するかと、結果の不確かさを後段へどう渡すかを実装する必要がある。根拠: [公式リポジトリ](https://github.com/ntnu-arl/drpm)。

**論文の実験は「LiDAR＋IMUのみ」の性能証明ではない。** IV節で次の補助入力を使う。

| 実験 | 位置合わせを補助する情報 |
|---|---|
| 建設現場 | 脚の動きから得るodometry |
| Seemühle鉱山 | ROVIOによるVIO |
| 円筒タンク | 全視野LiDARで別途得た軌道に雑音を加えた外部odometry |
| 500 mトンネル | レーダー速度。さらに地面からroll・pitch・高さを拘束 |

従って今回は「共通LIO＋DRPM」として、VIO・レーダー・真値等を入力しない条件で新たに評価する。実験1・3・4は真値がなく、定量的な真値比較は実験2である点も、論文の実証範囲として押さえる。

## 追加する価値がある手法

| 優先度 | 候補 | 比較する理由／適用条件 |
|---|---|---|
| 最優先の対照条件 | 現在のMIMOSAの退化対策OFF／ON | 同じ地図・IMU・点群処理で、退化対策だけの効果を測れる。DRPMの再現とは別条件 |
| 高 | FAST-LIO2 | 公開実装がある独立したLIO基準。今回の失敗がMIMOSA固有か、他のLIOでも起きるかを切り分ける |
| 高 | X-ICP系＋*Informed, Constrained, Aligned*の等式制約／弱方向を除く解法 | 方向ごとの判定と補正制限を比較できる。後続研究の公式`perfectlyconstrained`にはX-ICPの判定と複数の対策が含まれる。登録処理の比較環境であり完成LIOではないため、共通IMU予測で比較する |
| 中〜高 | LVI-SAM | 公式コードがあり、LiDAR側・視覚側の失敗を扱う融合の比較対象。Switch-SLAMを直ちに再現できない場合にも有用。ループ閉じ込みを無効にし、odometryとして比較する |
| 中 | GenZ-ICP | 点と面、点と点の距離を環境に応じて組み合わせる。弱い方向を止める方法とは違う対策を検証でき、公式ROS2実装もある。**IMUを使わないLO**として別枠に置く |
| 条件付き | RELEAD / RELEAD-LIO | 退化方向を制限したLiDAR-IMU更新と、外れたVIO等を抑える融合を比較できる。論文はLIO単独と融合を区別して実験する。著者公式の実行可能コードは今回確認できず、再実装コストがある |
| 条件付き | COIN-LIO | カメラなしでもLiDARの反射強度の模様で形状不足を補うため、研究上有力。現bagに有効な強度模様があるか未検証で、距離だけの模擬点群では効果を評価できない |
| 補助 | R3LIVE | 公開された別の直接法LIVO。通常LIVOを複数比較するときに追加。最小構成ではFAST-LIVO2を優先 |
| 継続調査 | GenZ-LIO | GenZ-ICP系の混合距離と環境スケール適応をLIOへ展開。純LIO候補として有力だがv2ではコード公開予定。GenZ-ICPの公開コードと取り違えない |

X-ICPの原論文と後続比較研究は同一実装・同一実験ではない。後続の[公式実装](https://github.com/leggedrobotics/perfectlyconstrained)で再現した条件は、その研究と設定名も併記する。単に固有値で方向を削るだけの処理をX-ICPの全再現とは呼ばない。

COIN-LIOは画像状の反射強度を使うが、カメラを追加しないLIOである。反射強度を使える別データへ進むなら優先度を上げる。GenZ-ICPの点対点制約も、完全に繰り返す形状から真の絶対移動を必ず回復できる保証ではない。

## このワークスペースで確認した適用条件

- **MIMOSAの既存退化対策はOFF。** `native/mimosa_dragon.yaml`に`project_on_degneneracy: false`、`degen_thresh_trans: 40`、`degen_thresh_rot: 0`がある。現ソースの`getProjectionMatrix()`は閾値で方向を選び、`geometric_factor.hpp`で弱い寄与を投影する。DRPMの確率による連続減衰とは異なる。再確認した有効コードは、弱い点の並進成分にだけ投影を適用する。回転用の投影行列も計算するが、回転成分へ適用する処理はコメントアウトされている。回転側閾値0という設定だけの問題ではなく、六軸すべての退化補正が実装済みとは扱わない。ONにするだけで姿勢破綻まで保護できると仮定しない。閾値は調整用区間で固定し、評価区間ごとに真値を見て変えない。
- **3カメラ対応は別の実装課題。** 現在のROVIOは3台を使う。FAST-LIVO2の確認した公式コードは1台。Switch-SLAMの論文実験も1台。カメラ台数・向きが違う結果を、融合方式だけの差とは扱わない。固定した共通1台での方式比較と、3台を使ったシステム比較を分ける案が妥当。実際の使用台数は導入時に明示し、無断で現在の3台構成を変更しない。
- **LiDARとカメラの重なる視野を確認する。** FAST-LIVO2/SA-LIVOはLiDAR地図点に画像を対応付ける。LiDAR水平360度でも上下方向の視野が一致するとは限らない。現ZaB設定は360×32点、10 Hz、最大30 m、垂直−7〜52度。3カメラごとに有効な対応点数を測る必要がある。
- **スキャン内時刻を捏造しない。** `native/slam_inputs.py`はGazeboの瞬時スキャンとして全点の相対時刻を0にする。新しい推定器の入力変換でも、実在しない回転走査時刻を作って不要な運動補正をかけない。元のフレーム時刻は整数nsで保持する。
- **ノイズモデルはセンサー生成条件に合わせる。** 各ソフトの分散・標準偏差・連続時間密度の意味を確認して変換する。ROVIOの設定数値を別推定器へそのままコピーしない。真値で位置・姿勢・バイアスを補正する診断条件は本比較から除く。
- **公式ROS1実装でも導入不能ではない。** このWSには既存のROS1推定器実行環境がある。ただし今回の調査では、追加方式のビルド・起動・再生適合性を検証していない。

## 推奨する実験の順序

1. 現行MIMOSAを再現し、センサー／IMU設定の整合を確認。旧結果を保持したまま、共通設定で退化対策OFF／ONを比較する。
2. FAST-LIVO2の画像なし／ありを比較。独立した基準としてFAST-LIO2を追加する。これで「視覚を足す効果」と「LIO実装を替える効果」を分ける。
3. 共通のLiDAR・IMU推定系へDRPMを導入し、そのOFF／ONを比較する。比較を広げる場合はX-ICP系の方向制限、またはGenZ-ICPの別方式を追加する。
4. Switch-SLAM／SA-LIVOは公式実装取得または再実装の整合確認後に追加する。すぐ比較を進めるなら、公開LVI-SAMを先行候補にする。

各飛行の初期化・離陸・向き合わせは推定器に入力し、誤差の集計対象だけを元と同じ軌道追従区間に限定する。同じ飛行のbag分割境界で推定器を初期化しない。元run識別子とtimestampの組を保持し、ループ閉じ込み・GNSS・真値による補助は無効にする。

評価では従来の六軸誤差に加え、完走率、発散した時刻、前後方向の移動不足、短区間の移動誤差、退化判定／抑制量、画像の有効対応数、実行時間を記録する。推定器が停止した区間を除いて平均誤差だけを比較しない。飛行開始の共通時刻で一度だけ座標を合わせ、全軌道を使った後合わせでドリフトを隠さない。

## 調査の限界

未確認なのは、追加推定器のこのWSでの実行適合性、ZaBでの精度、COIN-LIOに使える反射強度、公開予定コードの実際の入手時期。論文の図や著者報告値から、今回の全発散が解決すると予測することはできない。既存結果・推定設定・センサーデータは今回変更していない。


## 2026-10-05 公式DRPM計算部の導入・比較結果

[公式更新とセンサ誤差近似の比較](DRPM_OFFICIAL_COMPARISON_20261005.md)：同じ2飛行・校正済み単眼ROVIOで3条件と追加対照を検証。公式公開部品を改変せず適用した範囲、MIMOSAとの接続方法、旧方式の再現、位置合わせの採用・未収束を含む結果を記録。

[採用DRPMとFAST-LIVO2の比較](ADOPTED_DRPM_VS_FASTLIVO2_20261005.md)：ユーザー採用の公式更新＋距離方向モデルを画像ありFAST-LIVO2と比較。2方式だけの六軸図と数値を収録。

[センサ除外比較](SENSOR_ABLATION_20261005.md)：採用DRPM LIOを新規測定、既存FAST-LIVO2 LIO・ROVIOと集計。FAST-LIVO2の独立VIO未対応を公式実装から確認。

[FAST-LIVO2のカメラ比較](FASTLIVO2_CAMERA_COMPARISON_20261005.md)：link1/link3/link4を同じ2飛行・各3回で比較。長い飛行ではlink1が3/3、link4が2/3で発散。入力一致と数値の再現性の限界を記録。

[FAST-LIVO2の複数カメラ対応](FASTLIVO2_MULTICAMERA_20261005.md)：同期・固定露出の3台を共通状態へ逐次融合。2飛行各3回・ROSトピック入力・単眼互換を検証。

## 2026-10-08 LiDAR退化方向に応じた単眼ジンバル制御の先行研究

**確認した公開文献では、LIVOでLiDARの退化方向を求め、その方向の推定を改善するためにカメラを能動回転させる、という全条件を満たす論文は見つからなかった。** LiDAR退化方向への視覚情報の選択的融合、可動カメラによる自己位置推定の改善、LiDARの向きの能動制御には、それぞれ先行研究がある。「可動カメラをLIVOに付けること」だけを新規性とは判断しない。未発表研究や検索で発見できなかった論文の不存在を証明したものではない。

ユーザーの現在の方針はDRPM＋MIMOSAを採用し、単眼カメラをジンバルで動かす方法をまず調べること。今回の作業は文献調査のみであり、推定器・センサー配置・過去の評価結果は変更していない。

### 判定基準と主要文献

「退化方向」は見通しの悪い方角ではなく、観測から判別しにくい機体の移動・回転の方向。「視野制御」は取得後の画像の選別や融合重みの調整とは区別し、実際にカメラの向きを変える制御を指す。

| 文献 | 本文で確認した処理 | 今回の条件との差・確認箇所 |
|---|---|---|
| **Selective Kalman Filter: When and How to Fuse Multi-Sensor Information to Overcome Degeneracy in SLAM**（Xu et al., arXiv 2024） | R3LIVEを土台にLiDAR・画像・IMUを融合。LiDARの退化方向を求め、視覚情報をその方向に選択的に適用。 | 融合方針が最も近い。カメラの向きを選ぶ制御はない。III節の処理構成、IV節の退化判定、V–VII節の視覚選択・融合を確認。[本文](https://arxiv.org/html/2412.17235)・[保存PDF](active_gimbal_20261008/Selective_Kalman_Filter_2412.17235.pdf) |
| **IGLOV: Active View Planning for Visual SLAM in Outdoor Environments Based on Continuous Information Modeling**（Wang et al., IEEE/ASME Transactions on Mechatronics, 2023） | 地図中の視覚特徴から位置・姿勢の推定に使える情報量を評価し、2軸ジンバルの向きを計画。実験センサーはRGB-Dカメラ。 | ジンバル制御が最も近い。LiDAR退化方向に対する視覚の有効性を評価していない。III節の情報量モデル、IV節の向きの最適化、V-A節の構成を確認。[論文](https://doi.org/10.1109/TMECH.2023.3272910)・[保存PDF](active_gimbal_20261008/IGLOV_2211.06557v2.pdf)・[著者コード](https://github.com/HITSZ-NRSL/IGLOV) |
| **Active Visual SLAM with Independently Rotating Camera**（Bonetto et al., ECMR 2021） | 地上ロボットのRGB-Dカメラを本体とは独立に回転。2Dレーザー、2個のIMU、車輪の移動量も推定に利用し、カメラ相対角の誤差も扱う。 | レーザー・画像・IMU・可動カメラが既に共存する重要な例。ただしLiDARの退化方向からカメラ角を決める方式ではなく、今回の3D単眼LIVOと同一構成でもない。III節、特にIII-Cの推定入力と不確かさ、IV節の実験構成を確認。[論文](https://arxiv.org/abs/2105.08958)・[保存PDF](active_gimbal_20261008/Active_Visual_SLAM_Rotating_Camera_2105.08958.pdf)・[著者コード](https://github.com/eliabntt/active_v_slam) |
| **A Global ArUco-Based Lidar Navigation System for UAV Navigation in GNSS-Denied Environments**（Aerospace 2022） | UAVのLiDAR・IMU・ジンバルカメラを融合。位置既知の人工マーカーを観測し、LiDARの推定が退化した場合はマーカーの姿勢情報を使用する。 | センサー構成と退化時の視覚補助が近い。ただし自然画像の単眼VIOではなく、既知マーカーに依存する。退化方向からジンバルの目標角を求める制御則は示されていない。3.1–3.2節の構成・座標変換、3.4節の退化時処理を確認。[本文](https://www.mdpi.com/2226-4310/9/8/456)・[保存PDF](active_gimbal_20261008/ArUco_Lidar_Aerospace2022.pdf) |
| **AEOS: Active Environment-aware Optimal Scanning Control for UAV LiDAR-Inertial Odometry in Complex Scenes**（Li et al., ISPRS Journal of Photogrammetry and Remote Sensing, 2026／arXiv 2025） | 仮想的なLiDAR観測から、候補となる走査方向での推定の不確かさを予測し、モーターでLiDARを回す。モデルに基づく制御と学習を組み合わせる。 | 観測しにくさをセンサー回転で改善する先行例。ただし回す対象はLiDARで、視覚融合はない。著者公開版3節の装置、4.3節・式14–16の情報行列と不確かさを確認。[論文](https://doi.org/10.1016/j.isprsjprs.2026.01.006)・[本文](https://arxiv.org/html/2509.09141)・[保存PDF](active_gimbal_20261008/AEOS_2509.09141.pdf) |
| **DC-VINS: Dynamic Camera Visual Inertial Navigation System With Online Calibration**（Rebello et al., ICCV Workshops 2021） | 動くカメラと機体IMUの位置・姿勢関係を、機構のモデルとオンライン校正で扱い、VINS-Fusionへ組み込む。 | 可動カメラのVIO実装に直接関係する。LiDAR退化の検出や、それに基づく視野選択はない。本文のdegeneracyは校正の一意性の問題であり、LiDARの環境退化とは別。1節および提案モデル・校正の節を確認。[本文](https://openaccess.thecvf.com/content/ICCV2021W/TradiCV/papers/Rebello_DC-VINS_Dynamic_Camera_Visual_Inertial_Navigation_System_With_Online_Calibration_ICCVW_2021_paper.pdf)・[保存PDF](active_gimbal_20261008/DC_VINS_ICCVW2021.pdf) |

### 補助文献・未確定の関連動向

- **Texture-Aware SLAM Using Stereo Imagery and Inertial Information**（Manderson et al., CRV 2016）：画像中の追跡しやすい模様を評価し、ステレオカメラの向きを変更する視覚・慣性SLAM。LiDAR退化方向との連携はない。[著者公開本文](https://cim.mcgill.ca/~mrl/pubs/travism/crv_2016_texture_aware_slam.pdf)・[保存PDF](active_gimbal_20261008/Texture_Aware_SLAM_CRV2016.pdf)。
- **Perception-aware Autonomous Mast Motion Planning for Planetary Exploration Rovers**（Strader et al., Journal of Field Robotics, 2020）：既観測の環境から将来画像を予測し、視覚での移動推定に有利なマストの動きを選ぶ。LiDAR退化に対する補助カメラ制御ではない。[論文](https://doi.org/10.1002/rob.21925)・[保存PDF](active_gimbal_20261008/Mast_Planning_1912.06898.pdf)。
- **Fisher Information Field**（Zhang and Scaramuzza）：視覚による位置・姿勢の推定に使える情報を、経路・視点の計画で評価するための表現。視野候補を採点する基礎資料として保存したもので、今回の全条件を満たすシステムとは扱わない。[論文](https://arxiv.org/abs/2008.03324)・[保存PDF](active_gimbal_20261008/Fisher_Information_Field_2008.03324.pdf)。
- **AWARE**（2026）：機体のヨーを動かしLiDARによる推定を改善する。著者公開本文の方式・関連研究・限界を確認。カメラによる補助はない。本文はAEOSやFLAREという可動LiDAR研究も引用する。[本文](https://arxiv.org/html/2604.10598)・[保存PDF](active_gimbal_20261008/AWARE_2604.10598.pdf)。
- **SKF-Fusion / Fuse only what matters: Degeneracy-aware multi-sensor fusion for LiDAR-Inertial-Visual SLAM**（ISPRS 2026）：出版社ページでも方向選択的な融合を確認。[出版社](https://doi.org/10.1016/j.isprsjprs.2026.05.031)。出版社PDFは取得できなかったため、この最終版全体を確認したとは扱わない。上記Selective Kalman Filterの2024年公開本文を確認したことと区別する。
- **ETH Zurich RSLの研究紹介**には、視覚・IMU・LiDARの融合に加え、電動パン・チルトによる観測しにくさの回復・退化軽減を研究している旨が明記されている。ただし紹介ページだけでは、今回と同じ「LiDARの弱い方向を指定してカメラ制御」の実装・評価を示す論文は特定できない。研究が全く未着手という判断はしない。[研究紹介](https://rsl.ethz.ch/research/researchtopics/perception-navigation.html)。

### 今回の研究課題としての解釈

ここからは文献の記載そのものではなく、今回の構成への考察。

1. 差を検証する対象は、単なる特徴点数の最大化ではなく、**LiDARが捉えにくい移動・回転を、どのカメラ向きなら補えるか**を評価する制御である。SKFの方向選択とIGLOV等の向きの計画が対応する先行要素になる。この組合せが有用か、研究上十分な新規性かは、今回の調査だけでは確定しない。
2. 「退化方向を拘束する」と「その方角にカメラを正面から向ける」は同じではない。例えばトンネル長手方向の移動でも、側壁の模様を横から見るほうが画像の移動を捉えやすい条件がある。距離、模様、追跡の継続性、機体の運動も含めて方向ごとの効果を評価する必要がある。
3. カメラが機体に対して動くと、画像の動きには機体とジンバル両方の動きが混ざる。カメラ角と撮像時刻に対応した座標変換を推定へ入れる必要がある。この実装課題はDC-VINSが参考になる。本調査では現ROVIOを可動カメラ対応へ変更していない。
4. 比較するなら、同じ1台・同じ飛行で「固定方向」「視覚の情報量だけで向きを選択」「LiDARの弱い方向への補助効果で向きを選択」を分ける。単なるセンサー数増加や飛行軌道変更の効果と区別できる。実験は未実施。

### 保存・アクセス・調査範囲

10本のPDFを[active_gimbal_20261008](active_gimbal_20261008/)へ保存し、[download_manifest.json](active_gimbal_20261008/download_manifest.json)へURL・ページ数・SHA256・失敗したアクセスも記録した。全文を取得できた主要候補は、要旨だけでなく、処理構成・制御目的・推定入力が記載された本文を確認した。出版社版を取得できないものは著者公開版を利用し、出版社最終版との同一性は仮定していない。

2026-10-08時点で、LiDAR/visual/inertial、degeneracy/observability/weak direction、gimbal/rotating camera/pan-tilt/active gaze/view planningの組合せを検索し、近接研究の引用文献・著者ページも確認した。完全な網羅性や「世界初」は主張しない。特にETH RSLの研究紹介の具体的な論文対応と、入手できなかった出版社最終版は未確認事項として残す。
