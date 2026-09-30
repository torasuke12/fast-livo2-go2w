# voxel-plane切替（R1a、2026-09-28）

**後続の実データ検証で321.10秒から後退が再発した。** [失敗分析と修正方針](VOXEL_PLANE_FAILURE_ANALYSIS.md)を優先する。
以下の「再生未実施」は切替実装時点の検証範囲を示す。

MID360のLIO対応付けを既存の`voxel_plane`へ切り替えた。
面の法線・中心共分散と点の計測共分散から得た観測重みを、通常更新・X-ICP制約付き更新に使う。
**X-ICPの退化判定とPARTIAL補助問題は従来方式のまま**。品質を反映する拡張（R1b）と、
時間窓で姿勢・速度・biasを同時推定する処理（R2）は後続作業。
長時間bagでの後退症状の改善は、今回の設定・コード切替だけでは確認していない。

## 実行

ビルド・install反映済み。普段のROSコンテナ内、`/external`から、bag先頭より実行する。

```bash
bash scripts/fast_livo2/reconstruct_raw.sh \
  bags/raw_20260907_114335 --lio-only --rate 0.5 \
  --correspondence-mode voxel_plane --normal-gate off --capture-diagnostics
```

`go2w_fast_livo2.yaml`の既定もvoxel-planeなので、方式指定は再現条件を明示するため。
Hesai用YAMLの既定は変更していない。どちらも`--correspondence-mode`で上書きできる。
比較用ikd-treeは`--correspondence-mode fastlio_ikdtree`、法線ゲートは`--normal-gate on|off`。
互換名`fastlio_knn`も使える。voxel-planeと法線ゲートONの組合せはエラーになる。

`voxel_size=0.5`、`max_layer=2`、`max_points_num=100`、`dept_err=0.02`、`beam_err=0.05`を維持した。
X-ICPの有効設定・閾値、IMU・較正・地図更新規則はこの切替で調整していない。
初期化は既存の複数スキャン蓄積と面法線の広がり判定を使う。
consoleの初期化結果と初期化後約60秒の健全性を確認し、その後、ノード相対時刻360秒まで評価する。
初期化が通らない場合はconsoleを残し、タイムアウトを理由に判定を迂回しない。

## 保存・解析

保存先は`diagnostic_captures/lio-reconstruction-<日時>-<ランダム文字列>/`。
元設定・派生設定・実効設定、バックエンド、バイナリSHA256、ソース、通常Log、IMU時刻監視、
consoleと終了状態を保存する。実効方式が指定と異なるとbag再生前に検査で失敗する。
詳細・幾何の記録は最初のLiDARから`0 <= t < 360`秒。集計・通常Logは全区間。
初期化中などLIO更新前の幾何CSVはない。360秒での自動停止は行わない。
停止はCtrl+CまたはRViz終了。途中停止でも保存先を残し、失敗や不完全な終了は`summary.json`へ記録する。

voxel-planeは**schema-v4**、ikd-treeは従来のschema-v3。
voxel-planeの`geometry.csv`は各スキャン最終反復の全queryについて、次を記録する。

- 元スキャン点番号、採用対応番号、LiDAR/world座標。
- 参照root voxel、階層、面ID、法線・中心・固有値、支持点数、面更新の有効状態。
- 面の6×6共分散、照合時のworld点共分散、LiDAR点共分散。
- 面内距離・広がり、ゲート用分散と尤度、更新用分散の面・点別内訳。
- 採用、残差超過、voxelなし、平面なし、面範囲外、不正分散などの理由。

採用点は実際に選ばれた面を記録する。棄却点は最も後段まで進んだ代表候補を一つ記録し、
同段では規格化距離が小さい候補を優先する。全探索候補や地図全体のスナップショットではない。
存在しない5近傍やsource法線は出力しない。面更新停止後の支持点数も新しい支持の蓄積数とは扱わない。

既存の`correspondences.csv`で残差・実際の重み・H・速度/biasゲインへ結合できる。
`iterations.csv`と`scans.csv`の結合ヘッセ・事前事後共分散も継続する。
採用ゲート用と状態更新用は評価姿勢と点共分散が異なるため、分散を別々に照合する。

```bash
RUN=diagnostic_captures/lio-reconstruction-.../diagnostics/run-...
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  python3 humble_ws/src/fast-livo2/scripts/analyze_lio_diagnostics.py \
  "$RUN" --output "$RUN/analysis/filter"
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  python3 humble_ws/src/fast-livo2/scripts/analyze_lio_geometry.py \
  "$RUN" --output "$RUN/analysis/geometry"
```

geometry解析はschemaを見てvoxel用検査へ分岐する。座標変換、採否、共分散、分散の内訳、
対応番号、残差・H・実際の重みとの一致を検査する。
これらの数値検査への合格と、地図・姿勢の正しさや後退症状の解消は別に評価する。

## 併せて修正した不具合

- 面中心付近の丸めで面内距離の平方根がNaNになる場合を、非負への丸めで防いだ。
- ゲート分散が非有限・非正の場合は棄却する。採用面が未設定のまま対応成功を返さない。
- 再対応が0件になったスキャンは、状態・共分散をそのスキャンのIMU事前へ戻す。
  X-ICP無効時にも、前の反復の未確定補正を残さない。
- 実ノード起動試験で、既存カメラポインタを未初期化のまま`delete`する異常終了を確認した。
  `VIOManager`のカメラとサブマップのポインタ初期値を`nullptr`にした。

面の共分散は点間の相関や、姿勢と地図が共有する傾き誤差を完全には表さない。
今回の切替の評価後も、[改訂対処計画](UTURN_REMEDY_PLAN.md)の時間的整合性と地図の再評価を継続する。

## 検証結果

- Dockerの通常colconビルド・install成功、CTest **6/6 PASS**。
- captureのPythonテスト **14/14 PASS**。方式の上書き、不整合設定の拒否、実効方式の不一致も検査した。
- 実voxel平面を作るfixtureで、X-ICP ON/OFF、診断ON/OFFの推定結果一致、全棄却時の事前復元を確認した。
  分散・共分散・残差・H・ゲインへの結合もPASS。重みとゲート分散を壊したデータは期待どおり検査に失敗した。
- ikd-treeの診断と旧schema-v1/v2/v3の互換性検査PASS。
- install済みバイナリをセンサー入力なしで起動し、実効パラメータのvoxel-plane・幾何ON・ゲートOFF・画像OFFを確認した。
  SIGINTによる終了コードは0。これは実データ処理後の終了や長時間推定の検証ではない。

集計・ログ・設定・ソースハッシュは[検証記録](diagnostics/voxel_plane_switch_20260928/validation.json)。
生fixtureとビルドログは`diagnostic_captures/voxel-plane-switch-validation-20260928/`に保全した。
CTestのfixtureを残す場合は`VOXEL_TEST_OUTPUT=/tmp/<新規ディレクトリ>`を指定して
`humble_ws/build/fast_livo2/voxel_diagnostics_test`を実行し、
`tests/check_voxel_diagnostics.py <fixtureディレクトリ> <解析出力先>`で照合できる。

ビルド済みmappingバイナリSHA256:
`0b6627da701e0655183e72150d70351e37e3d4bfcc75aa1afda38cb00c12c863`。
今回はbagを再生していない。次の実データ検証では切替直後からの初期化・姿勢・biasを含めて記録する。
