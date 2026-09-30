# スキャン・地図の法線整合性ゲート

2026-09-28実装。対象は `fastlio_ikdtree` と互換名 `fastlio_knn`。
実データに残っていた「床点が異なる向きの地図面に採用される対応」を除く。
実装時点ではbagを再生していない。
その後のユーザーによるON再生では、ゲートの数値検査は通ったが、322.80秒から持続的後退が再発した。
[検証結果と失敗理由](NORMAL_GATE_FAILURE_ANALYSIS.md)、[改訂した対処計画](UTURN_REMEDY_PLAN.md)を優先する。
現在のMID360既定は[voxel-plane](VOXEL_PLANE_SWITCH.md)。本書の法線ゲート比較ではikd-treeを明示する。

## 判定

1. deskew・間引き後のLiDARスキャンから、点自身を含む20近傍を求める。
   中心化PCAをスキャンごとに一度計算し、地図点をこの法線推定に使用しない。
2. 最大近傍距離 `< 1.0 m`、固有値 `λ0 <= λ1 <= λ2` について
   `λ1, λ2 > 1e-12 m²`、`λ0/λ1 < 0.1`、`λ1/λ2 > 0.1` を満たす法線だけを使う。
   非有限・不足・線状・重複・広がり不足などは理由を記録し、従来の対応判定へ戻す。
3. 各IEKF反復で `n_world = R_IMU_to_world * R_LiDAR_to_IMU * n_source`。
   元の地図近傍・平面適合・残差条件を通った対応について、
   `acos(clamp(abs(n_world.dot(n_map)), 0, 1)) > 30°` なら棄却する。
   法線の符号には依存しない。30°ちょうどは採用する（丸め誤差1e-12°を許容）。
4. 地図5近傍をキャッシュした反復でも、姿勢を更新して角度を再評価する。
   棄却後の同一対応リストをH・残差・X-ICP・診断に渡す。
   ゲートONで対応が0件になったときは、X-ICPのON/OFFによらず状態と共分散をIMU予測へ戻す。

局所法線はdeskew後のスキャンから求めるため、姿勢真値ではない。
地図更新・0.5 mの地図幅・重み・較正・IMU/bias更新式・X-ICP閾値は今回変更していない。
対応の棄却だけで地図への点追加を止める実装でもない。

## 設定

ROSパラメータは `lio.normal_consistency.*`。コードと両Go2W YAMLの既定はOFF。

```yaml
normal_consistency:
  enabled: false
  neighbors: 20
  max_neighbor_distance: 1.0
  max_planarity_ratio: 0.1
  min_spread_ratio: 0.1
  max_angle_deg: 30.0
```

非有限値、近傍数<3、半径<=0、比率が(0,1)外、角度が(0,90]外なら起動時にエラー。
`voxel_plane`でゲートONも起動時にエラー。
ゲートOFFかつそのスキャンのgeometry記録OFFなら、追加のスキャン近傍探索は行わない。
ゲートOFFでもgeometry記録中は法線・角度を求め、採否を変えずに比較材料を保存する。

## 再生と保存

普段のROSコンテナ内、リポジトリのルート `/external` から:

```bash
bash scripts/fast_livo2/reconstruct_raw.sh \
  bags/raw_20260907_114335 --lio-only --rate 0.5 \
  --correspondence-mode fastlio_ikdtree --normal-gate on --capture-diagnostics
```

ホストからは同じ引数を `bash docker/robot/run.sh` に続ける。
比較用OFF実行は `--normal-gate off`。省略時は元YAMLの値を使う。
保存場所を変える場合は `--capture-root /external/diagnostic_captures` を追加する。
元YAMLは書き換えず、実行用の派生YAMLを作る。

`diagnostic_captures/lio-reconstruction-<日時>-<ランダム文字列>/` に毎回新規保存する。
以前の保存先は`normal-gate-...`。既存記録は移動しない。

| 保存物 | 内容 |
|---|---|
| `params.yaml`, `original_params.yaml`, `effective_params.yaml` | 派生設定、元設定、起動したnodeの実効設定 |
| `capture.json`, `provenance/` | 引数、実行バイナリのSHA256、ROS環境、両repoのHEAD/status/diff、関連ソースのコピー・ハッシュ |
| `logs/` | `mat_pre.txt`, `mat_out.txt`, `imu.txt`, 有効時の `xicp_diagnostic.csv` |
| `diagnostics/run-*/` | schema-v3の集計、共分散、対応点ゲイン、幾何 |
| `node.console.log`, `player.console.log`, `rviz.console.log` | 画面にも表示する実行ログ |
| `imu_stamps.csv`, `imu_summary.json` | IMUヘッダー時刻(ns)、受信時刻、件数・最大間隔・非増加回数 |
| `*.exit.json`, `summary.json` | 終了コード、最終LIO時刻、360秒到達の有無、保存の不完全理由 |

集計・通常ログ・IMU監視は全時間、対応点詳細・geometryは **最初のLiDARから0以上360秒未満**。
初期化中などLIO更新前には診断CSV行はない。360秒で再生を自動停止しない。
全記録は10 GB以上になる可能性があり、起動時に空き容量を表示する。

通常の4ログは新パラメータ `debug.log_dir` で直接保存先へ出す。
空文字の既定では従来の `Log/`。PCD・画像・trajectory保存先はこのパラメータの対象外。
現在のMID360設定ではこれらの保存は無効だが、独自設定で有効にする場合は別途保存先に注意する。
IMU監視はreliable QoS・depth 2000で、準備完了と実効設定の検査後にbagプレイヤーを起動する。
受信件数は監視subscriberでの件数であり、推定nodeの受信件数やraw bagとの完全一致を保証しない。

停止はCtrl+CまたはRViz終了。先にプレイヤーを止めてからnodeと監視を閉じる。
失敗時も部分データを残す。既知のnode終了時の `exit code -11` が起きた場合も隠さず、
`summary.json` の `capture_integrity: incomplete` と終了コードに記録する。
この場合も、完了済みスキャンが解析できるかはCSV検査で判断できる。

## 保存後の検査

`RUN`には今回の保存先の `diagnostics/run-...` を指定する。
途中停止・異常終了の場合は、まず親の `summary.json` とconsoleを確認する。

```bash
RUN=diagnostic_captures/lio-reconstruction-.../diagnostics/run-...
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  python3 humble_ws/src/fast-livo2/scripts/analyze_lio_diagnostics.py \
  "$RUN" --output "$RUN/analysis/filter"
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  python3 humble_ws/src/fast-livo2/scripts/analyze_lio_geometry.py \
  "$RUN" --output "$RUN/analysis/geometry"
```

後者は座標・角度・採否・法線品質・件数・対応点への結合を、前者はゲイン寄与・ヘッセ・共分散を検査する。
schema-v1/v2の集計、schema-v2のgeometryも読める。
改善評価では棄却件数だけでなく、姿勢・加速度biasの蓄積、Uターン後のIMU予測加速度と速度を比較する。

## 実装検証

- 実記録205.099543秒、scan 2018 / iteration 2 / source 1113の20 source近傍と5 map近傍を小さなfixtureに保存。
  局所PCA法線との差は **78.5598°**。旧判定で採用、ゲートONで棄却。
  原因分析の79.1177°は、より広い床面から推定した法線との角度なので値を区別する。
- 平面、符号反転、非恒等の外部回転・姿勢、30°境界、線・重複・不足・NaNをテスト。
- キャッシュした近傍のまま回転を変更した再判定、X-ICP ON/OFF双方の全棄却時rollbackをテスト。
- ON/OFF × X-ICP ON/OFFの混在面で、採用対応・H・ゲイン・CSVの結合を照合。
- 保存処理はROS/bagを模擬したテストで、固有ディレクトリ、起動失敗、実行中の故障、Ctrl+C、RViz終了を検査。

テストコマンドと実行結果はhandoff第17節に記録する。
