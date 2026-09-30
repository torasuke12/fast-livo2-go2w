# Temporal windowでのLIVO実行

`--temporal-window on` はLIO専用・LIVOの両方で使える。
`--lio-only` を省略すると、LiDAR・IMUに加えて画像の観測情報を時間窓に融合する。

```bash
bash docker/robot/run.sh bash scripts/fast_livo2/reconstruct_raw.sh \
  bags/raw_20260907_114335 \
  --temporal-window on --temporal-initialization legacy_voxel \
  --capture-diagnostics --rate 0.25
```

初期化は直前の比較と同じ `legacy_voxel` を明示している。省略時は既存の既定値 `single_scan`。
元YAMLの時間窓ON/OFF既定、較正・IMU雑音・RVizの表示設定は変更しない。

Temporal LIVOでは `lio.temporal.image_interval_sec=0.1` を既定とし、画像を約10 Hzで採用する。
LiDARもこの画像時刻で区切るため、従来の約100 ms分の観測を使う構成に合わせる。
元bagが30 Hzでも全画像を推定には使わない。時間窓OFFの通常LIVOの画像採用条件は変更しない。

別環境でソースを更新した場合は、通常のROSコンテナ内でビルドする：

```bash
cd /external/humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select fast_livo2 --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

## 以前拒否していた理由

従来はPython設定生成とC++起動条件の両方が画像ONを拒否していた。
時間窓とVIOフロントエンドが別々に状態を持ち、画像更新を窓内へ戻す経路がなかったためである。
ガードだけ外して共有状態をVIOで更新しても、次のLiDAR予測は窓内の古い状態から始まり、画像補正を上書きする。
さらに時間窓経路は、VIOが必要とする点・法線を渡していなかった。

## 統合の内容

- LIVOの既存同期処理に合わせ、画像時刻までのLiDAR点を同時刻の窓ノードに入れる。
  LIVOの点時刻は前回LiDAR更新時刻からのoffsetなので、IMU処理で時刻が進む前に基準を保存する。
  LIO専用の元スキャン時刻を使う処理は維持する。
  次の画像時刻を超える残りの点も毎回分割し直し、将来の点を早く消費しない。
- 画像フロントエンドが対応・ピラミッド反復を計算し、level 0で最後に受理した線形化から
  **画像の残差とJacobianだけ**を取り出す。フロントエンドの状態事前分布・事後状態・姿勢共分散を
  窓への新しい観測として重ねない。
- 露光推定がONなら、画像の露光列を自由なフレーム内変数としてQRで消去し、姿勢の平方根因子に圧縮する。
  露光の補正値は画像フロントエンドで管理し、時間窓の状態次元は15のまま。
  露光と窓内状態の全時刻の相関を保持する実装ではない。
- 画像因子は同時刻ノードへ一度だけ追加。窓内の姿勢・位置・速度・両biasと共分散を更新し、
  次の予測にも反映する。因子のノードを周辺化するときだけpriorへ取り込み、二重計上しない。
- 歪み補正後の点・局所面法線・点の共分散をVIOへ渡す。
  視覚点と参照画像に取得ノードを紐づけ、保持中の姿勢が修正された場合には位置・法線・参照姿勢と
  空間検索のvoxelを更新する。窓から外れた参照は最後に共有した姿勢で固定する。
- LIVOでは画像更新後の姿勢でodometry・path・今回の地図点・登録点群・画像を出力する。
  `/Laser_map` は全走行累積のままで、過去の累積点全体の再投影を今回実装したものではない。
- `lio.temporal.visual_fusion=true` を派生設定・実効設定で照合し、古いバイナリでの実行を検出する。

既存の直接法VIOと同様、視覚地図を条件付きの参照として扱う。
画像因子は追加時の線形化を保持し、過去画像の全画素・ランドマークを窓内で再最適化する
完全なphotometric bundle adjustmentではない。
今回のLIVO対応を、[既に確認したyaw・z問題](TEMPORAL_POST_INIT_ANALYSIS.md)の解消とは扱わない。

## 記録と検証

captureに `logs/temporal_visual.csv` を追加する。
画像時刻、`updated/no_constraints/rejected/frontend_rejected`、画素数、圧縮因子行数、
更新後の位置・速度・両bias、補正15成分・事後共分散を記録する。
`temporal_window.csv` はLiDAR更新後、`temporal_states.csv` はLiDAR・画像両更新後の窓状態を保存する。
同じ時刻・node IDの最後の状態が画像更新後になる。

`summary.json` の `temporal_visual_status_counts.updated` で、画像を実際に融合した回数を確認できる。
`no_constraints` は画像を処理したが使える因子がなかったフレーム。
拒否された画像は窓へ残さない。全画像が `no_constraints` の場合、画像の精度改善効果はない。
LIVO指定なのに画像診断が欠ける・途中で途絶える場合は、captureをincompleteにする。

RVizなしの短時間検証は、ROSコンテナ内で未使用のドメインを指定する：

```bash
cd /external
ROS_DOMAIN_ID=88 python3 scripts/fast_livo2/validate_temporal_replay.py \
  bags/raw_20260907_114335 --livo --temporal-initialization legacy_voxel \
  --duration 60 --rate 0.25
```

`--livo` を省略したヘッドレス検証は従来どおりLIO専用。
再生ラッパーは従来どおり画像topicの有無も確認する。

### 実装時に見つかった問題

- 最初のLIVO再生では、次の画像時刻より未来の点が残り点群から渡され、
  `invalid point or point timestamp` で停止した。残り点も画像時刻で分割する修正と、
  複数の画像区間を通して点が一度ずつ処理され元の取得時刻が保たれる試験を追加した。
- 修正後、入力画像30 Hzのまま60秒まで動作・保存した試行も、推定結果は不合格だった。
  初期に加速度biasのy成分が約−2 m/s²へ変わり、20秒以降に速度が発散した。
  `capture_integrity=ok` は保存の完全性を表し、推定の品質を保証しない。
  この結果を受け、約100 msの観測区間を保つ画像採用間隔を設けた。
  周期変更は初期化の蓄積、参照点群、因子数も変えるため、30 Hz失敗の単一原因を
  「点数不足」だけに限定するものではない。

検証記録・再現用集計は [diagnostics/temporal-livo-20260929](diagnostics/temporal-livo-20260929/)。

### 最終設定の実bag検証

- バイナリSHA256：`f51c500862d647ee3458f2dd1d17800f3f8c013033f57235086070bcf623e227`。
  captureの保存manifestと現在の推定器ソース・バイナリが一致。
- 約10 Hz、legacy初期化、rate 0.25のLIVO：
  `/tmp/temporal-livo-20260929/livo-10hz/lio-reconstruction-20260929-092958-yjqkog4r`。
  60.311秒までLiDAR 571行、画像570行（融合544、拘束なし26）、窓状態13,944行。
  node/player/monitor正常終了、`capture_integrity=ok`。最後の画像更新は60.211秒。
  監視IMUに20 ms超のgapなし。全状態は有限、LiDAR・画像更新後の共分散は正定値。
  区間内最大速度1.884 m/s、最大accel biasノルム0.402 m/s²。
  画像による位置・速度・両biasの更新が非ゼロであり、画像を読み込むだけの動作ではない。
- 同じ最終バイナリのLIO専用：
  `/tmp/temporal-livo-20260929/lio-interval/lio-reconstruction-20260929-092957-seshc5ih`。
  12.300秒まで、正常終了、integrity ok。変更前の最終legacy短時間記録と、共通89スキャンの
  全状態・共分散等がsolve_ms以外完全一致。窓状態731行・初期化3行・処理IMU先頭1,820行も一致。
- 最終C++ 9/9、Python全22件と画像間隔追加後のTemporal関連6件が成功。
  [final-validation.json](diagnostics/temporal-livo-20260929/final-validation.json)に実効設定・集計・ログhashを保存。

この検証は起動・融合・数値整合・短時間の著しい発散の有無を確認したもの。
真値に対する精度の改善、320秒後の後退、480秒付近・階段での誤差改善は、全走行で未検証。
