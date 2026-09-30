# 時間窓方式の初期化だけを切り替える比較

2026-09-29の全走行では、初期方位と前半の一部の軌道形状が改善した一方、階段の大きなz補正はほぼ不変だった。
以下の短時間評価と併せて[全走行の分析・次の対処](TEMPORAL_POST_INIT_ANALYSIS.md)を参照する。

`lio.temporal.initialization_mode`で、時間窓方式の地図初期化を切り替える。
既定値は`single_scan`。初期化後の最適化・面の重み・照合先の選び方を共通にして、
初期化変更が[精度の退行](TEMPORAL_EARLY_REGRESSION.md)に与えた影響を切り分ける。

| 値 | 初期化 |
|---|---|
| `single_scan` | 変更前の時間窓方式。最初の1スキャンを登録し、次から更新する |
| `legacy_voxel` | 従来の`TryInitializeVoxelMap()`で複数スキャンを蓄積し、点数・時間・面数・法線方向のREADY判定を通す。蓄積点群を最初の参照地図に渡す |

切り替えるのはLiDAR地図の初期化。3秒の静止IMU初期化、重力・較正・雑音設定は共通。
`legacy_voxel`でも従来IEKFの観測更新は行わず、初期化完了時点のIMU予測状態と共分散を
時間窓のpriorに使う。READY前には時間窓の観測更新を行わない。
READY時刻が変わるため、初回更新時刻と、それを起点に選ぶキーフレームの時刻も変わる。
キーフレームを選ぶ規則や周期は変更していない。

蓄積した各点は、取得時の従来処理で歪み補正してworldへ変換した座標を保持し、
READY時点のIMU終端座標系へ一度だけ変換する。これを最初のノードの参照形状として保存し、
現在スキャンのraw点・時刻と混ぜたり、架空の取得時刻で再び歪み補正したりしない。
最初のノードの姿勢は通常どおり同時推定し、参照形状はその座標系で固定する。

**従来のoctree照合・面共分散モデルまで復元する切替ではない。** READY判定には従来の面評価を使うが、
時間窓に渡した蓄積点群には共通の時間窓PCA・観測重みを使う。
次のキーフレームからは通常の単一スキャン参照へ進み、初期ノードの周辺化時に蓄積点群も解放する。
照合方式の復元は別の比較項目として残す。

## 0–60秒の比較コマンド

この変更はC++を含むため、古いバイナリを使用する別環境では再ビルドする。
通常のROSコンテナ内で実行する。

```bash
cd /external/humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select fast_livo2 --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
cd /external
```

未使用のROSドメインを選び、以下を順に実行する。こちらのヘッドレス検証では`ROS_DOMAIN_ID`を使う。
いずれも同じbagの先頭から再生し、推定時刻が60秒に到達すると自動停止する。
停止確認・終了処理による数スキャンの超過を許す。RVizは起動しない。

```bash
ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1 \
python3 scripts/fast_livo2/validate_temporal_replay.py \
  bags/raw_20260907_114335 --duration 60 --rate 0.25 \
  --temporal-initialization single_scan \
  --root diagnostic_captures/init-single-60

ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 \
python3 scripts/fast_livo2/validate_temporal_replay.py \
  bags/raw_20260907_114335 --duration 60 --rate 0.25 \
  --temporal-initialization legacy_voxel \
  --root diagnostic_captures/init-legacy-60
```

各rootの下に一意な`lio-reconstruction-*`を作成する。既存captureを上書きしない。
新しい現地収録は不要。両方とも同じビルドで実行する。

RVizを使う通常スクリプトも同じ切替に対応する。こちらは自動60秒停止ではない。

```bash
FAST_LIVO2_RECONSTRUCTION_DOMAIN_ID=44 \
bash scripts/fast_livo2/reconstruct_raw.sh bags/raw_20260907_114335 \
  --lio-only --temporal-window on --temporal-initialization legacy_voxel \
  --capture-diagnostics --rate 0.25
```

`--temporal-initialization`を明示する場合は時間窓を有効にする。
モードは派生YAML・実効パラメータ・captureメタデータに保存し、実効値の不一致は再生前に拒否する。

## 記録と判定

`logs/temporal_initialization.csv`に、判定時刻（最初のLiDARからの秒）、mode、waiting/ready、
gate適用の有無、蓄積スキャン数・点数・従来面数・法線情報の固有値・蓄積時間・reset回数、
最終参照点数・時間窓側の面数を記録する。従来面数と時間窓側の面数は異なる評価なので区別する。
`single_scan`はgateを適用せず、gate関連の数値は0、ready行は1行。
新しいcaptureの終了検証は、このCSVの欠落やREADY未達も不完全として報告する。

共通の`temporal_window.csv`に状態・予測からの補正・共分散・残差、`temporal_states.csv`に保持状態が残る。
まず初期姿勢補正と0–60秒の相対軌道を比較する。後退解消だけで採用せず、
改善が確認できた場合も180–300秒、320秒後、階段・全走行へ順に広げる。
従来推定を真値とは扱わない。

## 実bagでの比較（2026-09-29）

同じ`raw_20260907_114335`、rate 0.25、同一バイナリで先頭から60.199秒まで再生した。
出力先を除く実効パラメータの差は`lio.temporal.initialization_mode`の1項目だけ。
処理IMUログ11,420行は全列・全行一致。共分散は全スキャンで有限・正定値だった。

| 指標 | single_scan | legacy_voxel |
|---|---:|---:|
| 初期登録時刻 | 3.199秒 | 3.400秒 |
| 最初の観測更新 | 3.299秒 | 3.499秒 |
| 初期参照点数 | 14,350 | 15,791 |
| 時間窓で使う初期面数 | 200 | 245 |
| 3.499–10秒：姿勢補正95%点 | 5.579° | 20.041° |
| 同区間：姿勢補正最大 | 11.702° | 29.093° |
| 同区間：位置補正最大 | 0.01867 m | 0.25592 m |
| 10–20秒：姿勢補正95%点 | 3.014° | 0.558° |
| 20–60秒：位置補正95%点 | 0.04424 m | 0.02853 m |

`legacy_voxel`のREADY判定は、従来ログと同じ3スキャン・15,791点・625面、
法線固有値`[0.29826955, 0.31324614, 0.38848431]`を再現した。
625は従来gateの面数であり、時間窓側で再推定する245面と取り違えない。

**初期化を戻すだけで初期の不安定さを解決するという結果ではなかった。**
3.499–10秒では大きな補正が増え、後の区間では減った指標もある。
補正の大小自体を真値に対する精度とはせず、比較用の切替として扱う。
既定値は`single_scan`のまま。参照面・観測重み・因子の整合性は引き続き切り分けが必要。

既定モードの変更前ログとの照合は、先頭12.2秒のwindow CSV 91行
（処理時間を除く493数値列とstatus）、保持状態CSV 741行20列が完全一致。
C++ 7/7、Python 21/21成功。記録・再現スクリプトは
[diagnostics/temporal-initialization-20260929](diagnostics/temporal-initialization-20260929/)。
実行ログはホストの`/tmp/temporal-initialization-20260929/{single,legacy}`に保存した。
比較JSONの`/work`はこのホストディレクトリのコンテナ内マウント名。

### 停止時の不具合と検証版の区別

上の60秒比較はバイナリ`49eb0f67…57913`を使用。
single側は`integrity=ok`、legacy側は569スキャン保存後のSIGINT停止時にSIGSEGVとなり、
元の`summary.json`は**incompleteのまま保持**した。
解析スクリプトで明示的に`--allow-shutdown-sigsegv`を指定し、終了不良と保存済み行の比較を区別した。

12秒の再現試験でも、スタックは`LIVMapper`破棄中のPCL点群vectorの`free()`を示した。
バイナリPCLの確保と`-march=native`側のEigen aligned allocatorによる解放が一致しない。
独立した小さなPCLプログラムでも、PCL側に出力確保を任せるとexit −11、
呼出側で出力点数の上限まで事前確保するとexit 0を再現した。

今回追加した初期化経路では、間引き前に出力領域を入力点数まで`reserve()`するよう修正。
間引きの点・閾値・数式は同じ。従来初期化への切替が安全に終了できるようにする修正であり、
初期化後の推定モデルの調整は含めていない。

最終install済みバイナリは
`27baf23ed5c5c36e2f943b79b96a279433461bdafebd579f8137023353746bac`。
PCL出力の破棄を含むC++回帰テスト7/7成功。
`legacy_voxel`を12.200秒まで再生し、node/player/monitorはexit 0、`capture_integrity=ok`。
修正前の同区間とwindow 89行・保持状態731行・初期化3行が、処理時間以外は完全一致した。
最終ビルドでの実再生検証はこの12.2秒までで、60秒比較は前述の初版によるもの。
停止の修正後に全区間の精度が改善したとは扱わない。
詳細は[final-validation.json](diagnostics/temporal-initialization-20260929/final-validation.json)。
