# LIO更新の診断ログ

通常ビルドのFAST-LIVO2に、残差、速度／biasへのゲイン、事前・事後共分散、
回転・並進を結合したヘッセ行列を保存する機能を追加した。
X-ICPの有効・無効、ikd-tree・voxel-planeの双方で使える。
LIVOで有効にした場合も対象はLiDAR更新だけで、VIO画像更新は記録しない。

診断の既定値は無効。推定の更新式や対応点選択は変更しない。
既存の `xicp_diagnostic.csv` とは独立した設定・出力先を使用する。

## 有効化

`humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml` の `lio` 配下:

```yaml
diagnostics:
  enabled: true
  output_dir: ""    # 空ならソースのLog/diagnostics配下
  detail:
    enabled: false # trueで各対応点の残差・速度/biasゲインも記録
    start_sec: 170.0
    end_sec: 220.0
    final_only: false # trueなら対応点詳細は採用反復だけ。集計は全反復
  geometry:
    enabled: false # ikd-tree用。実際の近傍・棄却点・高精度状態を追加
```

この設定で通常のreconstructを起動すると、集計と行列を全反復について保存する。
詳細記録を有効にすると、`start_sec <= scan_time < end_sec` の対応点を追加保存する。
既定は全反復。`final_only: true`なら採用された最終反復に限定する。
時刻は最初のLiDARからのnode相対秒で、bagプレイヤー／RViz表示とは厳密には一致しない。
境界は有限値で `0 <= start_sec < end_sec` を満たす必要がある。

起動引数で上書きする場合（コンテナ内）:

```bash
source /opt/ros/humble/setup.bash
source /external/humble_ws/install/setup.bash
ros2 run fast_livo2 fastlivo2_mapping --ros-args \
  --params-file /external/humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml \
  -p common.img_en:=0 -p use_sim_time:=true \
  -p common.reliable_sensor_qos:=true \
  -p lio.diagnostics.enabled:=true \
  -p lio.diagnostics.output_dir:=/tmp/lio-diagnostics \
  -p lio.diagnostics.detail.enabled:=true \
  -p lio.diagnostics.detail.start_sec:=170.0 \
  -p lio.diagnostics.detail.end_sec:=220.0
```

別ターミナルからraw LiDAR・IMUとclockを再生する。
`reconstruct_raw.sh` の追加引数はbagプレイヤー用なので、上記nodeパラメータを
そのまま渡すことはできない。reconstruct経由ではYAMLを設定するか、
`--capture-diagnostics`で実行ごとの保存を有効にする。後者は通常ログも隔離し、
詳細・幾何の区間を[0,360)へ上書きする。`--normal-gate on|off`も追加した。
実行例と保存物は[法線ゲートの説明](NORMAL_CONSISTENCY_GATE.md)を参照。

## 保存形式

指定した親ディレクトリの下に `run-<system-clock-nanoseconds>-<suffix>/` を新規作成する。
既存実行のファイルを上書きしない。各実行の設定と形式は `metadata.json` に保存する。
診断を無効にすると、これらのファイル作成や診断用行列計算は行わない。

| ファイル | 内容 |
|---|---|
| `metadata.json` | schema version、状態順、単位・座標系、時間定義、詳細区間、宣言済みROSパラメータの文字列表現 |
| `iterations.csv` | 各IEKF反復の残差統計、6×6ヘッセ、19×19反復事前共分散、ゲイン集計、補正・制約 |
| `scans.csv` | スキャン単位の19×19事前・終了時共分散、更新結果、採用反復、診断の反復処理時間 |
| `correspondences.csv` | 詳細有効時のみ。各対応点の残差、innovation、重み、6列ヤコビアン、ゲインの9成分 |
| `geometry.csv` | 幾何記録有効時のみ。最終試行の全スキャン点、採否・棄却理由、探索時刻、実際の5近傍、平面情報 |
| `geometry_states.csv` | 上記のIMU予測・線形化時・更新後の高精度状態とLiDAR→IMU外部較正 |

現在の形式はschema version 3。集計解析はversion 1/2、幾何解析はversion 2も読める。

共通キーは `scan_id` と `iteration`。`scan_id` はこのロガーの呼び出し順で0から採番し、
初期地図作成など `StateEstimation` に入らない処理は含まない。
`iteration` は既存IEKFと同じく-1から始まる。
`point_index` はその反復の対応点リスト内だけで有効で、再対応付けや別スキャンで同じ点を意味しない。

行列列名は `名前_行_列`、0始まり・行優先。19状態の順序:

| 添字 | 状態 | 座標・単位 |
|---|---|---|
| 0–2 | 回転誤差 | IMU右摂動、rad（Euler角ではない） |
| 3–5 | 位置 | world、m |
| 6 | 逆露光状態 | 既存フィルタのinverse exposureスカラー |
| 7–9 | 速度 | world、m/s |
| 10–12 | gyro bias | IMU、rad/s |
| 13–15 | accel bias | IMU、m/s² |
| 16–18 | 重力 | world、m/s² |

### 残差・ヘッセ・ゲイン

`distance` は符号付き点―平面距離、`residual = -distance`。
`distance_mean` と `residual_mean` は符号付き平均で、既存コンソールの絶対値平均とは異なる。
`innovation = residual - H * prior_delta.head(6)`。
`H` は回転3列＋並進3列の観測ヤコビアン、`W = diag(inverse_variance)`。

- `HWH = Hᵀ W H` は6×6の全要素を保存する。回転・並進の交差ブロックを含み、IMU事前情報は含まない。
- `P_iter` はその反復の接空間へ移した事前共分散。
- `KH`、`gain_norm`（各行のL2ノルム）、`gain_noise_diag = diag(K R Kᵀ)`、
  `Kinnovation = K * innovation` を19状態すべてについて保存する。
- `prior_delta` と実際に適用した `solution` も保存する。重力補正は解を求めた後に射影されるので、
  重力成分については `solution - prior_delta` と `Kinnovation` が同じとは限らない。
- 詳細CSVの `K_7`〜`K_15` は、その対応点に対する速度・gyro bias・accel biasのゲイン全9成分。
  X-ICP有効時は制約付きの実際のゲイン、無効時は既存更新の計算済み逆情報行列から取り出したゲイン。
- `C` と `target = D * innovation` は最大6制約までゼロ埋めし、実際の行数を `constraints` に保存する。
  `gain_constraint_error` はX-ICPの既存ゲイン制約誤差で、無効時は0。
- `HTWinnovation = Hᵀ W innovation` と `innovation_wrss = innovationᵀ W innovation` も保存する。
  これらから同じ事前・対応点での無制約補正とNISをオフライン計算できる。

### 共分散と採用結果

`scans.csv` の `P_prior` はIMU予測時の共分散。
`P_after` は処理終了時の実際の状態共分散で、更新成功時には接空間移送・重力射影後の値。
反復途中の共分散を事後共分散として重複記録しない。

`status` は `updated`、`no_matches`、`rejected`、`no_map`、`no_update`。
`updated` の場合だけ `committed_iteration` が採用した反復を示す。その他は-2。
途中反復を記録していても、後の再対応付けで点がなくなったり数値更新が棄却された場合には、
その反復を採用済みと解釈しない。`P_after` には既存実装が実際に返した値を保存する。

`P_prior`、`P_iter`、`P_after` の回転・重力成分は、それぞれの状態の接空間に属する。
行列差をそのまま物理的な不確かさの減少と解釈しない。

## 解析

NumPyが必要。`--plot` を付ける場合はMatplotlibも必要。

```bash
python3 humble_ws/src/fast-livo2/scripts/analyze_lio_diagnostics.py \
  /tmp/lio-diagnostics/run-... --output /tmp/lio-analysis --plot
```

環境のユーザー側NumPyとシステムMatplotlibが競合する場合は、
`PYTHONNOUSERSITE=1` を付けて同じPython環境のパッケージで実行する。

`summary.json` と4枚のPNG（残差・ヘッセ、ゲイン、補正、共分散）を生成する。
共分散図の標準偏差は対数軸で、初期化後の小さな値も確認できる。
解析は採用スキャンの最終反復を選び、次を検査する:

- 記録済みスキャン内の反復の対応、採用結果、ログの欠落／重複。
- 補正恒等式（重力以外の16状態）、制約、終了時共分散の対称性・半正定値性。
- 詳細ログから再構成したヘッセ、残差統計、`KH`、速度・bias補正、ゲインノルム／雑音項と集計値の一致。
- 詳細ログの開始・終了境界と、設定で指定した反復・対応点が揃っていること。

ヘッセは単位が混在するため、生の行列を残したうえで、プロットでは
`diag(HWH)^(-1/2) HWH diag(HWH)^(-1/2)` の固有値を使う。
ゼロ情報列は正規化後もゼロとする。固有値やFULL判定だけで地図の正しさ・時間的可観測性を保証しない。
無制約補正 `K0*innovation` は同じ反復状態での比較であり、別の無制約軌跡ではない。
NISも観測・地図の相関を無視した記述量なので、カイ二乗検定の合否には使わない。

## 地図・対応点の幾何記録

`geometry.enabled: true`には`detail.enabled: true`が必要。
対応モードは`fastlio_ikdtree`と、その互換名`fastlio_knn`。
幾何記録を有効にして別モードを選ぶと起動時にエラーにする。
`detail.final_only`によらず、幾何記録は各スキャンの最後に試行した反復を1回保存する。
成功したスキャンでは採用反復、対応点なし・更新棄却では最後の失敗した試行に対応する。
地図なしで反復を行わなかったスキャンには幾何行がない。

### 点と局所地図

- `source_index`はdeskew・間引き後のLiDAR点群内の番号。同一スキャンの反復間では対応するが、
  元bagのraw点番号や別スキャンの同一点IDではない。
- `match_index`はその反復の`correspondences.csv`の`point_index`と結合できる。棄却点は−1。
- `lidar`はLiDAR座標、`world`は実際のマッチングに使ったfloat丸め後のworld座標。
  `normal`、`center`、`neighbors`もworld座標。`plane_d`で平面式`normal.dot(world) + plane_d`を作れる。
- `neighbors`は推定時に実際に使った最大5点。診断用の再探索は実行しない。
  キャッシュ再利用を区別するため、`search_iteration`、探索時の`search_world`、
  `search_farthest_squared`も保存する。現在の点位置から再計算した距離とは限らない。
- 近傍点の集合は**そのスキャンが参照した局所地図の記録**。未参照領域を含む完全なikd-tree、
  点の永続ID、内部の追加・削除・再構築イベント列は保存しない。
  対応平面・近傍の混在・地図面の時間変化を比較するために使う。
- `plane_fit_rms/max`は保存した平面に対する5近傍の適合誤差。
  `status`が4・5・6の場合だけ平面欄が有効。平面適合で棄却された群も近傍座標は残るので再評価できる。
  未使用の近傍欄や未成立の平面欄は0で埋める。無効入力点には非有限座標が残る場合がある。

| status | 判定 |
|---:|---|
| 0 | 入力点が非有限 |
| 1 | 近傍不足 |
| 2 | 近傍距離条件を満たさない |
| 3 | 平面適合条件を満たさない |
| 4 | 点―平面残差・点距離条件を満たさない |
| 5 | 対応点として採用 |
| 6 | スキャン・地図の法線不整合（schema-v3） |

### スキャン法線（schema-v3）

`source_normal`はLiDAR座標の中心化PCA法線、`source_eigenvalues`は昇順の3固有値（m²）。
`source_usable`、`source_quality`、`source_neighbor_count`、`source_max_neighbor_distance`を併記する。
品質番号は0未計算、1使用可、2非有限query、3近傍不足、4半径超過、5退化、6非平面、7広がり不足、8数値失敗。
不使用の法線・固有値欄は未計算なら0。品質の判定順によってPCA未実施の場合もある。

`normal_angle_valid=1`は元の近傍・平面・残差条件を通り、source法線が使用可能な点だけ。
`normal_angle_deg`はworld変換後の符号不変な角度。比較できなかった点はvalid=0、角度=nan。
ゲートOFFでもgeometryが有効なスキャンでは角度を保存する。

`scans.csv`には一度だけ行うsource法線計算の `source_normal_ms`、`source_normal_valid/invalid` と、
最後に試行した反復の `normal_comparisons`、`normal_rejected` を追加。
geometry記録中は正常なスキャンのvalid+invalidが全query点数に一致する。
ゲート・geometryの両方OFFでは追加探索せず、これらは0。

### 状態と検査

`geometry_states.csv`は`prior`（IMU予測）、`linearization`（その反復の補正前）、
`after`（戻り値の状態）の回転行列・位置・速度・両bias・重力・逆露光を最大精度で保存する。
回転はIMU→world、外部較正はLiDAR→IMU。
幾何照合には`linearization`を使い、採用結果は`scans.csv`の`status`で判断する。
`scans.csv`の`geometry_logging_ms`は幾何CSVの組み立て・書き込み時間。
呼び出し側の情報収集や最後のflush時間は含まない。

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  python3 humble_ws/src/fast-livo2/scripts/analyze_lio_geometry.py \
  /path/to/run-... --output /tmp/lio-geometry-analysis
```

大きな点CSVをスキャン単位で読み、記録区間の欠落、採用点との結合、座標変換、
平面距離・適合誤差、幾何から再構成したヤコビアンと残差を検査する。
フィルタ・ゲイン・共分散の検査は前節の`analyze_lio_diagnostics.py`も実行する。

現在のGo2W設定はユーザー指定の追加記録用に、診断ON・詳細140–325秒・最終反復のみ・幾何ON。
既存の較正・X-ICP・推定式は維持している。bagは初期化と地図履歴のため先頭から再生する。

```bash
# コンテナ内
cd /external
bash scripts/fast_livo2/reconstruct_raw.sh bags/raw_20260907_114335 --lio-only --rate 0.5
```

ノード時刻330秒以降を目安に停止し、同じ実行の通常Logと診断runを一緒に保全する。
幾何区間は140以上325未満なので、325秒以降の発散確認のために長く再生する必要はない。
診断ファイルは実行ごとの固有ディレクトリだが、通常の`mat_pre/out.txt`、`imu.txt`、
`xicp_diagnostic.csv`は次の再生で上書きされる。

## 負荷と失敗時

対応点の詳細記録はログ量・処理負荷が大きい。必要な時間帯だけ有効にする。
幾何記録では棄却点も保存する。今回の短時間実測は約7.4秒の詳細区間で
幾何約250 MB＋対応点約76 MB。185秒では数GB〜10 GB程度を見込むが、点数に依存する。
`iteration_logging_ms` は統計計算と反復CSV書き込みの時間で、ゲイン取り出し・スキャン行出力・flushは含まない。
全体の負荷は既存のLIO処理時間と再生終了時刻も併せて比較する。

各スキャン終了時にflushする。書き込み失敗時は警告を1回出してこの診断ロガーを停止し、
推定処理は続行する。その場合、途中までのCSVを完全な検証データとして扱わない。
解析のPASSは記録済みデータの整合性を示す。目的時刻まで記録できたかは、
最終スキャン時刻とnodeログの診断停止警告も併せて確認する。
既知のikd-tree再構築の非決定性があるため、診断による実行タイミングの変化で
長時間の軌跡が変わり得る。診断のON/OFFだけを根拠に精度の改善・悪化を判断しない。

## 実装時の検証（2026-09-27）

通常ビルドとCTest 4/4が成功。追加テストは、制約あり／なしのゲインを使ったCSV往復、
共分散の保存段階、詳細区間の境界、途中反復後の棄却・対応点なし、地図なし、
出力の無効化とI/O失敗を確認する。解析側でも空ログ・反復欠落の検出と、
更新なしのスキャンの扱いを確認した。

`raw_20260907_114335` の12秒LIO再生では、旧バイナリに対して
診断OFF・集計のみ・詳細ありの88状態とIMU積分ログが全列完全一致。
書き込み不能な保存先でも警告は1回で、88状態とIMU積分ログが旧バイナリと一致した。
X-ICP無効（88状態）とvoxel-plane（86状態）でも診断の整合性検査に合格した。
詳細CSVからヘッセ・残差・速度／biasゲインを再構成して集計値との一致を検査し、
無制約補正の解析式を独立した19状態の線形解法とも照合した。

長時間検証は同じbagをrate 1.0・RELIABLE・deterministic・RVizなしで、
診断OFF→ONの順に360秒ずつ再生した。X-ICPは両方有効で、詳細区間は170–220秒。
各実行のLogを別ディレクトリへマウントし、既存のユーザーログを保護した。

- 両方とも3,568状態、最終ノード時刻359.999秒、非有限値・更新棄却・診断停止警告なし。
- IMU積分71,382行、スキャン時刻、raw点数が完全一致。独立購読器の受信も各72,012件で、
  stampと6軸値のSHA256が一致。最大受信間隔8.346048 ms、10 ms超の間隔なし。
- 診断ONの14,126反復すべてで整合性検査に合格。詳細区間の500スキャン・1,980反復・
  3,322,070対応点を全件読み戻した。詳細再構成の最大相対スケール誤差は7.71e-11以下
  （`abs(actual-expected)/(1+abs(expected))`）。
- 補正恒等式の最大絶対誤差5.55e-16、制約補正誤差2.78e-16。
  終了時共分散の最小固有値は約1e-8、最大非対称成分2.43e-27以下。

| 処理時間・容量 | 診断OFF | 診断ON |
|---|---:|---:|
| 既存mappingタイマー・全区間平均 | 32.48 ms | 35.06 ms |
| 同・170–220秒平均 / p95 | 32.87 / 43.52 ms | 46.84 / 68.09 ms |
| 診断の反復処理・詳細区間外平均 | — | 0.63 ms/scan |
| 診断の反復処理・詳細区間平均 / p95 | — | 13.53 / 19.80 ms/scan |
| iterations / scans CSV | — | 151.70 / 41.04 MB |
| correspondences CSV | — | 1,440.11 MB |

容量は10進MB。既存mappingタイマーは終了端のバッファ未出力1行を除く3,567行を集計し、
時間窓には丸められたmat_out時刻ではなくX-ICP CSV時刻を使った。
実行間で軌跡は完全一致しないため、処理時間差にも対応点・反復の違いが含まれる。
両方で既知のSIGINT終了時exit -11を再現したが、360秒到達後の終了処理で発生した。
この検証は診断の記録・整合性・負荷を対象とし、後退問題の改善を示すものではない。

保存した検証結果:

- [検証集計](diagnostics/lio_diagnostics/validation.json)、[診断解析の集計](diagnostics/lio_diagnostics/summary.json)、[ビルド・CTest](diagnostics/lio_diagnostics/build_ctest.log)
- [残差・結合ヘッセ](diagnostics/lio_diagnostics/residuals_hessian.png)、[ゲイン](diagnostics/lio_diagnostics/gains.png)、[補正](diagnostics/lio_diagnostics/corrections.png)、[共分散](diagnostics/lio_diagnostics/covariance.png)
- 生ログ・再生スクリプト・開始時差分: `/tmp/fast-livo2-diagnostics-2dbp5e9q/`。
  詳細ログは `long-detail-diagnostics/run-1790501460215797836-0/`。

最終ビルドのSHA256は `9b79099fe63394bf44a70184f3b6657da5e01754cbafc55c055c0871884277da`。
短時間比較後の変更はメタデータの逆露光単位表記のみで、最終ビルドでもCTest 4/4と360秒比較を実施した。
