# LIO/LIVO統一取得

本取得はユーザーが端末から開始する。既存captureを再利用しない。帰着評価、軌道最適化、設定チューニングは行わない。

## 条件

`config/experiments/lio_livo_comparison.yaml` が条件定義。2 bag × baseline/current × A/B/C = 12条件。

- baseline: f281e70ソース、voxel_plane対応。Go2W向けIMU等の変更を維持するためupstream FAST-LIVO2そのものではない。
- current: 作成時の作業ソースをsnapshot、2秒/6状態/legacy_voxel/X-ICP OFF。
- A: 画像更新なし、走査終端LIO。
- B: 画像経路を維持、視覚更新OFF。
- C: 画像経路、視覚更新ON。

B/Cは0.1秒間隔（許容1e-4秒）の画像採用。A/Cが主8条件、Bが補助4条件。同一構成のA/B/Cは同じバイナリを使用する。
基準Bは一時的な視覚posterior計算後、状態・共分散・露光と投影姿勢を復元し、その状態で視覚地図処理を継続する。
現在Bは時間窓への画像因子を空にし、露光posteriorの引き継ぎも無効にする。これは以前のABC実験と区別する明示的なパッチである。
A–Bは同期・点群分割・画像処理の複合差、B–Cは後続処理へのフィードバックを含む視覚更新有効化の総効果。

較正と時刻補正は採用プリセットから共通化する。画像内参はbag内CameraInfoの最初のメッセージと照合する。外部較正の物理的な正しさを保証する検査ではない。

## 作成と起動

Dockerへアクセスできる通常端末から、リポジトリルートで実行する。Python 3、PyYAML、Dockerが必要。

```bash
python3 scripts/fast_livo2/experiment_suite.py prepare \
  experiments/unified_acquisition/package_new --image fast-livo2-go2w:latest
```

タグは作成時にイメージIDへ解決する。基準ソースはGit archive、現在ソースは変更・未追跡のソースを含むsnapshot。Livox依存ライブラリも独立ビルドする。通常のbuild/installを変更しない。
パッケージ作成は31 GiB程度のbagハッシュ計算とC++ビルドを含む。途中失敗したディレクトリにはlock.jsonがなく、実行不能。新しい出力名で作り直す。

以後は**パッケージ内の固定された入口**を使う。

```bash
python3 experiments/unified_acquisition/package_new/runtime/experiment_suite.py verify \
  experiments/unified_acquisition/package_new

python3 experiments/unified_acquisition/package_new/runtime/experiment_suite.py prefix \
  experiments/unified_acquisition/package_new experiments/unified_acquisition/run_new

nohup python3 experiments/unified_acquisition/package_new/runtime/experiment_suite.py run \
  experiments/unified_acquisition/package_new experiments/unified_acquisition/run_new \
  --resume > experiments/unified_acquisition/run_new/driver.log 2>&1 &
```

`run`だけでも各条件の45秒prefix→本取得を順次行う。上記はprefixを先に確認する手順。既存出力の利用には`--resume`が必要で、今回の同一パッケージの完了条件のみスキップする。発散条件は延長しない。失敗条件を自動再試行しない。
`--condition sep07_baseline_B`で1条件を選択できる。

```bash
python3 experiments/unified_acquisition/package_new/runtime/experiment_suite.py status \
  experiments/unified_acquisition/run_new
```

ホスト側のdriver.logはパッケージの外へ保存する。固定パッケージにはログ等を追加しない。ソース・実行物・設定の改変、追加・欠落はverifyで拒否する。

## 制限と停止

ユーザー指定の今回のPC用プロファイル: CPU上限6論理CPU相当、コンテナ全体10 GiB hard cap・swap追加禁止、プロセス群RSS 9 GiBで停止、ホストMemAvailable 3 GiB未満で停止。主要数値計算1スレッド。旧来の2 CPU/6 GiBとは別の明示的プロファイルで、小容量ホストでは開始しない。

全条件0.1倍速、同時実験1本。遅れを取り戻す高速投入をしないので所要時間は単純な10倍より長くなることがある。1秒ごとに監督。連続2区間5 m/sまたは単発10 m/s、非有限値、対応ゼロ3更新、残差増大、overflow、処理10秒超、初期化後の進捗30秒停滞、ホスト圧迫等で停止する。
初期化中は更新イベントの進捗を監視し、0.1倍速では最大130秒で初期化タイムアウト。同一時刻の視覚更新で1 mを超える位置ジャンプも停止対象とする。
発散は記録して後始末後に次条件へ進む。資源問題・クラッシュ・監視不能・起動失敗は全列を停止する。停止後コンテナを削除し、残存を確認する。旧版の保存後exit 139も正常終了扱いしない。

`result.json`のcomplete/diverged/failedと停止理由を必ず参照する。prefixとfullの出力は別。部分軌道を全区間として扱わない。

## 診断の意味

- events.csv: 更新時刻、状態の主要値、LiDAR対応数/残差、処理時間。初期化中など対応未確定は-1。`visual_candidates`列はVIOのtotal_points（選択済みパッチ数）であり、棄却前の全候補数ではない。時間窓ログのcandidate_pointsとは異なる。
- states.csv/covariance.csv: 更新前後の19状態と共分散。状態順序・座標系はdiagnostic_schema.json。時間窓固有15状態ログも残す。
- lidar_correspondences.csv: 基準は最後のIEKF対応と逆分散、現在は時間窓内の構造面対応と因子重み。両者の重みの数値をそのまま比較しない。
- visual_points.csv: 選択されたmap patchの画面位置・誤差。Bでは適用されない候補posteriorで計算した誤差を含む。棄却された全候補は保持されない。
- images.csv/inputs.csv: 原bag時刻・画像採用有無と入力履歴。Aでも場面確認用画像を保存するが推定器は画像を採用しない。
- images/、scans/、poses.csv、export/trajectory.tum: 約0.5秒間隔の場面・未補正軌道。

入力件数と画像採用周期を終了時に検査する。共分散の小ささ、対応数、軌道間差は真値精度を保証しない。

2026-11-11更新: ホスト全体のSwapFree減少を単独の停止条件から除外。resource.csvのswap_free_bytes記録、コンテナswap追加禁止、RSS/利用可能メモリ/PSIの停止条件は維持する。既存パッケージは変更せずfrozen_20261111_swap_telemetryを作成した。

## 固定基準版をRVizで手動確認する（2026-11-21追記）

対象は今回の固定基準版（f281e70＋実験用パッチ、`voxel_plane`）のA：LIOと、9月7日のbag。以前の`fastlio_knn`構成ではない。固定バイナリを直接起動し、同じコンテナのRVizで表示する。通常の`reconstruct_raw.sh`による現在のワークスペースの起動とは異なる。

初期挙動を確認するため、先頭10秒・0.1倍速に限定する。これはユーザーが手動で行う診断であり、campaignの発散・資源監督処理を通さない。`docker/robot/run.sh`も本取得用のCPU・メモリ上限を設定しない。実時間180秒のtimeoutのみを設定する。Codexがこの手順をそのまま検証実行するものではない。

### 1. ホストのデスクトップ端末でコンテナを開く

```bash
cd ~/fast_livo2_go2w/src/fast-livo2-go2w

FAST_LIVO2_GO2W_IMAGE=sha256:4a7f294e39a8ad5dbe23223ceb0124752e52c8caa473c5253a2c05999f764a47 \
FAST_LIVO2_GO2W_CONTAINER=baseline-rviz \
ROS_DOMAIN_ID=89 \
bash docker/robot/run.sh bash
```

### 2. コンテナ内で設定し、RVizを起動する

```bash
source /opt/ros/humble/setup.bash
cd /external

PKG=/external/experiments/unified_acquisition/frozen_20261111_swap_telemetry
OUT=/external/experiments/unified_acquisition/manual_baseline_A_$(date +%Y%m%d_%H%M%S)

mkdir -p "$OUT"
cp "$PKG/params/baseline_A.yaml" "$OUT/params.yaml"

export LD_LIBRARY_PATH="$PKG/livox/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export FAST_LIVO2_DETERMINISTIC=1
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1

rviz2 -d /external/config/fast_livo2/fast_livo2.rviz \
  --ros-args -p use_sim_time:=false \
  > "$OUT/rviz.log" 2>&1 &

echo "保存先: $OUT"
```

### 3. RVizが開いたら、同じ端末で再構成を開始する

```bash
sha256sum "$PKG/build/baseline/fastlivo2_offline" \
  > "$OUT/binary.sha256"

set -o pipefail

timeout --signal=INT --kill-after=5s 180s \
  "$PKG/build/baseline/fastlivo2_offline" \
  /external/bags/raw_20260907_114335 \
  "$OUT/frontend" \
  --ros-args \
  --params-file "$OUT/params.yaml" \
  -p offline.rate:=0.1 \
  -p offline.duration:=10.0 \
  2>&1 | tee "$OUT/frontend.log"
```

10秒分の処理に実時間約100秒以上を要する。180秒で終了しなければINT、その5秒後まで残ればKILLで打ち切る。途中停止はCtrl+C。RVizで表示されない場合はFixed Frameを`camera_init`にし、`/cloud_registered`の表示を確認する。再構成終了後もRVizは開いたままなので、確認後にウィンドウを閉じる。

出力は`manual_baseline_A_*`に分離し、campaignや固定パッケージを上書きしない。コマンドラインで上書きした区間指定等は`frontend/effective_parameters.txt`を確認する（コピーした`params.yaml`単体には上書き値が反映されない）。本取得の`result.json`による監督結果は生成されず、この短区間の結果を本取得の完走・発散判定に混ぜない。

2026-11-21 RViz接続設定の訂正: マルチキャスト無効時に別プロセスのRVizと推定器が探索できるよう、上記CYCLONEDDS_URIにlocalhostのPeerとParticipantIndex=autoを明示した。変更は両プロセスの再起動後に反映される。Aでは画像トピックの内容は配信されず、publish.map_en=falseなので累積地図/Laser_mapも配信されない。表示確認には/cloud_registeredと/pathを使う。

### 1コマンドで起動する補助スクリプト

リポジトリ直下で次を実行する。ホストではDockerを起動し、既存コンテナ内ではそのまま実行する。RViz・DDS設定、パラメータのコピーと区間／倍率反映、一意な出力先、バイナリハッシュ・実行コマンド・終了コードの保存を自動化する。

```bash
bash scripts/fast_livo2/reconstruct_baseline_rviz.sh
```

既定は9月7日、基準版A、10秒、0.1倍速。手動で指定する場合:

```bash
bash scripts/fast_livo2/reconstruct_baseline_rviz.sh --mode A --rate 0.7 --duration 50
```

`--mode B`／`--mode C`も指定可能。Bには画像更新無効化の環境変数を適用する。短区間診断専用でdurationは60秒以下、rateは0より大きく1以下。実時間上限は `ceil(duration / rate * 2 + 60)` 秒。推定器終了後はRVizを閉じるとスクリプトも終了する。この補助スクリプトもcampaignの発散・資源監督を通さない。シェル構文と引数の検査を実施済みだが、Codexによるbag再実行・GUI接続試験は行っていない。
