# 法線ゲート検証の解析成果物

対象は`normal-gate-20260928-071728-w6d4ljyu`の`run-1790579862998301727-0`。
[分析報告](../../NORMAL_GATE_FAILURE_ANALYSIS.md)と[改訂計画](../../UTURN_REMEDY_PLAN.md)を参照。

- `summary.json`: 取得条件、完全性検査、全点の品質別情報・ゲイン寄与、状態・床面の区間集計。
- `state-comparison.png`: 前回OFFとの状態比較。比較対象の記録終了後にOFFを外挿していない。
- `remaining-geometry.png`: 床面相互角度とworld傾斜、床棄却、判定不能法線群の前方情報。
- `repository-analysis-check.json`: 本番の追跡対象差分・ソース・バイナリの確認。
- `prepare.py`: 小さなログと取得条件を解析先へコピーし、取得時ソースハッシュを照合。
- `validate_extract.py`: 幾何・対応CSVの全件検査、全点寄与集計、627スキャンのキャッシュ抽出、フィルタ診断検査。
- `dynamics.py`: 事前・事後状態、実際のゲインと無制約ゲイン、姿勢／位置からbiasへの寄与、処理済みIMU比較。
- `geometry_samples.py`: 地図非依存のスキャン床候補と実際の地図近傍の平面検査。
- `summarize.py`: 集計JSON・図の生成、X-ICPの実際のカテゴリの確認。

スクリプトはrepo直下をカレントディレクトリとして実行する。
以前の解析の`analyze_geometry_cause.py`、FAST-LIVO2内の解析モジュールと、前回OFFの保存済み状態系列を利用する。
派生キャッシュはrepo内の`diagnostic_captures/normal-gate-analysis-20260928/`へ保存済み。
元の取得CSVを変更せず、bagを再生しない。

## 保存済みキャッシュから集計・図を再生成

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
MPLCONFIGDIR=/tmp/normal-gate-mpl \
python3 docs/diagnostics/normal_gate_failure_20260928/summarize.py \
  diagnostic_captures/normal-gate-analysis-20260928 \
  /tmp/normal-gate-recheck
```

## 元CSVからやり直す場合

ホストから元captureの権限がなく読めない場合は、既存Dockerイメージを使いrepoをread-onlyでマウントする。
解析先だけを書込み可能にし、元captureの権限を変更しない。
以下の処理は数分以上かかり、15 GB程度のCSVを複数回読む。通常の確認では再実施は不要。

```bash
mkdir -p /tmp/normal-gate-rebuild
docker run --rm --network none \
  -v "$PWD:/external:ro" -v /tmp/normal-gate-rebuild:/review \
  -w /external -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 \
  --entrypoint python3 fast-livo2-go2w:latest \
  docs/diagnostics/normal_gate_failure_20260928/prepare.py \
  diagnostic_captures/normal-gate-20260928-071728-w6d4ljyu /review

docker run --rm --network none \
  -v "$PWD:/external:ro" -v /tmp/normal-gate-rebuild:/review \
  -w /external -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 \
  -e MPLCONFIGDIR=/tmp/normal-gate-mpl \
  --entrypoint python3 fast-livo2-go2w:latest \
  docs/diagnostics/normal_gate_failure_20260928/validate_extract.py \
  diagnostic_captures/normal-gate-20260928-071728-w6d4ljyu /review
```

`validate_extract.py`の完了時に解析先のサブディレクトリをホストから読めるようにする。
続いてホストで実行する。Docker側が作った同名ファイルを再生成する場合は、新しい解析先を使う。

```bash
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR=/tmp/normal-gate-mpl
python3 docs/diagnostics/normal_gate_failure_20260928/dynamics.py /tmp/normal-gate-rebuild
python3 docs/diagnostics/normal_gate_failure_20260928/geometry_samples.py /tmp/normal-gate-rebuild
python3 docs/diagnostics/normal_gate_failure_20260928/summarize.py \
  /tmp/normal-gate-rebuild /tmp/normal-gate-recheck
```

床面サンプルは全期間約1 Hzと295–325秒の全スキャン。
床の集計は候補150点以上・RMS 15 mm未満に限定した。床の水平や姿勢真値は仮定していない。
全点集計の前方情報は水平化した推定IMU X軸への並進情報射影。
床面サンプルのSchur情報は、検出した床面の接線方向へ射影し、同じ法線利用可能点でmap/source法線を比較する。
両者は異なる指標なので混ぜない。

無制約ゲインや観測部分集合の比較は保存状態での感度検査であり、推定器を変更した閉ループ結果ではない。
