# 140–325秒の幾何診断の取得（2026-09-27）

取得後の全スキャン分析は[Uターン後退の幾何原因分析](UTURN_GEOMETRY_ROOT_CAUSE.md)を参照。

不足していた前半の対応点・局所地図の情報を取得するため、診断を拡張し、
既存bagを先頭から330秒までLIO-only・0.5倍速で再生した。
今回の記録は以前の348.4秒停止runとは別の実行。
各実行の姿勢・共分散・対応点を混ぜず、以下の同一runを使って解析する。

## 保存先

リポジトリ直下の **`diagnostic_captures/lio-geometry-ehdz2pcz/`**。
`/tmp`外に保存し、大きい生データの誤追加を避けるため`diagnostic_captures/`をGitの除外対象にした。

| ディレクトリ／ファイル | 内容 |
|---|---|
| `diagnostics/run-1790505839456699151-0/` | 今回の診断CSV・metadata |
| `capture/` | 同じ実行の通常Log、node/player出力、受信IMU時刻、再生結果 |
| `analysis/` | 数値整合性・幾何整合性の検査結果と図 |
| `provenance/` | 使用したバイナリ・設定・関連ソース、Git差分、短時間検証結果 |
| `manifest.json` | 診断・通常ログのサイズとSHA256、IMU比較結果 |

[保存ファイルのmanifest](../diagnostic_captures/lio-geometry-ehdz2pcz/manifest.json)と、
[使用バイナリ・設定の記録](diagnostics/geometry_capture_20260927/provenance.json)を参照。
生データはローカルの上記ディレクトリにあり、Gitだけでは保全されない。

## 記録内容

- 全区間は3,268スキャン、12,943反復。ノード時刻約330秒まで、全スキャンupdated・非有限状態なし。
- 幾何・対応点詳細は **140.099867–324.999745秒、1,850スキャン**。
  `140 <= t < 325`の全対象スキャンを記録した。
- 採用対応点 **3,014,303組**。残差、innovation、重み、ヤコビアン、速度／biasゲインを保存。
- 幾何情報 **8,183,295点**。採用・棄却した全間引き後スキャン点、その座標、採否・棄却理由、
  実際の5近傍、平面の法線・中心・オフセット・適合誤差を保存。
- IMU予測・最終反復の補正前・更新後の高精度状態、LiDAR→IMU外部較正も同時に保存。
- 近傍キャッシュを再利用した場合は、元の探索反復と探索点を記録。診断のための再探索は実行していない。
- CSV合計 **約6.55 GB**。幾何5.07 GB、対応点1.30 GB、全反復集計・共分散等約0.18 GB。

近傍の座標は、そのとき実際に参照した局所地図の記録。
未参照領域を含む完全な木、点の永続ID、内部の追加・削除・再構築イベント列は含まない。
スキャン内の点番号は間引き後の点番号で、bagのraw点番号ではない。
詳細な列定義は[LIO診断ガイド](LIO_DIAGNOSTICS.md)を参照。

## 検証

- 通常ビルド成功。診断テストに最終反復のみの出力、幾何の境界、近傍キャッシュ、棄却時の記録を追加。
  既存を含む4テストが成功。最初の実行で1件がコンテナ内にないネットワークIFの設定で失敗したため、
  `CYCLONEDDS_URI`を解除してそのテストを再実行した。
- 12秒の実bag比較で、新旧バイナリの状態出力88行、予測ログ120行、IMU入力1,780行が全列完全一致。
  短時間の幾何ログ74スキャン・396,324点の検査もPASS。
- 以前のschema version 1の詳細ログも、更新した解析スクリプトで検査PASS。
- 長時間記録のゲイン・共分散・詳細対応点検査PASS。
  補正恒等式最大誤差2.22e-16、制約誤差1.67e-16、事後共分散最小固有値約1e-8。
- **幾何ログ全1,850スキャン・8,183,295点の検査PASS**。
  座標変換・採用点の残差は再構成と完全一致。ヤコビアンの最大相対スケール誤差4.89e-15、
  平面距離・近傍適合誤差は最大2.82e-14。区間内の対象スキャンに欠落なし。
  幾何CSVの記録時間は詳細区間内平均約12.92 ms/scan、最大26.50 ms。
  情報収集・flushを含まないため、総処理時間とは区別する。
- 処理済みIMU **65,382行**が、前回348.4秒記録の同区間と全列完全一致。
  独立受信監視は66,007件、最大間隔8.346048 ms、10 ms超と逆行・重複時刻は0。
- 診断停止警告・更新棄却なし。終了時SIGINT後の既知のexit −11は今回も発生したが、
  330秒までの全スキャンと対象区間の記録が保存された。

既存のIMU単体テストが通常の`Log/imu.txt`を初期化したため、テスト前の保全コピーから復元した。
元の通常Log4ファイルは、取得後にも保存時のSHA256との一致を確認済み。
短時間・長時間のbag再生は通常Logを別ディレクトリへマウントして保護した。
以後のCTestも、通常Logを別マウントにした検証コンテナで実施する。

今回も機体前方の推定速度が20スキャン連続で負になる最初の時刻は321.099643秒。
終了時は−6.014 m/s。実速度の真値ではないが、調査対象の症状を含む記録が得られた。
ikd-treeの非同期再構築は残るため、前回との発症時刻差を診断拡張による精度変化と解釈しない。

保存した[集計・共分散検査](diagnostics/geometry_capture_20260927/capture-summary.json)、
[幾何検査](diagnostics/geometry_capture_20260927/capture-geometry-summary.json)、
[短時間の新旧比較](diagnostics/geometry_capture_20260927/smoke-comparison.json)を参照。
検査のPASSは記録・計算の整合性であり、地図や推定姿勢の正しさを保証するものではない。

## 再解析

リポジトリ直下から:

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/mpl-lio-geometry \
  python3 humble_ws/src/fast-livo2/scripts/analyze_lio_diagnostics.py \
  diagnostic_captures/lio-geometry-ehdz2pcz/diagnostics/run-1790505839456699151-0 \
  --output diagnostic_captures/lio-geometry-ehdz2pcz/analysis --plot

PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 \
  python3 humble_ws/src/fast-livo2/scripts/analyze_lio_geometry.py \
  diagnostic_captures/lio-geometry-ehdz2pcz/diagnostics/run-1790505839456699151-0 \
  --output diagnostic_captures/lio-geometry-ehdz2pcz/analysis
```

通常YAMLも診断ON・詳細140–325秒・最終反復のみ・幾何ONへ設定済み。
再取得する場合のコンテナ内コマンド:

```bash
cd /external
bash scripts/fast_livo2/reconstruct_raw.sh bags/raw_20260907_114335 --lio-only --rate 0.5
```

通常のreconstructでは診断は`humble_ws/src/fast-livo2/Log/diagnostics/`へ保存される。
上記の取得済みデータには追加の再生は不要。
推定式・較正・X-ICP閾値・gravity/bias推定設定は変更していない。
