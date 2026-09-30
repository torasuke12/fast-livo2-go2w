# diagnostic_captures の削除候補（2026-09-28調査）

削除は実施していない。以下のパスはすべてリポジトリ直下の `diagnostic_captures/` からの相対パス。
容量は実ディスク使用量、GB = 10^9 bytes。調査時の合計は **73.15 GB（68.12 GiB）**。
書き込み中の記録は増えるため、合計・空き見込みは調査時点の概算。

## A. 最初に削除する候補：旧経路の早期異常実行

| ディレクトリ全体 | 容量 | 判断理由 |
|---|---:|---|
| `lio-reconstruction-20260928-093457-4ekba5ed/` | 10.87 GB | rate未指定の早期異常実行。処理済みIMUに大きな欠落があり、Uターン後の後退比較の基準には使っていない。 |
| `lio-reconstruction-20260928-093324-oih3knjw/` | 3.21 GB | 同じくrate未指定の初期試行。31.8秒までの不完全記録で、推定器・RVizの異常終了あり。 |

**合計14.08 GB。** この2実行の早期異常をさらに調べる場合の原記録は失われるが、
現在の時間窓推定と、入力が揃った旧経路のUターン比較データには影響しない。
判断根拠は `docs/VOXEL_PLANE_FAILURE_ANALYSIS.md` の第1節にも記録済み。

## B. 原記録を残す場合に削除できる解析用データ

| ディレクトリ全体 | 容量 | 条件 |
|---|---:|---|
| `voxel-plane-analysis-20260928/` | 1.87 GB | `093848-xfyj5zxg` の原CSVを残す場合。抽出サンプル、解析出力、ログの控え。 |
| `normal-gate-analysis-20260928/` | 0.74 GB | `normal-gate-20260928-071728-w6d4ljyu` の原CSVを残す場合。解析出力とログの控え。 |

**合計2.61 GB、Aと合わせて16.69 GB。** 集計・図・解析コードは `docs/diagnostics/` にもある。
ただし、その小容量成果物が全ての原データを代替するわけではない。
下記Cも削除する場合は、Bの抽出データを元CSVから再生成できなくなるため、**Bは残すことを推奨**する。

## C. 過去の点単位の再解析を終了する場合：大容量CSVのみ

下記6ファイルは、既に分析した旧方式の全点対応・幾何情報。
削除しても状態・速度・bias・共分散などの集計比較は残せるが、任意の過去時刻の全点対応、
棄却理由、面品質を同じ記録から再検証することはできなくなる。
`docs/` に集計・図・結論は保存済み。大容量CSVの完全な別コピーは確認していない。

| ファイル | 容量 |
|---|---:|
| `lio-reconstruction-20260928-093848-xfyj5zxg/diagnostics/run-1790588346649852202-0/geometry.csv` | 27.35 GB |
| `lio-reconstruction-20260928-093848-xfyj5zxg/diagnostics/run-1790588346649852202-0/correspondences.csv` | 5.92 GB |
| `normal-gate-20260928-071728-w6d4ljyu/diagnostics/run-1790579862998301727-0/geometry.csv` | 12.20 GB |
| `normal-gate-20260928-071728-w6d4ljyu/diagnostics/run-1790579862998301727-0/correspondences.csv` | 2.41 GB |
| `lio-geometry-ehdz2pcz/diagnostics/run-1790505839456699151-0/geometry.csv` | 5.07 GB |
| `lio-geometry-ehdz2pcz/diagnostics/run-1790505839456699151-0/correspondences.csv` | 1.30 GB |

**合計54.25 GB。Aと合わせると68.33 GB削減、残り約4.82 GB。** Bの抽出サンプル・解析結果はこの見込みでは残している。

この3実行のディレクトリ全体を削除する案ではない。次は残す。

- `iterations.csv`、`scans.csv`、`geometry_states.csv`、`metadata.json`
- `logs/`、`normal-log/`、`capture/` にある状態・IMUなどの記録
- `capture.json`、params類、provenance、終了記録、解析結果

## D. 削除可能だが容量効果が小さいもの

以下は起動前のパラメータ検証で停止した記録。各約1.2 MB、6件で約7.3 MB。
同名ノード競合・daemon障害の調査結果は `docs/TEMPORAL_LIO_VALIDATION.md` と
`docs/diagnostics/temporal-lio-20260928/` に記録済み。

```text
lio-reconstruction-20260928-110254-g2vwxv39/
lio-reconstruction-20260928-110347-om_ag8bj/
lio-reconstruction-20260928-110428-q53d2ovk/
lio-reconstruction-20260928-112036-tcp_vsij/
lio-reconstruction-20260928-112155-gvybrxg4/
lio-reconstruction-20260928-112549-f5s58s1e/
```

`temporal-validation/lio-reconstruction-20260928-102722-jb78iki9/`（約15 MB）は、
平方根QRへ修正する前の実装の100秒途中停止記録であり、旧実装の生データを再検証しなければ削除候補。
現在の360秒成功実行とは別物。

## 残すもの

| 対象 | 理由 |
|---|---|
| `lio-reconstruction-20260928-113320-mx5y2ada/` | 最新の実行。推定器・IMU monitor等が現在もファイルを開いている。 |
| `lio-reconstruction-20260928-112403-xrqsubvx/` | 古い時刻名だが、推定器・IMU monitorが現在もファイルを開いている。終了確認前は削除しない。 |
| `temporal-validation/lio-reconstruction-20260928-103605-kvnl2o13/` | 約52 MB。360秒の時間窓推定成功とUターン改善の比較元。 |
| `temporal-final-parity/` | 約3.7 MB。最終ビルドと連続検証との一致確認。 |
| `lio-reconstruction-20260928-111239-0t3qdiem/` | 約11 MB。71.7秒でIMUギャップ上限により異常終了した時間窓実行。容量効果が小さく、入力欠落調査用に残す。 |
| `lio-reconstruction-20260928-111609-y38sum6q/`、`lio-reconstruction-20260928-111807-g7ni5il8/` | 小容量の未完了記録。今回は経緯を詳しく分析しておらず、削除対象には選んでいない。 |

現在開かれているファイルはユーザーコンテナの `/proc/*/fd` で確認した。
今回の調査ではプロセス停止・ファイル削除・圧縮は行っていない。
`bags/` の元センサーデータ、`docs/diagnostics/` の集計・図は、この削除候補の対象外。
