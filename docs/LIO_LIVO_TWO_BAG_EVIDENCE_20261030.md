# 9月7日・10月20日bag：f281e70／現行X-ICP／現行時間窓のLIO・LIVO

2026-11-03追記。対象は `bags/raw_20260907_114335`（648.1秒）と `bags/raw_20261020_043530`（339.9秒）。比較する推定条件は**3構成×LIO/LIVO×2 bag＝12通り**。保存済み結果の完全性、評価可能な区間、設定差を先に確認し、発散後や別設定の値を完走比較に混ぜない。独立した真値軌道はないため、軌道差と再訪点群は内部整合性の指標である。異なる推定経路では開始座標系のyaw差があるため、未整列の位置差だけで発散を判定しない。

場面画像・保存点群の形状・共分散・bias・視覚更新を時刻同期した解釈は[環境別の証拠整理](LIO_LIVO_ENVIRONMENT_EVIDENCE_20261103.md)に記した。

## 構成の定義

| 構成 | 元のYAML | 設定と実効推定経路 |
| --- | --- | --- |
| **f281e70** | `fast-livo2` commit `f281e70ce50a7126b4a8d6bb38df9d95a98bc0df` の `go2w_fast_livo2.yaml` | FAST-LIO形式のKNN対応・従来IEKF。X-ICP未実装。旧実行ファイル SHA256 `5a045a538e5a3ddac7c340cda037bb88a723d45c2d4fa1eaccabd87ca6bac5e3` |
| **現行採用（X-ICP）** | 最新の `humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml` | `lio.temporal.enabled=false`、`lio.xicp.enabled=true`。voxel-plane IEKFにX-ICPが作用する。`reconstruct` デフォルト |
| **現行採用（時間窓）** | 同じ最新デフォルトYAML | `lio.temporal.enabled=true`、`lio.xicp.enabled=false`。時間窓長6秒、最大16状態、初期化`single_scan`。それ以外の推定パラメータは現行採用（X-ICP）と同じ |

現行の元YAML SHA256は `6bacc9bc72ccfa1a51cc299b89b4dccc4ddf24bb08d715bbca470234889cd1e5`、実行ファイル SHA256は `a618fa6ad3e380a2169af6e7620683ffc4d05fc1ee5c4326188f5ac569007de1`。同じ構成内のLIOとLIVOは `common.img_en=0/1` だけを変える。診断出力、オフライン入力rate・durationは実験制御として別に記録する。`vio.image_interval_sec=0.1` を追加した10月20日LIVO試行は、デフォルト設定のLIVOと同一条件にはしない。

この二者択一は単なる命名ではない。現行コードは時間窓有効時に `voxel_config.xicp_.enabled=false` とし、LiDAR更新を `handleTemporalLIO()` へ分岐する。[実装](../humble_ws/src/fast-livo2/src/LIVMapper.cpp)。従って時間窓YAMLでX-ICPをtrueにしても、実効X-ICPは無効。**本比較では時間窓側のX-ICPフラグもfalseと明示し、設定と実際の処理を一致させる。**

## 12通りの保存状況

| bag | 構成 | LIO | LIVO | 使用できる根拠 |
| --- | --- | --- | --- | --- |
| 9月7日 | f281e70 | bag読了。連続2姿勢区間の5 m/s超を493.9秒で判定 | bag読了。同じ判定は498.6秒 | [旧構成の比較](../diagnostic_captures/pre_oct1_f281e70_20260907/FINDINGS.md) |
| 9月7日 | 現行採用（X-ICP） | 368.9秒まで姿勢保存、exit 137。連続5 m/s超は159.5秒 | 546.1秒まで姿勢保存、exit 137。連続5 m/s超は422.0秒 | [デフォルトYAMLの試行](../diagnostic_captures/current_20260907_20261028/README.md)・[速度の事後判定](diagnostics/F281_LATEST_XICP_20260907.md) |
| 9月7日 | 現行採用（時間窓） | 45秒prefix取得。未整列の旧構成との差6.85 mは初期姿勢合わせ後0.90 m。未延長 | 35.6秒で連続2姿勢区間の5 m/s超を検知して自動停止 | [今回の監督付き部分記録](../diagnostic_captures/current_default_temporal_20261101/environment_evidence.json) |
| 10月20日 | f281e70 | bag読了 | bag読了 | [旧構成の比較](../diagnostic_captures/pre_oct1_f281e70_20261020/FINDINGS.md) |
| 10月20日 | 現行採用（X-ICP） | 339.9秒までbag読了 | 125秒の限定試行。連続5 m/s超は75.8秒。全bag結果ではない | [LIO全区間比較](../diagnostic_captures/f281_vs_reconstruct_default_20261020/REPORT.md)・[LIVO部分記録](../diagnostic_captures/xicp_off_20261020/livo_xicp_on_125/frontend/summary.json) |
| 10月20日 | 現行採用（時間窓） | 45秒prefix取得。44秒の旧構成との差は初期姿勢合わせ後3.84 m。未延長 | 45秒prefix取得。同差10.42 m。未延長 | [今回の監督付き部分記録](../diagnostic_captures/current_default_temporal_20261101/environment_evidence.json) |

10月20日のX-ICP LIVO部分記録は、元デフォルトYAMLと比べた推定パラメータ差が `common.img_en=1` のみ。保存済み姿勢を事後にGo2W停止基準で検査すると75.8秒で5 m/sを二つの連続区間で超える。125秒まで姿勢が残っていても、75.8秒以降を正常走行として比較しない。この古い試行は現在の自動監督下で実施したものではない。[派生パラメータ](../diagnostic_captures/xicp_off_20261020/livo_xicp_on_125/params.yaml)・[既存manifest](../diagnostic_captures/xicp_off_20261020/manifest.json)。9月7日のX-ICP側も発散後の姿勢が保存されているため、完走や帰着精度を主張しない。

既存記録に該当する同条件captureはなかったため、今回、最新デフォルトYAMLから `lio.temporal.enabled=true`、`lio.xicp.enabled=false` を変更した2 bag×LIO/LIVOの4本を取得した。**いずれもbag読了結果ではない**。監督付きprefixで得た軌道・共分散・画像更新の診断値は次節にまとめる。

## 新規取得：現行採用（時間窓）の監督付きprefix

入力は上記の2 bag、バイナリと元YAMLのSHA256は「構成の定義」記載の値。各実効YAMLは最新デフォルトから時間窓ON、X-ICP OFF、LIOのみ `common.img_en=0` とした。その他の推定パラメータは共通。全runで `offline.rate=0.5`、`offline.duration=45`、診断有効、地図publish無効とした。これは**直接のオフライン実行と診断CSV保存**であり、`reconstruct_raw.sh --capture-diagnostics` のCLI経由ではない。各[manifest](../diagnostic_captures/current_default_temporal_20261101/sep07_lio_45/manifest.json)にコマンド・ハッシュ・実効パラメータ・停止条件を保存した。

| bag・モード | 保存姿勢の終端 | 監督停止理由 | LiDAR更新数／対応数の中央値 | LiDAR RMS中央値 | 視覚更新数／有効画素中央値 | 最大RSS |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| [9/7 LIO](../diagnostic_captures/current_default_temporal_20261101/sep07_lio_45/frontend/poses.csv) | 45.0秒 | 設定した45秒に到達 | 419／984 | 0.0329 m | 対象外 | 238 MiB |
| [9/7 LIVO](../diagnostic_captures/current_default_temporal_20261101/sep07_livo_45/frontend/poses.csv) | 35.7秒 | 連続2姿勢区間で5 m/s超、35.6秒に自動停止 | 327／968 | 0.0514 m | 326／3648 | 351 MiB |
| [10/20 LIO](../diagnostic_captures/current_default_temporal_20261101/oct20_lio_45/frontend/poses.csv) | 44.8秒 | 設定した45秒に到達 | 419／736 | 0.0245 m | 対象外 | 221 MiB |
| [10/20 LIVO](../diagnostic_captures/current_default_temporal_20261101/oct20_livo_45/frontend/poses.csv) | 44.7秒 | 設定した45秒に到達 | 418／729 | 0.0249 m | 418／3008 | 338 MiB |

保存済み姿勢を時刻で線形補間し、**未整列**の開始座標系で比較した位置差は次の通り。対照は各bagのf281e70 LIOの既存軌道で、9/7の30–35秒と10/20の30–44秒はその軌道がまだ速度基準で破綻する前である。10秒、20秒の位置差はすべて0.55 m以下だが、開始yaw差は移動後の未整列位置差を大きくし得る。対照軌道も独立真値ではない。

**2026-11-03の補正:** 後半の未整列差には開始yaw差の効果が大きく含まれる。20秒の位置・yawだけを一致させると、44秒の対照との差は9/7 LIO **0.90 m**、10/20 LIO **3.84 m**、10/20 LIVO **10.42 m**。9/7 LIVOは35秒で**42.80 m**。下表の未整列差6.85 m／10.78 mをそのままLIOの発散と読んだ以前の判断は強すぎた。[計算方法・全値](../diagnostic_captures/current_default_temporal_20261101/environment_evidence.json)。

[XY軌道の比較図](../diagnostic_captures/current_default_temporal_20261101/comparison.svg)には各bagの開始45秒だけを表示し、9/7 LIVOは自動停止までの区間のみ描いた。

| bag・対象 | 20秒 | 30秒 | 35秒 | 44秒 |
| --- | ---: | ---: | ---: | ---: |
| 9/7 時間窓LIO 対 f281 LIO | 0.27 m | 1.96 m | 3.62 m | 6.85 m |
| 9/7 時間窓LIVO 対 f281 LIO | 0.55 m | 15.25 m | 42.07 m | 停止済み |
| 9/7 時間窓LIO 対 LIVO | 0.45 m | 15.39 m | 42.27 m | 停止済み |
| 10/20 時間窓LIO 対 f281 LIO | 0.10 m | 3.49 m | 6.03 m | 10.78 m |
| 10/20 時間窓LIVO 対 f281 LIO | 0.02 m | 0.71 m | 5.79 m | 10.43 m |
| 10/20 時間窓LIO 対 LIVO | 0.09 m | 3.61 m | 9.84 m | 17.18 m |

4本とも有効LiDAR対応はゼロになっておらず、LiDAR RMSの最大値も0.079 m以下。視覚更新はLIVOの保存区間のほぼ全LiDAR更新時刻で行われたが、9/7 LIVOでは約25秒以降、LIOより大きく軌道が離れた。したがって**画像更新の頻度や対応画素数だけで軌道の正しさは保証できない**。最後のLiDAR更新時の位置共分散対角成分はLIOでは両bagで概ね1–1.5 m²、LIVOでは概ね10⁻⁵–10⁻⁴ m²で、軌道差に比べてLIVO側が非常に小さい。これはフィルタ内の不確かさであり、実測誤差や地形の因果効果と同一視できない。[全数値](../diagnostic_captures/current_default_temporal_20261101/comparison.json)と各runの `frontend/logs/temporal_window.csv`、LIVOの `temporal_visual.csv` から再確認できる。

4本とも `frontend/poses.csv`、監督結果、資源ログ、実効YAML、診断CSVを保存した。各runの `export/trajectory.tum` と `export/map.ply` は**保存済みprefixだけの未補正出力**である（[9/7 LIO](../diagnostic_captures/current_default_temporal_20261101/sep07_lio_45/export/README.md)、[9/7 LIVO](../diagnostic_captures/current_default_temporal_20261101/sep07_livo_45/export/README.md)、[10/20 LIO](../diagnostic_captures/current_default_temporal_20261101/oct20_lio_45/export/README.md)、[10/20 LIVO](../diagnostic_captures/current_default_temporal_20261101/oct20_livo_45/export/README.md)）。9/7 LIVOは速度基準で停止した。残る3本は当時、未整列差を安全上の懸念とみて延長しなかったが、9/7 LIOについては開始座標系を考慮した再評価でその根拠が弱くなった。全bagの帰着・地形別適性はこの4本からは評価できない。

## 完全性が確認できる既存結果

10月20日のf281e70 LIOと現行採用（X-ICP）LIOは両方bagを読み終えた。時刻合わせした未整列の位置差は中央値0.236 m、最大0.362 m、339秒で0.235 m。出発4–12秒→帰着220–230秒の15 cm以内点群重なり率は27.4%対30.2%。[比較図](../diagnostic_captures/f281_vs_reconstruct_default_20261020/comparison.png)・[全数値](../diagnostic_captures/f281_vs_reconstruct_default_20261020/comparison.json)。視点・遮蔽が異なるため、数ポイントの重なり差を絶対位置精度の優劣としない。

10月20日のf281e70 LIO/LIVOは時刻合わせ位置差の中央値0.039 m、95%点0.090 m、最大0.134 m。出発→帰着の15 cm以内重なり率は27.4/29.0%、局所面残差RMSEは0.0627/0.0483 m。一方、静止区間220–339秒のZ標準偏差は0.0023/0.0132 m、加速度biasノルム中央値は0.085/0.489 m/s²。画像で局所残差が下がっても、高さやbiasまで一様に改善したとは言えない。[全数値](../diagnostic_captures/pre_oct1_f281e70_20261020/comparison.json)。9月7日のf281e70 LIO/LIVOは両方とも終盤に発散し、出発→帰着の15 cm以内重なり率はいずれも0%。[全数値](../diagnostic_captures/pre_oct1_f281e70_20260907/comparison.json)。

## 以前の2秒時間窓プリセットは別条件

以前「現行採用」と呼んだ `config/fast_livo2/go2w_mid360_online.yaml` は、最新デフォルトを時間窓ONにしただけの設定ではない。時間窓2秒・最大6状態・`legacy_voxel`初期化・X-ICP無効に加え、IMU雑音と初期共分散なども異なる。**今回の現行採用（時間窓）に読み替えない。**

この別プリセットの9月7日LIO/LIVOはbag読了したが、出発→帰着の15 cm以内重なり率が21.7/1.6%、NN中央値が0.747/2.876 mだった。[比較値](../diagnostic_captures/adopted_online_20260907_20261028/comparison.json)。LIOは同じバイナリ・実質同じ推定条件の既存完走記録を再利用し、LIVOのみ新規に再構成した。[manifest](../diagnostic_captures/adopted_online_20260907_20261028/manifest.json)。10月20日の同プリセットLIOは235.3秒までの部分記録で帰着位置が約116 mずれ、LIVOは122.9秒に連続5 m/s超で自動停止した。[LIO比較](../diagnostic_captures/temporal_vs_legacy_20261020/REPORT.md)・[LIVO監督結果](../diagnostic_captures/adopted_online_20261020_20261101/livo_prefix_125s/supervisor_result.json)。この失敗を今回の6秒時間窓設定の成績として扱わない。

## 残る検証

1. 今回の時間窓prefixで約25–35秒に生じたLIO/LIVOの軌道分岐を、初期座標系の差を除いたうえで、LiDAR対応の配置・法線、視覚残差、IMUの各診断値と場面に時刻同期して調べる。9/7 LIVOの発散したprefixを同条件の全bag replayへ延長しない。9/7 LIOの延長が必要なら、今回の健全性を再評価して監督付きで行う。
2. X-ICP側の9月7日と10月20日LIVOも保存済みの発散前prefixから原因を調べる。未整列軌道差、共分散、対応点の幾何、画像の有効画素・残差、IMU飽和を同時刻で対応させる。独立真値がない範囲では環境適性を限定的に主張する。
