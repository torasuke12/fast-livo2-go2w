# X-ICP 導入時の検証記録（2026-09-27）

## 条件

- 親リポジトリ・`humble_ws/src/fast-livo2` ともに `feat/xicp-degeneracy`。
- 既存の未コミット変更は保持。比較用バイナリ・設定は今回の編集前に保存。
- `fast-livo2-go2w:latest` コンテナ内で Release ビルド、既存の `humble_ws/install` に反映済み。
- bag: `raw_20260907_114335`、再生速度0.5倍、rawトピックのみ、RELIABLE QoS、deterministic処理。
- RVizなし。各再生のDDSドメインとログを分離し、既存の作業ログは上書きしていない。
- 比較バイナリ SHA256: `5dbcb9ee9955cab3dbac89cb1030eea27d1662c05cc3412555c13d5ea9021330`
- 導入後バイナリ SHA256: `88740378e9b283028f539197dab03ecbe31cb9726cacb95d64be47ac6c380dc5`

## テスト結果

CTest 3/3 成功:

1. `xicp_geometry_and_constrained_filter`（9グループの数値・幾何・入力検証）
2. `imu_propagation_no_new_samples`
3. `voxel_map_initialization`

12秒のLIO比較では、編集前バイナリと導入後の `lio.xicp.enabled: false` の `mat_out.txt` が88行すべて完全一致。
`voxel_plane` + X-ICPでも12秒再生し、初期化完了、86状態の出力、非有限値なしを確認した。voxel-planeでの長時間性能は未評価。

## ikd-tree / fastlio_knn での160秒比較

145〜160秒の前進速度（IMU/body x方向、m/s）の範囲を含む結果:

| モード | X-ICP | 出力状態数 | 約150秒の前進速度 | 145秒以降の最小前進速度 |
|---|---|---:|---:|---:|
| LIO | なし・編集前 | 1568 | +1.384 | +0.715 |
| LIO | あり | 1568 | +1.158 | +0.956 |
| LIVO | なし・編集前 | 6263 | +1.252 | +1.007 |
| LIVO | あり | 6263 | +1.258 | +0.463 |

いずれも160秒まで数値異常なし。IMUの最大間隔は8.34 msで、10 msを超える欠落はなかった。
LIVOの状態数にはLiDAR更新と画像更新の両方が含まれる。

X-ICP有効時は、LIOで6170反復、LIVOで12056反復の診断を記録。両モードでFULL/PARTIAL/NONEが発生しており、常時無制約の状態ではない。
LIOの最弱並進方向はFULL 3964回、PARTIAL 2182回、NONE 24回。
制約ゲインの誤差 `|C L - D|` の最大はLIOで `8.54e-17`、LIVOで `1.61e-15`。数値解の棄却は0回。
追加区間（判定・制約解法・状態反映、ログI/Oを除く）の中央値／95%点はLIOで0.95／2.13 ms、LIVOで0.38／1.16 ms。

### この検証で言えること・言えないこと

- 現在の設定ではX-ICPの有無どちらも最初のUターン後を前進のまま通過した。したがって、**後退問題の改善を実証した比較ではない**。
- LIVOでは有効時の前進速度が159.4秒付近で約0.46 m/sまで落ち、無効時より大きく変動した。全区間の精度改善は主張しない。
- 正解軌跡・正解速度による誤差評価は未実施。300秒以降のUターンや全bag完走も未検証。
- LIVOの一部の分割フレームでは対応点が35点未満で全方向NONEとなった。MID360に対する閾値調整と疎フレームの扱いは今後の評価対象。
- 再生時間に到達してSIGINT停止した際、編集前・導入後の両方で既存の終了時SIGSEGV（exit -11）が再現した。再生中の異常終了とは区別し、今回は変更していない。

## 保存物

詳細ログ、比較用バイナリ、実行スクリプト、導入前の差分は次に保存:

`/tmp/fast-livo2-xicp-impl.oy1yo0/`

主なディレクトリ: `baseline_lio160/`, `enabled_lio160/`, `baseline_livo160/`, `enabled_livo160/`, `enabled_voxel_lio12/`。
各ディレクトリに `summary.json`, `node.log`, `mat_out.txt`, `mat_pre.txt`, `imu.txt`、有効時は `xicp_diagnostic.csv` を保存。
これは一時領域のため、長期保存が必要なら別途コピーすること。
