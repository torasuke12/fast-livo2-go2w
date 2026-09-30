# raw_20260907_114335 の全走行再構成（2026-09-30）

FAST-LIVO2の時間窓LIOをフロントエンドに使い、再訪点群から求めた拘束で軌跡を最適化し、地図を再投影するオフライン経路を追加した。
約648秒を最後まで処理し、出発・帰着の部屋の壁と天井が重なる結果を保存した。
これは再訪を利用するオフライン再構成の改善であり、従来のオンライン推定器単体でドリフトが解消したという結果ではない。

## 完成した成果物

- [比較図](../reconstruction_results/raw_20260907_114335/comparison.png)：青が出発時、橙が帰着時。右列は走行中の高さ。
- [地図 map.ply](../reconstruction_results/raw_20260907_114335/map.ply)：909,037点、約42 MiB。
- [軌跡 trajectory.tum](../reconstruction_results/raw_20260907_114335/trajectory.tum)：1,215姿勢。
- [部屋の比較値](../reconstruction_results/raw_20260907_114335/room_metrics.json)、[最適化の集計](../reconstruction_results/raw_20260907_114335/summary.json)。
- 数値・試験・入力hashの小さい記録は [diagnostics/offline-loop-20260930](diagnostics/offline-loop-20260930/)。大きな地図・実行ログはgit対象外。

## 再実行

ホストのリポジトリ直下から実行する。今回の環境には専用venvとバイナリを準備済み。
出力先には新しいディレクトリを指定する。

```bash
bash scripts/fast_livo2/reconstruct_offline.sh \
  bags/raw_20260907_114335 reconstruction_results/new_run --return-to-start
```

別環境ではまず依存を用意する。

```bash
python3 -m venv .venv-reconstruction
.venv-reconstruction/bin/pip install -r scripts/fast_livo2/reconstruction-requirements.txt

# ROSコンテナ内
cd /external/humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select fast_livo2 --symlink-install --cmake-args -DBUILD_TESTING=ON
```

`fast-livo2-go2w:latest` をホストから起動し、bagとソースをread-only、出力先だけをread-writeでマウントする。
`FAST_LIVO2_MAP_PYTHON`、`FAST_LIVO2_DOCKER_IMAGE` でPythonとイメージを指定できる。
ROSコンテナ内から呼ぶ場合は、その環境にも上記Python依存が必要。

`--livo` は比較用。今回の既存画像設定では改善しなかったため、対象bagの完成結果にはLIOを採用した。
`--rate` は使わない。bagを直接読み、同じFAST-LIVO2のcallback、同期、IMU処理、状態推定を順に呼び出す。
再生速度やDDS配送の欠落に依存せず、処理時間に応じて進行する。

## 保存済み地図をRVizで表示

ROSコンテナ内の2端末で実行する。再構成やbag再生は不要。

```bash
# 端末1
cd /external
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=90
unset CYCLONEDDS_URI
python3 scripts/fast_livo2/view_reconstruction.py \
  reconstruction_results/raw_20260907_114335
```

```bash
# 端末2。同じROS_DOMAIN_IDを使う。
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=90
unset CYCLONEDDS_URI
rviz2 -d /external/config/fast_livo2/fast_livo2.rviz --ros-args -p use_sim_time:=false
```

新規再実行の結果を表示する場合は `reconstruction_results/new_run/optimized` を指定する。
配信先は `/Laser_map` と `/path`、固定フレームは `camera_init`。点群はXYZで、RVizでは高さで着色できる。
実際のROS購読で909,037点と1,215姿勢、有限値、終了コード0を確認した。GUI自体の目視試験は行っていない。

## 実装

1. `fastlivo2_offline`：rosbag2 readerから入力を取得し、既存推定器に渡す。約0.5秒ごとに歪み補正済みIMU座標点群と画像融合後／LIO更新後の姿勢を保存する。画像OFFならカメラを読み込まない。topic型検査、既存出力の上書き拒否、異常・途中終了の記録を行う。
2. オフライン用設定：時間窓2秒、最大6状態、legacy voxel初期化。静止初期化の初期姿勢・位置共分散を各0.0001、速度を0.001とする。既存X-ICP、較正、過去の初期化比較の経路も残る。複数設定を合わせた候補の全走行結果であり、各変更の単独改善量は未分離。
3. 部分地図：通常は2.5秒単位。区間内の高さ変化が0.3 mまたは回転変化が20度を超える場合は個々の取得へ分割する。階段中の誤った過去姿勢修正を、長い剛体部分地図に焼き込むことを避ける。
4. 局所照合：直前の部分地図群へpoint-to-plane ICPで照合する。姿勢の位置差が推定速度と大きく食い違うときは、速度からの移動を**探索の初期値**にする。幾何が移動を決め、速度に最終位置を固定しない。
5. 再訪：双方向の重なり、面残差、法線分布、階の整合を検査する。帰着の部屋は慣性姿勢から45度以内で位置・向きを探索し、3つ以上の異なる時刻、5秒以上の間隔で一致する補正を要求する。競合する同程度の仮説は採用しない。
6. 軌跡最適化：SO(3)対数による回転残差と疎な非線形最小二乗を使う。IMUの重力方向を弱い事前として使い、yawと位置は推定する。局所登録の重みを面残差・重なり率で調整し、照合失敗の移動は弱い拘束とする。部屋を閉じた後に往復区間を再探索して追加する。
7. 地図出力：補正済み姿勢から全保存スキャンを再投影する。古い累積 `/Laser_map` に点を足して見かけだけ合わせる処理ではない。開始・終了のロボット位置が等しいという因子も入れない。

点群処理・登録情報の計算にはOpen3Dを使用した。
関連する公式説明：[point cloud registration / pose graph](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html)。
最終のグラフ最適化は本スクリプトのSO(3)対数残差とSciPyで実装し、Open3Dのglobal_optimizationをそのまま採用したものではない。

## 実データの結果

入力LiDAR **6,481件**、IMU **129,619件**を全件読み込み、推定時刻**648.0996406秒**まで到達。
初期化後の更新6,448回、保存スキャン1,215、部分地図395、再訪拘束404組（うち帰着の部屋15組）。
最適化2段階はいずれも相対コスト減少による収束判定で終了した。

部屋の評価は4–12秒と632–648秒。評価時に追加の剛体位置合わせはしていない。
ICPの当てはめに使わなかった奇数番の点を使用した。ただしこれらはループ採否にも使うので、完全に独立した精度検定ではない。

| 指標 | 地図補正前 | 最終結果 |
|---|---:|---:|
| 帰着→出発点群の最近傍距離中央値 | 4.790 m | 0.270 m |
| 出発→帰着点群の最近傍距離中央値 | 6.187 m | 0.705 m |
| 帰着→出発、15 cm以内の対応率 | 0% | 41.7% |
| 出発→帰着、15 cm以内の対応率 | 0% | 24.4% |
| 帰着→出発、対応面のRMSE | 対応なし | **3.14 cm** |
| 出発→帰着、対応面のRMSE | 対応なし | **3.50 cm** |
| 640秒の推定z | 約−5.95 m | 約+0.044 m |

面RMSEは最近傍距離15 cm以内の対応に限定した点対平面残差であり、全点や絶対軌跡の3 cm精度を意味しない。
視点・遮蔽・家具・近距離の戻り点が違うため、部屋全体の全点が一致した結果でもない。
共通に見える壁・天井の重なりは比較図で確認できる。
最終のロボット位置差は約1.14 mで、実測点群から得た異なる停止位置を許している。この値を閉ループ誤差とは呼ばない。

上階の代表値は140秒3.47 m、300秒3.42 m、480秒3.53 m、530秒3.72 m。
往復廊下・階段・部屋の形状は改善したが、上階にも数十cm規模の高さ変動が残る。真値軌跡・測量した階高がないため、全走行ATEや最小誤差の保証はできない。

## 比較・失敗も保存

- 親の `a/main/feat/xicp-degeneracy/feat/temporal-lio-window` は全て `7c261e9`、内部の3ブランチは全て `f281e70`。実装差は既存の未コミット変更とdocsにあり、名前だけのcheckoutでは別実装にならない。既存変更を保持した。
- 従来voxel-plane LIOのX-ICP無効比較は160秒以降に発散し、335秒付近までの途中結果を不採用として保存した。
- 採用した時間窓LIOは全区間の保存姿勢で最大速度1.744 m/s、最大accel biasノルム0.281 m/s²。ただし地図補正前には下降後の高さが約−6 mとなった。
- 同じ時間窓設定のLIVOは300秒のz約−9.99 mで悪化。約311秒で打ち切り、不完全実行として保存した。カメラ内参はbagのCameraInfoと一致した。悪化原因を外部較正だけに断定していない。
- 自由なFPFH登録は矩形の部屋の90度対称性を誤採用し、時刻間の一致だけでは排除できなかった。最終版は慣性方向と整合する探索を使う。
- 初期グラフ試行の約180度回転誤差、一定重みで下降の位置飛びを信頼した試行、固定2.5秒の部分地図の階高過大は不採用。最終版は回転対数残差、登録品質の重み、急変区間の分割を使う。

試行は `diagnostic_captures/offline-20260930/`。
最終検証元は `window-lio/` と、キャッシュを使わず全登録を計算した `optimized-lio-v8/`。
古いv2–v7の結果を完成地図と混同しない。

## 検証と再現性

- C++ CTest 9/9。Python 27/27（新規の登録・廊下退化・グラフ向き・時間的合意・大回転の回帰5件を含む）。
- 直接bag入力の先頭87更新は既存のROS再生ログと時刻が一致。状態と行列の差は丸め誤差の範囲。
- 最終オフラインバイナリはtopic型検査とROSログ先の修正後、通常ユーザー権限の12秒実行で正常終了。全走行の先頭87更新と、solve_ms以外の全数値列・時刻・statusが完全一致した。
- 元bagの先頭8秒だけを切り出した別bagで、ホストのラッパー→Docker→FAST-LIVO2→グラフ→PLY/TUM保存を通して成功。
- 保存地図の実ROS配信・購読で909,037点、1,215姿勢、有限値、終了コード0を確認。
- 全走行後にオフライン入口へ加えた変更はtopic型検査とROSログ保存先。最終入口バイナリでの全648秒再実行はしていない。推定器本体のコード・ライブラリはその間変更していない。
- 通常mappingバイナリは従来と同じ `f51c5008…623e227`。新offlineバイナリは `dfb0e659…108acd`。
- raw DB SHA256: `67aa09c0ee6f1ce316fc0a26e4b4c36fefad30828fecaebefdcef325098139ba`。

`--reuse-registrations` は解析用で、姿勢・点群hash・部分地図構成が一致する保存登録だけを再利用する。
通常の再構成ラッパーは毎回新規に登録を計算する。`--return-to-start` を指定して帰着の照合を確認できなければ、成功扱いで終了しない。
