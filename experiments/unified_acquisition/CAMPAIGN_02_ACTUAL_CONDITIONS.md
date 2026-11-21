# campaign_02 実際に使用した実験条件

作成日: 2026-11-20（JST）。対象: `experiments/unified_acquisition/campaign_02`。

この文書は操作説明書ではなく、固定パッケージの provenance と各ステージに保存された params.yaml・command.json・result.json を基にした実行条件の記録である。現在の作業ツリーの設定を実行条件とみなさない。取得結果・固定パッケージは変更していない。

## 1. 目的と比較の範囲

- 9月7日・10月20日のデータで、基準版／現在版と画像融合の有無を比較する。精度改善のためのチューニングは今回の目的に含めない。
- 出発・帰着の一致は評価対象としない。
- 主比較は A/C の8条件、画像更新の影響を切り分ける B の4条件を加えて計12条件。
- 基準版は元のFAST-LIVO2に近い比較対象を意図するが、Go2W対応と実験用パッチを含む。upstream無改変版でも、f281e70チェックアウトそのままでもない。
- 現在版は2秒採用プリセット。HEADだけではコードを識別できないため、ソースのスナップショット・パッチ・バイナリハッシュ・package IDを識別子とする。

## 2. 入力bag

| ID | bag | 記録時間（秒） | LiDAR件数 | IMU件数 | 画像件数 |
|---|---|---:|---:|---:|---:|
| oct20 | `raw_20261020_043530` | 339.911921056 | 3399 | 67986 | 10163 |
| sep07 | `raw_20260907_114335` | 648.066271136 | 6481 | 129619 | 19428 |

入力ファイルのSHA256（固定時に記録）:

- `oct20/metadata.yaml`: `b13f8cbfe62226f62260a4a3e35f2f7dc14b7a29d17b14670d26743b94d24102`
- `oct20/raw_20261020_043530_0.db3`: `5d201c0c37bb5a6a46dff61c1ea5308c730f167db112b484753c99aaec85a674`
- `sep07/metadata.yaml`: `8be7fe8db708c433e8c7c831710b1b518abbcd25e7acfa8e6db630a4d2fe4469`
- `sep07/raw_20260907_114335_0.db3`: `67aa09c0ee6f1ce316fc0a26e4b4c36fefad30828fecaebefdcef325098139ba`

## 3. 構成と画像条件

| モード | 画像経路 | LiDAR更新タイミング | 画像による状態更新 |
|---|---|---|---|
| A | 無効 | scan_end | 無効（LIO） |
| B | 有効 | image_time | 無効。画像フロントエンドの計算は行う |
| C | 有効 | image_time | 有効（LIVO） |

B/Cの画像採用間隔は0.1秒、許容値は0.0001秒。入力画像を全件10 Hzとして記録したという意味ではなく、採用間隔の条件である。基準版にも共通の画像採用条件を適用した。同一版のA/B/Cは同じバイナリを使う。

Bでは `EXPERIMENT_VISUAL_OFF=1` を実行環境に渡す。基準版は画像候補更新後に状態・共分散・露光等を復元し、現在版は `visual_factor_enabled=false` と露光の持ち越し抑止を使う。Bの画像候補残差は、状態へ適用された画像更新の残差と同一視しない。

A対Cは画像融合だけでなく更新時刻も異なる。画像融合を有効にした総合的影響の比較はB対Cを中心とする。

| 項目 | 基準版 | 現在版 |
|---|---|---|
| ソースの基点 | f281e70 + 実験用パッチ | 同じHEADに未コミット変更を含む作業ソースを固定 + 実験用パッチ |
| 推定経路 | voxel_plane_iekf | temporal_2s |
| 対応方式設定 | voxel_plane | voxel_plane |
| temporal window | なし | 2秒、最大6状態、keyframe 0.5秒 |
| temporal初期化 | 該当なし | legacy_voxel |
| X-ICP | 経路なし | 無効 |

**両版は推定方式以外にも差がある。** 特に `imu.gyr_cov` は基準版0.1／現在版0.0001、`imu.gravity_est_en` はtrue／false、初期位置・姿勢共分散は1.0／0.0001、初期速度共分散は1.0／0.001、`preprocess.filter_size_surf` は0.5／0.1。したがって版間差を単一ファクターの効果と断定しない。全設定は付録、固定テンプレート間の差は [parameter_difference.json](frozen_20261111_swap_telemetry/parameter_difference.json) を参照する（同JSONは実行時の診断設定上書き前）。

## 4. 共通の入力・収集・資源条件

- rosbagの直接読み込み経路 `fastlivo2_offline` を使用。DDS経由のbag playではない。
- 入力倍率0.1。遅れを取り戻す高速投入はしないため、実時間はbag時間の10倍を超え得る。
- 各条件で45秒prefix。正常完了した条件のみ、bag先頭からfullを別実行する。prefixから状態を引き継がない。
- fullの `offline.duration=0` は全区間指定。`offline.rate=0`（無制限速度）とは異なる。
- 同時実行1本。数値計算スレッド1、Docker CPU quotaは6論理CPU相当。
- Dockerメモリ上限10 GiB、memory-swapも10 GiB（追加swapを許可しない）。RSS 9 GiB以上またはホストMemAvailable 3 GiB未満で停止。
- 監督間隔1秒。画像・点群等のkeyframe収集間隔0.5秒。
- カメラ・外部パラメータ・時刻オフセットは共通。640×480、RadTan、fx=607.06781、fy=607.022583、cx=313.942291、cy=253.383209、歪み係数0、各センサ時刻オフセット0。両bagのCameraInfo確認記録は [input_compatibility.json](frozen_20261111_swap_telemetry/input_compatibility.json)。外部変換行列は付録に掲載。

## 5. 監督条件と途中変更

主な停止条件は、非有限姿勢・状態、推定位置差による速度10 m/s超1回または5 m/s超2区間連続、異常ジャンプ、LiDAR有効対応ゼロ3更新連続、残差RMS 0.2超、残差の急増、voxel/index overflow、1更新10秒超、初期化後の進捗30秒停止、初期化タイムアウト、メモリ・ホスト負荷の圧迫。

実時間上限は `max(対象秒数 / rate * 2, 対象秒数 * 9) + 300` 秒。閾値の厳密な実装は各パッケージの `runtime/experiment/supervisor.py` を参照する。発散判定でその条件を終了し次条件へ進む。資源・起動等の失敗は一括処理を停止する。停止後にコンテナの終了・削除を確認する。

初期パッケージではホスト全体のSwapFree減少も停止条件だった。現在版A（10月20日）のfullがこれで2回停止したため、2026-11-11にswap使用量を記録のみに変更した。RSS・MemAvailable・負荷監視およびコンテナの追加swap禁止は維持した。

元パッケージを上書きせず新パッケージへ移行した。移行記録では推定器ソース・バイナリ・パラメータは不変で、変更対象はsupervisor、provenance、validation（および再生成したlock）である。既存結果のpackage IDは保持し、campaignのpackage IDだけを移行した。旧prefixの正常結果を再利用してfullを先頭から再実行している。

- 旧: [frozen_20261108](frozen_20261108/provenance.json)、package ID `0ca88957a468225b9ca1a6d5d9bf49c6d6fff7edbf635de322417c72492467ce`
- 新: [frozen_20261111_swap_telemetry](frozen_20261111_swap_telemetry/provenance.json)、package ID `6231ef7a8b62176d887664e22caa669a3240f1467f638a4607c2ce41f7acc151`

変更台帳: [package_transition_20261111_swap_telemetry.json](campaign_02/package_transition_20261111_swap_telemetry.json)

## 6. 実行コードの識別

- baseline基点: `f281e70ce50a7126b4a8d6bb38df9d95a98bc0df`
- current HEAD（これだけでは識別不可）: `f281e70ce50a7126b4a8d6bb38df9d95a98bc0df`
- Docker image: `sha256:4a7f294e39a8ad5dbe23223ceb0124752e52c8caa473c5253a2c05999f764a47`
- baseline binary SHA256: `1d5edf92d261b5594a015c34deb343334924c7e3d63cfb5f15bb3773c6959b13`
- current binary SHA256: `60a8b80de98a8f555c6ee0a71cf2bf44664bcf84045dd0f6405b56444973d56e`

ソース実体は固定パッケージの `sources/`、適用パッチは `patches/`、ファイル別ハッシュは `lock.json`。currentの差分はprovenance内の `current_git_diff` にも保存されている。

## 7. 各条件の実際の到達区間と使用パッケージ

旧／新は上記package IDに対応。時刻はresult.jsonの最終有限姿勢に記録された推定器時刻であり、bagメタデータのdurationと完全一致するとは限らない。

| 条件 | prefixパッケージ | fullパッケージ | 最終段階 | 状態・理由 | 最終時刻（秒） | 証拠 |
|---|---|---|---|---|---:|---|
| oct20_baseline_A | 旧 | 旧 | full | diverged / large_lidar_residual | 160.190 | [result.json](campaign_02/oct20_baseline_A/full/result.json) |
| oct20_baseline_B | 旧 | 旧 | full | diverged / large_lidar_residual | 171.040 | [result.json](campaign_02/oct20_baseline_B/full/result.json) |
| oct20_baseline_C | 旧 | 旧 | full | diverged / large_lidar_residual | 172.041 | [result.json](campaign_02/oct20_baseline_C/full/result.json) |
| oct20_current_A | 旧 | 新 | full | complete / bag_exhausted | 339.900 | [result.json](campaign_02/oct20_current_A/full/result.json) |
| oct20_current_B | 新 | 新 | full | complete / bag_exhausted | 339.824 | [result.json](campaign_02/oct20_current_B/full/result.json) |
| oct20_current_C | 新 | 新 | full | diverged / pose_divergence | 122.206 | [result.json](campaign_02/oct20_current_C/full/result.json) |
| sep07_baseline_A | 新 | 未実施 | prefix | diverged / pose_divergence | 0.399 | [result.json](campaign_02/sep07_baseline_A/prefix/result.json) |
| sep07_baseline_B | 新 | 未実施 | prefix | diverged / large_lidar_residual | 0.468 | [result.json](campaign_02/sep07_baseline_B/prefix/result.json) |
| sep07_baseline_C | 新 | 未実施 | prefix | diverged / large_lidar_residual | 0.468 | [result.json](campaign_02/sep07_baseline_C/prefix/result.json) |
| sep07_current_A | 新 | 新 | full | complete / bag_exhausted | 648.100 | [result.json](campaign_02/sep07_current_A/full/result.json) |
| sep07_current_B | 新 | 新 | full | complete / bag_exhausted | 648.018 | [result.json](campaign_02/sep07_current_B/full/result.json) |
| sep07_current_C | 新 | 新 | full | complete / bag_exhausted | 648.018 | [result.json](campaign_02/sep07_current_C/full/result.json) |

全12条件の判定が終了。完走5条件、full中の発散停止4条件、prefix停止・full未実施3条件。完走は精度保証を意味しない。

再試行として別保存した取得:

- [result.json](campaign_02/oct20_current_A/full_swap_stopped_01/result.json): `oct20_current_A/full_swap_stopped_01`、旧、swap_rising、実時間 4013.112秒。主比較のfullに混ぜない。
- [result.json](campaign_02/oct20_current_A/full_swap_stopped_02/result.json): `oct20_current_A/full_swap_stopped_02`、旧、swap_rising、実時間 2650.298秒。主比較のfullに混ぜない。

## 8. 証拠の参照方法と付録の扱い

各 `campaign_02/<条件>/<prefix|full>/` に以下を保存している。

- `params.yaml`: 起動時に渡したパラメータ（実行時の診断設定・区間指定を含む）。
- `frontend/effective_parameters.txt`: ノードが報告したパラメータ。デフォルト値確認にも使う。数値表記は丸められるため、精密値はYAMLとソースも確認する。
- `command.json`: Docker引数、環境変数、マウント、実行バイナリを含む起動コマンド。Bの無効化はYAMLだけでは完結しない。
- `result.json`: package ID、binary/params SHA256、停止理由、最終状態。
- `resource.csv`: 実時間・推定器時刻・RSS・MemAvailable・SwapFree。

付録は10月20日の各構成の**実行時full用params.yaml**を転記したもの。9月7日は同構成について `lio.diagnostics.detail.end_sec` の対象区間のみが異なる（現在版）。prefixは `offline.duration=45`、診断終了区間45秒（現在版）。実行済み全23ステージについて、同構成からこれら区間指定を除いたYAMLが一致すること、およびparams.yamlのSHA256がresult.jsonと一致することを作成時に確認した。

設定項目が存在しても、その経路が無効なら推定に使われるとは限らない。実効パラメータ・環境変数・ソースを併せて解釈する。

## 付録: 実行時パラメータ全文（6構成）


### baseline_A

出典: [params.yaml](campaign_02/oct20_baseline_A/full/params.yaml)

```yaml
laserMapping:
  ros__parameters:
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      img_topic: /camera/camera/color/image_raw
      img_en: 0
      lidar_en: 1
      ros_driver_bug_fix: false
    camera:
      cam_model: RadTan
      cam_width: 640
      cam_height: 480
      scale: 1.0
      cam_fx: 607.06781
      cam_fy: 607.022583
      cam_cx: 313.942291
      cam_cy: 253.383209
      cam_d0: 0.0
      cam_d1: 0.0
      cam_d2: 0.0
      cam_d3: 0.0
      cam_d4: 0.0
    extrin_calib:
      extrinsic_T:
      - -0.011
      - -0.02329
      - 0.04412
      extrinsic_R:
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      Pcl:
      - 0.0175
      - -0.1138
      - -0.112
      Rcl:
      - 0.0
      - -1.0
      - 0.0
      - 0.313627591
      - 0.0
      - -0.949546068
      - 0.949546068
      - 0.0
      - 0.313627591
    time_offset:
      imu_time_offset: 0.0
      img_time_offset: 0.0
      lidar_time_offset: 0.0
      exposure_time_init: 0.0
    preprocess:
      hilti_en: false
      point_filter_num: 1
      filter_size_surf: 0.5
      lidar_type: 8
      scan_line: 4
      blind: 0.5
      feature_extract_enabled: false
    vio:
      max_iterations: 5
      outlier_threshold: 500.0
      img_point_cov: 1000.0
      patch_size: 8
      patch_pyrimid_level: 4
      grid_size: 5
      grid_n_height: 17
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: true
      inv_expo_cov: 0.1
      image_interval_sec: 0.1
    imu:
      imu_en: true
      imu_int_frame: 10
      acc_cov: 0.1
      gyr_cov: 0.1
      gravity_est_en: true
      ba_bg_est_en: true
      stationary_gyr_mean_threshold: 0.1
      stationary_gyr_std_threshold: 0.02
      stationary_acc_std_ratio_threshold: 0.05
      initial_rotation_cov: 1.0
      initial_position_cov: 1.0
      initial_velocity_cov: 1.0
      initial_gyro_bias_cov: 0.0001
      initial_accel_bias_cov: 0.001
    lio:
      max_iterations: 3
      dept_err: 0.02
      beam_err: 0.05
      min_eigen_value: 0.0001
      voxel_size: 0.5
      neighbor_voxel_radius: 1
      correspondence_mode: voxel_plane
      knn_num_match_points: 5
      knn_max_neighbor_sq_dist: 5.0
      knn_plane_threshold: 0.1
      knn_residual_score_threshold: 0.9
      knn_map_voxel_size: 0.5
      knn_laser_point_cov: 0.0003
      gravity_norm_constraint: true
      max_state_cov_for_matching: 0.01
      max_layer: 2
      max_points_num: 100
      layer_init_num:
      - 5
      - 5
      - 5
      - 5
      - 5
    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0
    uav:
      imu_rate_odom: false
      gravity_align_en: true
    publish:
      dense_map_en: true
      map_en: false
      map_publish_interval: 10
      map_voxel_size: 0.5
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0
    evo:
      seq_name: go2w
      pose_output_en: false
    pcd_save:
      pcd_save_en: false
      type: 0
      colmap_output_en: false
      filter_size_pcd: 0.15
      interval: -1
    image_save:
      img_save_en: false
      interval: 1
    debug:
      plot_time: -10.0
      frame_cnt: 6
    offline:
      rate: 0.1
      duration: 0.0
      keyframe_interval: 0.5
```

### baseline_B

出典: [params.yaml](campaign_02/oct20_baseline_B/full/params.yaml)

```yaml
laserMapping:
  ros__parameters:
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      img_topic: /camera/camera/color/image_raw
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: false
    camera:
      cam_model: RadTan
      cam_width: 640
      cam_height: 480
      scale: 1.0
      cam_fx: 607.06781
      cam_fy: 607.022583
      cam_cx: 313.942291
      cam_cy: 253.383209
      cam_d0: 0.0
      cam_d1: 0.0
      cam_d2: 0.0
      cam_d3: 0.0
      cam_d4: 0.0
    extrin_calib:
      extrinsic_T:
      - -0.011
      - -0.02329
      - 0.04412
      extrinsic_R:
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      Pcl:
      - 0.0175
      - -0.1138
      - -0.112
      Rcl:
      - 0.0
      - -1.0
      - 0.0
      - 0.313627591
      - 0.0
      - -0.949546068
      - 0.949546068
      - 0.0
      - 0.313627591
    time_offset:
      imu_time_offset: 0.0
      img_time_offset: 0.0
      lidar_time_offset: 0.0
      exposure_time_init: 0.0
    preprocess:
      hilti_en: false
      point_filter_num: 1
      filter_size_surf: 0.5
      lidar_type: 8
      scan_line: 4
      blind: 0.5
      feature_extract_enabled: false
    vio:
      max_iterations: 5
      outlier_threshold: 500.0
      img_point_cov: 1000.0
      patch_size: 8
      patch_pyrimid_level: 4
      grid_size: 5
      grid_n_height: 17
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: true
      inv_expo_cov: 0.1
      image_interval_sec: 0.1
    imu:
      imu_en: true
      imu_int_frame: 10
      acc_cov: 0.1
      gyr_cov: 0.1
      gravity_est_en: true
      ba_bg_est_en: true
      stationary_gyr_mean_threshold: 0.1
      stationary_gyr_std_threshold: 0.02
      stationary_acc_std_ratio_threshold: 0.05
      initial_rotation_cov: 1.0
      initial_position_cov: 1.0
      initial_velocity_cov: 1.0
      initial_gyro_bias_cov: 0.0001
      initial_accel_bias_cov: 0.001
    lio:
      max_iterations: 3
      dept_err: 0.02
      beam_err: 0.05
      min_eigen_value: 0.0001
      voxel_size: 0.5
      neighbor_voxel_radius: 1
      correspondence_mode: voxel_plane
      knn_num_match_points: 5
      knn_max_neighbor_sq_dist: 5.0
      knn_plane_threshold: 0.1
      knn_residual_score_threshold: 0.9
      knn_map_voxel_size: 0.5
      knn_laser_point_cov: 0.0003
      gravity_norm_constraint: true
      max_state_cov_for_matching: 0.01
      max_layer: 2
      max_points_num: 100
      layer_init_num:
      - 5
      - 5
      - 5
      - 5
      - 5
    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0
    uav:
      imu_rate_odom: false
      gravity_align_en: true
    publish:
      dense_map_en: true
      map_en: false
      map_publish_interval: 10
      map_voxel_size: 0.5
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0
    evo:
      seq_name: go2w
      pose_output_en: false
    pcd_save:
      pcd_save_en: false
      type: 0
      colmap_output_en: false
      filter_size_pcd: 0.15
      interval: -1
    image_save:
      img_save_en: false
      interval: 1
    debug:
      plot_time: -10.0
      frame_cnt: 6
    offline:
      rate: 0.1
      duration: 0.0
      keyframe_interval: 0.5
```

### baseline_C

出典: [params.yaml](campaign_02/oct20_baseline_C/full/params.yaml)

```yaml
laserMapping:
  ros__parameters:
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      img_topic: /camera/camera/color/image_raw
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: false
    camera:
      cam_model: RadTan
      cam_width: 640
      cam_height: 480
      scale: 1.0
      cam_fx: 607.06781
      cam_fy: 607.022583
      cam_cx: 313.942291
      cam_cy: 253.383209
      cam_d0: 0.0
      cam_d1: 0.0
      cam_d2: 0.0
      cam_d3: 0.0
      cam_d4: 0.0
    extrin_calib:
      extrinsic_T:
      - -0.011
      - -0.02329
      - 0.04412
      extrinsic_R:
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      Pcl:
      - 0.0175
      - -0.1138
      - -0.112
      Rcl:
      - 0.0
      - -1.0
      - 0.0
      - 0.313627591
      - 0.0
      - -0.949546068
      - 0.949546068
      - 0.0
      - 0.313627591
    time_offset:
      imu_time_offset: 0.0
      img_time_offset: 0.0
      lidar_time_offset: 0.0
      exposure_time_init: 0.0
    preprocess:
      hilti_en: false
      point_filter_num: 1
      filter_size_surf: 0.5
      lidar_type: 8
      scan_line: 4
      blind: 0.5
      feature_extract_enabled: false
    vio:
      max_iterations: 5
      outlier_threshold: 500.0
      img_point_cov: 1000.0
      patch_size: 8
      patch_pyrimid_level: 4
      grid_size: 5
      grid_n_height: 17
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: true
      inv_expo_cov: 0.1
      image_interval_sec: 0.1
    imu:
      imu_en: true
      imu_int_frame: 10
      acc_cov: 0.1
      gyr_cov: 0.1
      gravity_est_en: true
      ba_bg_est_en: true
      stationary_gyr_mean_threshold: 0.1
      stationary_gyr_std_threshold: 0.02
      stationary_acc_std_ratio_threshold: 0.05
      initial_rotation_cov: 1.0
      initial_position_cov: 1.0
      initial_velocity_cov: 1.0
      initial_gyro_bias_cov: 0.0001
      initial_accel_bias_cov: 0.001
    lio:
      max_iterations: 3
      dept_err: 0.02
      beam_err: 0.05
      min_eigen_value: 0.0001
      voxel_size: 0.5
      neighbor_voxel_radius: 1
      correspondence_mode: voxel_plane
      knn_num_match_points: 5
      knn_max_neighbor_sq_dist: 5.0
      knn_plane_threshold: 0.1
      knn_residual_score_threshold: 0.9
      knn_map_voxel_size: 0.5
      knn_laser_point_cov: 0.0003
      gravity_norm_constraint: true
      max_state_cov_for_matching: 0.01
      max_layer: 2
      max_points_num: 100
      layer_init_num:
      - 5
      - 5
      - 5
      - 5
      - 5
    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0
    uav:
      imu_rate_odom: false
      gravity_align_en: true
    publish:
      dense_map_en: true
      map_en: false
      map_publish_interval: 10
      map_voxel_size: 0.5
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0
    evo:
      seq_name: go2w
      pose_output_en: false
    pcd_save:
      pcd_save_en: false
      type: 0
      colmap_output_en: false
      filter_size_pcd: 0.15
      interval: -1
    image_save:
      img_save_en: false
      interval: 1
    debug:
      plot_time: -10.0
      frame_cnt: 6
    offline:
      rate: 0.1
      duration: 0.0
      keyframe_interval: 0.5
```

### current_A

出典: [params.yaml](campaign_02/oct20_current_A/full/params.yaml)

```yaml
laserMapping:
  ros__parameters:
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      img_topic: /camera/camera/color/image_raw
      img_en: 0
      lidar_en: 1
      ros_driver_bug_fix: false
    camera:
      cam_model: RadTan
      cam_width: 640
      cam_height: 480
      scale: 1.0
      cam_fx: 607.06781
      cam_fy: 607.022583
      cam_cx: 313.942291
      cam_cy: 253.383209
      cam_d0: 0.0
      cam_d1: 0.0
      cam_d2: 0.0
      cam_d3: 0.0
      cam_d4: 0.0
    extrin_calib:
      extrinsic_T:
      - -0.011
      - -0.02329
      - 0.04412
      extrinsic_R:
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      Pcl:
      - 0.0175
      - -0.1138
      - -0.112
      Rcl:
      - 0.0
      - -1.0
      - 0.0
      - 0.313627591
      - 0.0
      - -0.949546068
      - 0.949546068
      - 0.0
      - 0.313627591
    time_offset:
      imu_time_offset: 0.0
      img_time_offset: 0.0
      lidar_time_offset: 0.0
      exposure_time_init: 0.0
    preprocess:
      hilti_en: false
      point_filter_num: 1
      filter_size_surf: 0.1
      lidar_type: 8
      scan_line: 4
      blind: 0.5
      feature_extract_enabled: false
    vio:
      max_iterations: 5
      outlier_threshold: 500.0
      img_point_cov: 1000.0
      patch_size: 8
      patch_pyrimid_level: 4
      grid_size: 5
      grid_n_height: 17
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: true
      inv_expo_cov: 0.1
      image_interval_sec: 0.1
    imu:
      imu_en: true
      imu_int_frame: 10
      imu_init_duration_sec: 3.0
      acc_cov: 0.1
      gyr_cov: 0.0001
      gravity_est_en: false
      ba_bg_est_en: true
      stationary_gyr_mean_threshold: 0.1
      stationary_gyr_std_threshold: 0.02
      stationary_acc_std_ratio_threshold: 0.05
      initial_rotation_cov: 0.0001
      initial_position_cov: 0.0001
      initial_velocity_cov: 0.001
      initial_gyro_bias_cov: 1.0e-06
      initial_accel_bias_cov: 0.001
      gyro_bias_cov: 1.0e-06
      accel_bias_cov: 1.0e-06
    lio:
      temporal:
        enabled: true
        initialization_mode: legacy_voxel
        window_sec: 2.0
        keyframe_sec: 0.5
        max_nodes: 6
        iterations: 3
        max_source_points: 1200
        voxel_size: 0.5
        sample_size: 0.15
        min_plane_points: 6
        plane_ratio: 0.05
        min_plane_spread: 0.01
        max_residual: 0.2
        point_sigma: 0.03
        huber_delta: 2.5
        max_imu_gap: 0.02
        consistent_deskew: true
        structure:
          enabled: true
          sigma_deg: 0.01
          max_angle_deg: 8.0
          min_planes: 8
        structural_landmarks: true
        max_landmarks: 256
        persistent_map: false
        plane_uncertainty: true
        accel_saturation: 38.259
        accel_saturation_variance: 100.0
        image_interval_sec: 0.1
        visual_factor_enabled: true
        visual_reprojection: false
        visual_joint_reference: false
        visual_full_points: false
        visual_fusion: false
      normal_consistency:
        enabled: false
        neighbors: 20
        max_neighbor_distance: 1.0
        max_planarity_ratio: 0.1
        min_spread_ratio: 0.1
        max_angle_deg: 30.0
      max_iterations: 3
      dept_err: 0.02
      beam_err: 0.05
      min_eigen_value: 0.0001
      voxel_size: 0.5
      neighbor_voxel_radius: 1
      correspondence_mode: voxel_plane
      diagnostics:
        enabled: true
        output_dir: /result/frontend/diagnostics
        detail:
          enabled: true
          start_sec: 0.0
          end_sec: 339.911921056
          final_only: true
        geometry:
          enabled: false
      xicp:
        enabled: false
        diagnostics: false
        contribution_min: 0.17364817766693041
        contribution_strong: 0.7071067811865476
        enough_information: 250.0
        insufficient_information: 180.0
        minimal_information: 35.0
        rank_relative_tolerance: 1.0e-06
      voxel_init_min_points: 5000
      voxel_init_min_planes: 100
      voxel_init_min_normal_eigenvalue: 0.02
      voxel_init_min_duration_sec: 0.1
      voxel_init_max_duration_sec: 2.0
      voxel_init_max_points: 100000
      fastlio_filter_size_map: 0.5
      fastlio_cube_side_length: 1000.0
      fastlio_detection_range: 100.0
      gravity_norm_constraint: true
      max_state_cov_for_matching: 0.01
      max_layer: 2
      max_points_num: 100
      layer_init_num:
      - 5
      - 5
      - 5
      - 5
      - 5
    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0
    uav:
      imu_rate_odom: false
      gravity_align_en: true
    publish:
      dense_map_en: true
      map_en: false
      map_publish_interval: 10
      map_voxel_size: 0.5
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0
    evo:
      seq_name: go2w
      pose_output_en: false
    pcd_save:
      pcd_save_en: false
      type: 0
      colmap_output_en: false
      filter_size_pcd: 0.15
      interval: -1
    image_save:
      img_save_en: false
      interval: 1
    debug:
      plot_time: -10.0
      frame_cnt: 6
    offline:
      rate: 0.1
      duration: 0.0
      keyframe_interval: 0.5
```

### current_B

出典: [params.yaml](campaign_02/oct20_current_B/full/params.yaml)

```yaml
laserMapping:
  ros__parameters:
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      img_topic: /camera/camera/color/image_raw
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: false
    camera:
      cam_model: RadTan
      cam_width: 640
      cam_height: 480
      scale: 1.0
      cam_fx: 607.06781
      cam_fy: 607.022583
      cam_cx: 313.942291
      cam_cy: 253.383209
      cam_d0: 0.0
      cam_d1: 0.0
      cam_d2: 0.0
      cam_d3: 0.0
      cam_d4: 0.0
    extrin_calib:
      extrinsic_T:
      - -0.011
      - -0.02329
      - 0.04412
      extrinsic_R:
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      Pcl:
      - 0.0175
      - -0.1138
      - -0.112
      Rcl:
      - 0.0
      - -1.0
      - 0.0
      - 0.313627591
      - 0.0
      - -0.949546068
      - 0.949546068
      - 0.0
      - 0.313627591
    time_offset:
      imu_time_offset: 0.0
      img_time_offset: 0.0
      lidar_time_offset: 0.0
      exposure_time_init: 0.0
    preprocess:
      hilti_en: false
      point_filter_num: 1
      filter_size_surf: 0.1
      lidar_type: 8
      scan_line: 4
      blind: 0.5
      feature_extract_enabled: false
    vio:
      max_iterations: 5
      outlier_threshold: 500.0
      img_point_cov: 1000.0
      patch_size: 8
      patch_pyrimid_level: 4
      grid_size: 5
      grid_n_height: 17
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: true
      inv_expo_cov: 0.1
      image_interval_sec: 0.1
    imu:
      imu_en: true
      imu_int_frame: 10
      imu_init_duration_sec: 3.0
      acc_cov: 0.1
      gyr_cov: 0.0001
      gravity_est_en: false
      ba_bg_est_en: true
      stationary_gyr_mean_threshold: 0.1
      stationary_gyr_std_threshold: 0.02
      stationary_acc_std_ratio_threshold: 0.05
      initial_rotation_cov: 0.0001
      initial_position_cov: 0.0001
      initial_velocity_cov: 0.001
      initial_gyro_bias_cov: 1.0e-06
      initial_accel_bias_cov: 0.001
      gyro_bias_cov: 1.0e-06
      accel_bias_cov: 1.0e-06
    lio:
      temporal:
        enabled: true
        initialization_mode: legacy_voxel
        window_sec: 2.0
        keyframe_sec: 0.5
        max_nodes: 6
        iterations: 3
        max_source_points: 1200
        voxel_size: 0.5
        sample_size: 0.15
        min_plane_points: 6
        plane_ratio: 0.05
        min_plane_spread: 0.01
        max_residual: 0.2
        point_sigma: 0.03
        huber_delta: 2.5
        max_imu_gap: 0.02
        consistent_deskew: true
        structure:
          enabled: true
          sigma_deg: 0.01
          max_angle_deg: 8.0
          min_planes: 8
        structural_landmarks: true
        max_landmarks: 256
        persistent_map: false
        plane_uncertainty: true
        accel_saturation: 38.259
        accel_saturation_variance: 100.0
        image_interval_sec: 0.1
        visual_factor_enabled: false
        visual_reprojection: false
        visual_joint_reference: false
        visual_full_points: false
        visual_fusion: true
      normal_consistency:
        enabled: false
        neighbors: 20
        max_neighbor_distance: 1.0
        max_planarity_ratio: 0.1
        min_spread_ratio: 0.1
        max_angle_deg: 30.0
      max_iterations: 3
      dept_err: 0.02
      beam_err: 0.05
      min_eigen_value: 0.0001
      voxel_size: 0.5
      neighbor_voxel_radius: 1
      correspondence_mode: voxel_plane
      diagnostics:
        enabled: true
        output_dir: /result/frontend/diagnostics
        detail:
          enabled: true
          start_sec: 0.0
          end_sec: 339.911921056
          final_only: true
        geometry:
          enabled: false
      xicp:
        enabled: false
        diagnostics: false
        contribution_min: 0.17364817766693041
        contribution_strong: 0.7071067811865476
        enough_information: 250.0
        insufficient_information: 180.0
        minimal_information: 35.0
        rank_relative_tolerance: 1.0e-06
      voxel_init_min_points: 5000
      voxel_init_min_planes: 100
      voxel_init_min_normal_eigenvalue: 0.02
      voxel_init_min_duration_sec: 0.1
      voxel_init_max_duration_sec: 2.0
      voxel_init_max_points: 100000
      fastlio_filter_size_map: 0.5
      fastlio_cube_side_length: 1000.0
      fastlio_detection_range: 100.0
      gravity_norm_constraint: true
      max_state_cov_for_matching: 0.01
      max_layer: 2
      max_points_num: 100
      layer_init_num:
      - 5
      - 5
      - 5
      - 5
      - 5
    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0
    uav:
      imu_rate_odom: false
      gravity_align_en: true
    publish:
      dense_map_en: true
      map_en: false
      map_publish_interval: 10
      map_voxel_size: 0.5
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0
    evo:
      seq_name: go2w
      pose_output_en: false
    pcd_save:
      pcd_save_en: false
      type: 0
      colmap_output_en: false
      filter_size_pcd: 0.15
      interval: -1
    image_save:
      img_save_en: false
      interval: 1
    debug:
      plot_time: -10.0
      frame_cnt: 6
    offline:
      rate: 0.1
      duration: 0.0
      keyframe_interval: 0.5
```

### current_C

出典: [params.yaml](campaign_02/oct20_current_C/full/params.yaml)

```yaml
laserMapping:
  ros__parameters:
    common:
      lid_topic: /livox/lidar
      imu_topic: /livox/imu
      img_topic: /camera/camera/color/image_raw
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: false
    camera:
      cam_model: RadTan
      cam_width: 640
      cam_height: 480
      scale: 1.0
      cam_fx: 607.06781
      cam_fy: 607.022583
      cam_cx: 313.942291
      cam_cy: 253.383209
      cam_d0: 0.0
      cam_d1: 0.0
      cam_d2: 0.0
      cam_d3: 0.0
      cam_d4: 0.0
    extrin_calib:
      extrinsic_T:
      - -0.011
      - -0.02329
      - 0.04412
      extrinsic_R:
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      - 0.0
      - 0.0
      - 0.0
      - 1.0
      Pcl:
      - 0.0175
      - -0.1138
      - -0.112
      Rcl:
      - 0.0
      - -1.0
      - 0.0
      - 0.313627591
      - 0.0
      - -0.949546068
      - 0.949546068
      - 0.0
      - 0.313627591
    time_offset:
      imu_time_offset: 0.0
      img_time_offset: 0.0
      lidar_time_offset: 0.0
      exposure_time_init: 0.0
    preprocess:
      hilti_en: false
      point_filter_num: 1
      filter_size_surf: 0.1
      lidar_type: 8
      scan_line: 4
      blind: 0.5
      feature_extract_enabled: false
    vio:
      max_iterations: 5
      outlier_threshold: 500.0
      img_point_cov: 1000.0
      patch_size: 8
      patch_pyrimid_level: 4
      grid_size: 5
      grid_n_height: 17
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: true
      inv_expo_cov: 0.1
      image_interval_sec: 0.1
    imu:
      imu_en: true
      imu_int_frame: 10
      imu_init_duration_sec: 3.0
      acc_cov: 0.1
      gyr_cov: 0.0001
      gravity_est_en: false
      ba_bg_est_en: true
      stationary_gyr_mean_threshold: 0.1
      stationary_gyr_std_threshold: 0.02
      stationary_acc_std_ratio_threshold: 0.05
      initial_rotation_cov: 0.0001
      initial_position_cov: 0.0001
      initial_velocity_cov: 0.001
      initial_gyro_bias_cov: 1.0e-06
      initial_accel_bias_cov: 0.001
      gyro_bias_cov: 1.0e-06
      accel_bias_cov: 1.0e-06
    lio:
      temporal:
        enabled: true
        initialization_mode: legacy_voxel
        window_sec: 2.0
        keyframe_sec: 0.5
        max_nodes: 6
        iterations: 3
        max_source_points: 1200
        voxel_size: 0.5
        sample_size: 0.15
        min_plane_points: 6
        plane_ratio: 0.05
        min_plane_spread: 0.01
        max_residual: 0.2
        point_sigma: 0.03
        huber_delta: 2.5
        max_imu_gap: 0.02
        consistent_deskew: true
        structure:
          enabled: true
          sigma_deg: 0.01
          max_angle_deg: 8.0
          min_planes: 8
        structural_landmarks: true
        max_landmarks: 256
        persistent_map: false
        plane_uncertainty: true
        accel_saturation: 38.259
        accel_saturation_variance: 100.0
        image_interval_sec: 0.1
        visual_factor_enabled: true
        visual_reprojection: false
        visual_joint_reference: false
        visual_full_points: false
        visual_fusion: true
      normal_consistency:
        enabled: false
        neighbors: 20
        max_neighbor_distance: 1.0
        max_planarity_ratio: 0.1
        min_spread_ratio: 0.1
        max_angle_deg: 30.0
      max_iterations: 3
      dept_err: 0.02
      beam_err: 0.05
      min_eigen_value: 0.0001
      voxel_size: 0.5
      neighbor_voxel_radius: 1
      correspondence_mode: voxel_plane
      diagnostics:
        enabled: true
        output_dir: /result/frontend/diagnostics
        detail:
          enabled: true
          start_sec: 0.0
          end_sec: 339.911921056
          final_only: true
        geometry:
          enabled: false
      xicp:
        enabled: false
        diagnostics: false
        contribution_min: 0.17364817766693041
        contribution_strong: 0.7071067811865476
        enough_information: 250.0
        insufficient_information: 180.0
        minimal_information: 35.0
        rank_relative_tolerance: 1.0e-06
      voxel_init_min_points: 5000
      voxel_init_min_planes: 100
      voxel_init_min_normal_eigenvalue: 0.02
      voxel_init_min_duration_sec: 0.1
      voxel_init_max_duration_sec: 2.0
      voxel_init_max_points: 100000
      fastlio_filter_size_map: 0.5
      fastlio_cube_side_length: 1000.0
      fastlio_detection_range: 100.0
      gravity_norm_constraint: true
      max_state_cov_for_matching: 0.01
      max_layer: 2
      max_points_num: 100
      layer_init_num:
      - 5
      - 5
      - 5
      - 5
      - 5
    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0
    uav:
      imu_rate_odom: false
      gravity_align_en: true
    publish:
      dense_map_en: true
      map_en: false
      map_publish_interval: 10
      map_voxel_size: 0.5
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0
    evo:
      seq_name: go2w
      pose_output_en: false
    pcd_save:
      pcd_save_en: false
      type: 0
      colmap_output_en: false
      filter_size_pcd: 0.15
      interval: -1
    image_save:
      img_save_en: false
      interval: 1
    debug:
      plot_time: -10.0
      frame_cnt: 6
    offline:
      rate: 0.1
      duration: 0.0
      keyframe_interval: 0.5
```
