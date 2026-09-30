# X-ICP による LiDAR 退化対処

本ブランチは [X-ICP: Localizability-Aware LiDAR Registration for Robust Localization in Extreme Environments](https://arxiv.org/abs/2211.16335)
（TRO 2024、手元の v4 論文）の方向別 localizability 判定と制約付き更新を、FAST-LIVO2 の反復誤差状態 Kalman filter に組み込んだ実装です。
論文の pose-only ICP を丸ごと置換したものではありません。以下に論文との対応と適応部分を明記します。

## 適用範囲・切り替え

- 現在の `fastlio_knn`（`fastlio_ikdtree` の互換名）を維持します。
- 対応点探索、ikd-tree 地図更新、外部キャリブレーション、IMU 初期化・gravity/bias 設定は変更しません。
- 対応点から後段の LiDAR 更新に適用するため、`voxel_plane` でも利用できます。
- LIO-only と LIVO の両方で LiDAR 更新に適用します。VIO の画像更新自体には適用しません。
- `humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml` の `lio.xicp.enabled: true` が Go2W の既定です。
  `false` にすると従来の LiDAR 更新経路に戻ります。YAML の変更は次回ノード起動で反映され、再ビルド不要です。
  X-ICP 設定を持たない他のパラメータファイルでは既定で無効です。

通常どおりコンテナ内のリポジトリルートから実行します。raw トピックだけを再生する既存の仕組みは維持しています。

```bash
bash scripts/fast_livo2/reconstruct_raw.sh bags/raw_20260907_114335 --lio-only --rate 0.5
```

LIVO は `--lio-only` を外します。比較時は同じ bag・再生速度を使い、先に `enabled: false`、次に `true` で実行し、それぞれのログを保存してください。
IMU 欠落がある比較は退化対処の評価に使えません。

## 論文に対応する部分

各反復で、採用された点–平面対応を現在の **LiDAR 原点のローカル座標**に変換します。
世界原点からの距離や IMU の事前共分散が退化判定を左右しないよう、判定には重みなしの幾何だけを使います。

1. `g_r = p × n`、`g_t = n` から回転・並進それぞれの 3×3 ヘッセ行列を計算し、固有方向を求めます。
2. 回転の寄与計算では `g_r / max(1, |g_r|)` を使います。小さい回転モーメントは拡大しません。
3. 各固有方向に対し、寄与の絶対値が低閾値以上の和 `Lc`、強寄与閾値以上の和 `Ls` を求めます。
4. `Lc >= k1` または `Ls >= k2` なら FULL、そうでなく `Lc >= k2` または `Ls >= k3` なら PARTIAL、それ以外は NONE です。
5. PARTIAL は `Lc >= k2` なら低閾値以上の対応、そうでなければ強寄与の対応を再利用します。
   再選択した対応の **3次元**の補助最小二乗問題を解き、該当方向への射影だけを拘束値として採用します。
6. FULL には拘束を加えず、NONE は LiDAR 補正をゼロ、PARTIAL は補助解の射影に拘束した連立問題を解きます。

単に小さい固有値を切り捨てる方式や、最初の反復だけを判定する簡略版 Xs ではありません。
回転・並進の分離は **判定と補助問題のみ**です。最終更新では元の回転・並進・他状態の結合を維持します。

既定閾値は論文の Velodyne 用の出発値です。MID360 で妥当性を検証済みの調整値ではありません。

| パラメータ（`lio.xicp.*`） | 既定値 | 意味 |
|---|---:|---|
| `contribution_min` | cos(80°) ≈ 0.173648 | 弱すぎる寄与を除外 |
| `contribution_strong` | cos(45°) ≈ 0.707107 | 強寄与の下限 |
| `enough_information` | 250 | k1 |
| `insufficient_information` | 180 | k2 |
| `minimal_information` | 35 | k3 |
| `rank_relative_tolerance` | 1e-6 | 補助ヘッセ行列の SVD 相対打ち切り閾値 |

寄与は平均ではなく **和**です。1更新の対応点が35点未満なら既定値では全方向 NONE となります。
LIVO の画像周期で分割された疎な LiDAR フレームでは特に確認が必要です。点数を増やすための蓄積・同期方式は今回変更しません。

## IEKF への適応と論文との差分

### 制約の基準と座標変換

19状態の順序は `[回転_I, 位置_W, 露光, 速度_W, gyro_bias, accel_bias, gravity_W]` です。
IMU 回転は右摂動です。`p_I = E p_L + t`、IMU 姿勢を `R` とすると、状態増分から LiDAR ローカルの pose 増分への写像 `B` は次になります。

```text
theta_L = E^T theta_I
trans_L = -E^T [t]x theta_I + (R E)^T delta_position_W
```

この lever arm 項を含め、ローカル固有方向の拘束を19状態へ写します。並進方向をそのまま世界座標の xyz に当てはめません。

論文の ICP 増分の代わりに、**同じスキャンの IMU 予測からの LiDAR 補正** `e` を拘束します。
反復ごとにゼロの基準をリセットして補正が積み上がらないようにするためです。
現在の接空間へ運んだ事前差分を `v`、現在の観測残差を `r`、19列の Jacobian を `J` とすると、

```text
y = r - J v
delta = v + e
C e = D y
```

となります。NONE の `D` 行はゼロ、PARTIAL は再選択した対応の3D補助解の射影を `y` に作用させる行です。
この prior-relative 制約と Kalman 事前項は、pose-only ICP にはない統合部分です。

### 解法と共分散

`P` を現在の接空間の事前共分散、`W` を観測逆分散とすると、

```text
A = P^-1 + J^T W J
K0 = A^-1 J^T W
Z = A^-1 C^T (C A^-1 C^T)^-1
L = K0 - Z (C K0 - D)
delta = v + L y
```

を解きます。これは二次目的関数の **等式制約 KKT 系を Schur 補行列で解いたもの**であり、出力 pose のみを後処理で切り取る処理ではありません。
全観測数の平方サイズの行列は作りません。

共分散は実際のゲイン `L` で Joseph 形式に更新します。

```text
P_new = (I - L J) P (I - L J)^T + L W^-1 L^T
```

PARTIAL の目標値は元の残差と同じノイズから作られるため、独立の観測として二重に数えません。
NONE は「正しい位置が分かった」という観測ではなく補正の抑制です。
したがってその方向の共分散を人工的にゼロにしません。速度・bias もこのゲインの相互共分散を通じて更新します。
全6方向 NONE なら LiDAR による速度・bias 補正も消え、事前共分散を維持します。
既存の SO(3)/gravity 接空間の移送はその後に行います。

数値計算の差分・保守的な追加処理:

- 論文の補助問題の pivoted LU + RIF の代わりに、3×3 rank-revealing SVD を使います。
- 補助問題の観測可能な空間に拘束方向が含まれない場合、PARTIAL を NONE に格下げし、ログに記録します。
- 回転モーメントがゼロでも並進の法線情報は保持します。論文の「near-zero torque の情報ペアを除外」を両成分に適用する実装とはこの点が異なります。
- 共分散・入力・解が不正なら、そのスキャンの LiDAR 補正を取り消して IMU 予測へ戻し、エラーを出します。
  無制約更新への暗黙フォールバックはしません。

## 診断ログ

`lio.xicp.diagnostics: true` で `humble_ws/src/fast-livo2/Log/xicp_diagnostic.csv` を出力します。
既存の `mat_pre.txt` 等と同様、各実行で上書きするため比較前に別名で保存してください。

- `time`: 最初の LiDAR 時刻基準の秒。`iteration=-1` は最初の反復です。
- `matches`, `constraints`: 採用対応点数と等式拘束数。
- `r0..r2`, `t0..t2`: 固有値昇順の回転・並進方向。固定 roll/pitch/yaw/x/y/z ではありません。
- `category`: `0=NONE`, `1=PARTIAL`, `2=FULL`。`combined`, `strong` は `Lc`, `Ls`。
- `axis_l*`, `axis_w*`: LiDAR座標・世界座標の方向。符号は任意です。
- `selected`, `rank_fallback`: PARTIAL補助問題の対応数と格下げ有無。
- `prior_*`, `before_*`, `after_*`: IMU予測、反復直前、反復直後の速度・pitch・bias。
  `forward_speed` は IMU/body の x 方向速度、pitch は rad です。
- `gain_constraint_error`: `|C L - D|`、`solve_ms`: 判定・制約解法・状態更新の時間。ログI/O時間は含みません。

ログ行は各反復時点の診断であり、後の反復で対応点が無くなった／数値解を棄却した場合、そのスキャンの最終採用結果ではありません。
最終軌跡は `mat_out.txt` と合わせて確認します。更新棄却はノードログの `X-ICP rejected LIO update` に出ます。

## 検証と限界

導入時のビルド・単体テスト・160秒比較再生の結果は [XICP_VALIDATION.md](XICP_VALIDATION.md) を参照してください。

`xicp_geometry_and_constrained_filter` テストは閾値境界、回転正規化、法線符号、3D補助解、rank落ち、座標不変性、lever arm、KKT参照解、FULLの等価性、NONEの事前維持、共有観測ノイズの共分散、非有限値の棄却を確認します。
既存の IMU・voxel初期化テストも併用します。

重要: **LiDAR拘束が完全に無い方向の正しい速度を、この処理だけで復元することはできません。**
IMU予測ですでに後退していれば、NONEでそれを保持することもあります。PARTIALでは残った対応から補正できますが、情報のない方向には別センサー、構造の見える視野、biasの改善等が必要です。
X-ICPは時間同期・IMU欠落・誤った外部較正・誤対応の全般を修復するものではありません。
