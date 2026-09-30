# voxel-plane実データの分析

レポート: [VOXEL_PLANE_FAILURE_ANALYSIS.md](../../VOXEL_PLANE_FAILURE_ANALYSIS.md)。
元capture: `diagnostic_captures/lio-reconstruction-20260928-093848-xfyj5zxg/`。
派生キャッシュ: `diagnostic_captures/voxel-plane-analysis-20260928/`。
入力は3.499–335.999秒の完了3,326スキャン。作業中の2回の確認で元CSVサイズに変化はなかった。
`summary.json`・node終了情報はまだないため、captureの正常終了は未確認。

`data/`には通常Log・集計CSV・実効設定のコピー、`snapshot.json`にはそのSHA256を保存した。
元captureの巨大geometry/correspondencesは変更せず、`extract.py`で162スキャンを別の`sample/`へ抽出した。
サンプルは全体5秒間隔、170–220秒と295–330秒は1秒間隔、320–323秒は0.1秒間隔に最も近いスキャンの和集合。
選択時刻とIDは`sample-selection.json`。全点での時間平均と、時間サンプルでの中央値を混同しない。

リポジトリルートから、NumPy・SciPyを使って再計算できる。
元captureはコンテナ所有の700ディレクトリなので、抽出だけは元repoを読み取り専用でマウントしたDocker内で行った。
再抽出時に`capture`引数へ元capture、`work`へ別の解析出力先を渡す。`work/data`の小容量ログを先に保全する。

```bash
WORK=diagnostic_captures/voxel-plane-analysis-20260928
SCRIPTS=docs/diagnostics/voxel_plane_failure_20260928
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
python3 "$SCRIPTS/dynamics.py" "$WORK"
python3 "$SCRIPTS/geometry.py" "$WORK"
python3 "$SCRIPTS/covariance_frame_audit.py"
```

- `dynamics.py`: 全スキャンの姿勢・速度・bias、予測と更新の収支、同状態の無制約ゲイン、
  条件付き共分散によるbias寄与分解、前回IMUとの完全一致、全反復の行列検査。
  集計だけの検査用metadataは別ディレクトリに作り、元metadataは変更しない。
- `geometry.py`: 選択162スキャンの分散・採否・ゲイン・ヘッセ照合、面の重み・更新停止・情報集中、
  スキャン床と地図面の傾斜、query共分散の座標系を修正した局所感度。
  後者は地図を再生成する再生ではない。ゲート反転率を閉ループの効果と解釈しない。
- `covariance_frame_audit.py`: 独立な有限差分とworld座標回転による共分散変換の代数検査。

幾何の元検査は面内距離だけが規格化誤差2.00e−6で閾値1e−6を超えた。
保存結果`sample-geometry-validation.json`はFAILを維持している。
`geometry.py`で明示したfloat演算誤差の区間では全サンプルが整合し、他項目も閾値内。
補足判定は`sample-geometry-qualification.json`。補正・共分散の全行検査とサンプル対応点検査はPASS。

図は保存済みの`dynamics/dynamics.npz`と前回の
`diagnostic_captures/normal-gate-analysis-20260928/dynamics/dynamics.npz`から作成した。
予測加速度の全体比較だけは1秒移動平均。ターン拡大図は各スキャン値。
原理上の「FULL」「共分散PSD」「小残差」と、真の姿勢・速度・biasの正しさを区別する。
