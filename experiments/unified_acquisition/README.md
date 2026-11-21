# 統一取得の成果物

使用する固定パッケージは **frozen_20261111_swap_telemetry**。
ID: `6231ef7a8b62176d887664e22caa669a3240f1467f638a4607c2ce41f7acc151`

条件定義: `../../config/experiments/lio_livo_comparison.yaml`
詳細手順: `../../docs/UNIFIED_LIO_LIVO_ACQUISITION.md`

```bash
# リポジトリルートから。45秒prefixだけを12条件で実行する。
python3 experiments/unified_acquisition/frozen_20261111_swap_telemetry/runtime/experiment_suite.py prefix \
  experiments/unified_acquisition/frozen_20261111_swap_telemetry \
  experiments/unified_acquisition/campaign_01

# 同じ今回のcampaignを再開し、合格prefixのみ本取得へ進める。
nohup python3 experiments/unified_acquisition/frozen_20261111_swap_telemetry/runtime/experiment_suite.py run \
  experiments/unified_acquisition/frozen_20261111_swap_telemetry \
  experiments/unified_acquisition/campaign_01 --resume \
  > experiments/unified_acquisition/campaign_01/driver.log 2>&1 &
```

本取得はまだ開始していない。基準側の9月7日A/B/Cは6秒動作確認の途中、約0.4–0.47秒で速度／残差停止した。設定を調整していないため、45秒prefixでも早期停止する可能性が高い。これを無理に全区間へ進めない。

現在側A/B/Cは各6秒を正常終了。Bでは視覚イベントの状態・露光変化が0、Cでは視覚更新が有効。これらは収集系の動作確認であり、45秒prefix合格や全区間安定性を意味しない。
10月20日はbagハッシュ・入力トピック・CameraInfoを確認したが再生していない。

`package_v1`は作成失敗、`package_v2`/`package_v3`は開発中ビルド。誤実行防止のためv2/v3のlockはdevelopment-lock.jsonとして保管してあり、実行対象ではない。
`smoke_*`は開発検証専用。今回の比較結果・本取得prefixへ転用しない。
最終パッケージはv3と同一の推定器ソース・バイナリに、起動・初期化監督等の修正後スクリプトを固定したもの。関連するハッシュ・検証範囲はfrozen_20261111_swap_telemetry/validation.jsonに記録。

## 2026-11-11 監督処理の更新

swap使用量は記録のみとし、単独のswap_rising停止を削除。推定器ソース・バイナリ・実効推定設定は変更なし。campaign_02の実行パッケージを新IDへ移行済み。既存result.jsonの旧IDは保持し、移行証跡はcampaign_02/package_transition_20261111_swap_telemetry.jsonへ保存。2回目の途中fullはfull_swap_stopped_02へ退避済み。新パッケージでcampaign_02を--resumeすると、合格済みprefixを再利用しoct20_current_Aのfullを先頭から再取得する。旧パッケージをcampaign_02へ指定するとID不一致で拒否される。
