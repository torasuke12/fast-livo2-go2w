# PCL点群の解放時異常とビルド修正

2026-10-01、`visual-safe-standard60` は60秒の入力処理とsummaryの保存後にexit 139となった。
同一バイナリと設定に診断用signal handlerを付けた反復でも再現した。
バックトレースは `LIVMapper::~LIVMapper()` から `pcl::PointCloud<pcl::PointXYZINormal>` の解放、`free()` を指した。
OOMによる終了ではない。summaryのcompleteだけでは正常終了と判断できない。

小さな独立プログラムで、配布PCLのVoxelGridへ点群を渡し、出力点群を呼び出し側で破棄した。
同一ソース・同一PCLでコンパイル設定だけを変えた結果は以下。

| コンパイル | Eigen既定alignment | malloc aligned | 出力の破棄 |
|---|---:|---:|---|
| `-O2 -march=native` | 32 bytes | 0 | 最初の出力でSIGSEGV |
| `-O2` | 16 bytes | 1 | サイズを変えた16反復が正常終了 |

[実行ログ](pcl-abi-reproduction.log)。native版の解放命令はpoint-bufferの8 bytes前を読み、
それをfreeの引数にしていた。配布PCLの通常mallocによるバッファと、呼び出し側のEigen独自aligned-freeが合っていない。
この試験は、少なくとも終了異常の原因を示す。地図・軌道のずれの全原因が同じだとは示していない。

CMakeから `-march=native` / `-mcpu=native` を除き、`-mtune=native` は残した。
インストール済みPCL内で確保し、FAST-LIVO2側で解放する実際の境界を通る
`pcl_binary_allocation_compatibility` 回帰試験を追加した。
独自に命令セットを変更したビルドでは、依存ライブラリとの整合性をこの試験で確認する必要がある。
実際の再現・修正後検証の対象は今回のx86-64環境。ARM実機の動作確認ではない。

Eigen公式資料も、alignmentの設定がバイナリ互換性に影響することを説明している。
またEigen 3.4では、`EIGEN_MAX_ALIGN_BYTES=16` の指定だけではAVXとの組合せでallocatorの不一致を防げない問題が報告されている。
そのため今回は命令セットの既定値を揃えた。
[Eigenの設定資料](https://libeigen.gitlab.io/eigen/docs-3.3/TopicPreprocessorDirectives.html)、
[Eigenの該当報告](https://gitlab.com/libeigen/eigen/-/issues/2556)。

修正版は別のbinary/source hashで記録する。浮動小数点の計算経路も変わるため、
旧nativeビルドの軌道精度・ROS一致・反復性の結果を修正版の根拠として流用しない。


修正後は追加試験を含む15件のC++試験に成功した（[build log](build-pcl-abi.log)）。
通常LIVOの同じ60秒・同じ600/12001/1798入力でもexit 0となり、点群解放時の異常は消失した。
ただし59.31秒のzは−1.059 mで、高さのずれは残る（[結果](pcl-abi-standard60-result.json)）。
この通常LIVO条件は精度面では採用しない。
