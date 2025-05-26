# ハードウェア抽象化レイヤー

## 3Dカメラ
### Topics

|名前|タイプ|Pub/Sub|機能|
|:---|:---|:---:|:---|
|~pc2|PointCloud2|pub|キャプチャした点群|
|~image|Image|pub|キャプチャした2D画像|
|~capture|Bool|sub|キャプチャトリガー(Trigger modeのみ)|
|~captured|Bool|pub|キャプチャ完了通知(Trigger modeのみ)|

### Params
|名前|タイプ|機能|
|:---|:---|:---|
|~mode|Int32|0:Streaming mode(default), 1:Trigger mode|
|~fps|Int32|Frames per second|
|~projector_intensity|Int32|プロジェクタ発光強度(%)|
|~camera_exposure|Int32|露光時間(msec)|
