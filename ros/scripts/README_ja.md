# ROS1 1フレーム取得スクリプト

## channel 200のazimuthグラフを10秒ごとに作る

`plot_channel_azimuth_ros1.py` は `/cepton3/points` をsubscribeし、channel
ID 200の点を1秒間収集して、azimuthとtimestampのグラフをPNGで保存します。
収集窓は10秒ごとに開始します。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun cepton_ros plot_channel_azimuth_ros1.py
```

PNGはデフォルトで`~/デスクトップ/cepton_azimuth_plots`に保存されます。azimuthを
度単位にする場合や保存先を変える場合は次のように実行します。

```bash
rosrun cepton_ros plot_channel_azimuth_ros1.py \
  --degrees \
  --output-dir /tmp/cepton_azimuth_plots
```

トピック、channel ID、収集時間、周期も変更できます。

```bash
rosrun cepton_ros plot_channel_azimuth_ros1.py \
  --topic /cepton3/points \
  --channel-id 200 \
  --window 1 \
  --interval 10
```

このスクリプトには、`WITH_TS_CH_F=ON`と`WITH_POLAR=ON`で生成された
PointCloud2、およびPython 3の`matplotlib`が必要です。

## 概要

`capture_1frame_ros1.py` は、ROS1 の `PointCloud2` トピックから1フレームだけ受信し，CSV ファイルとして保存するためのスクリプトです．

イカをやること．

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 使い方

デフォルトでは `/cepton3/points` から1フレームを取得し，このスクリプトと同じディレクトリに `cepton_frame.csv` を保存します．

```bash
python3 capture_1frame_ros1.py
```

## オプション

### トピック名を指定する

```bash
python3 capture_1frame_ros1.py --topic /cepton3/points
```

### 出力先 CSV の名前を指定する

```bash
python3 capture_1frame_ros1.py --output ./output/cepton_frame.csv
```

### タイムアウトを指定する

単位は秒です．

```bash
python3 capture_1frame_ros1.py --timeout 30
```

### ROS ヘッダー情報も CSV に含める

列の意味が書いてあります．
CSV の列名は，受信した `PointCloud2` メッセージのフィールド名に基づいて自動で作成されます．

```bash
python3 capture_1frame_ros1.py --include-header
```

## 実行例

```bash
python3 capture_1frame_ros1.py \
  --topic /cepton3/points \
  --output ./cepton_frame.csv \
  --timeout 10 \
  --include-header
```

## 10フレームをCSV保存してグレースケール画像に変換する

`capture_10frames_grayimage_ros1.py` は，`capture_1frame_ros1.py` を10回実行してCSVを保存し，
各CSVを `ros/ros2/scripts/pointcloudgrayimage.py` でPNG画像に変換します．

```bash
python3 capture_10frames_grayimage_ros1.py
```

デフォルトでは以下に出力します．

- CSV: `capture_grayimage_output/csv/cepton_frame_01.csv` から `cepton_frame_10.csv`
- PNG: `capture_grayimage_output/grayimage/cepton_frame_01_gray.png` から `cepton_frame_10_gray.png`

実行回数，出力先，トピック名，ガンマ値を指定する例です．

```bash
python3 capture_10frames_grayimage_ros1.py \
  --topic /cepton3/points \
  --count 10 \
  --output-dir ./output_10frames \
  --gamma 1.0 \
  --timeout 10
```




## 注意事項

- `NaN` を含む点も CSV に出力されます。
