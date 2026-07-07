# 1フレーム取得スクリプト


## 概要

`capture_1frame_ros2.py` は，ROS2 の `PointCloud2` トピックから1メッセージ（1フレーム）を取得し，csvで保存します．


## 使い方

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

```bash
python3 ros/ros2/scripts/capture_1frame_ros2.py
```

## オプション

### トピック名を指定する

```bash
python3 ros/ros2/scripts/capture_1frame_ros2.py --topic /cepton_pcl2
```

### 出力先 CSV の名前を指定する

```bash
python3 ros/ros2/scripts/capture_1frame_ros2.py --output ./output/cepton_frame.csv
```

### タイムアウトを指定する

単位は秒です．

```bash
python3 ros/ros2/scripts/capture_1frame_ros2.py --timeout 30
```

### ROS ヘッダー情報も CSV に含める

CSV の先頭列にヘッダーを追加します．
点群データの列名は，受信した `PointCloud2` メッセージのフィールド名に基づいて自動で作成されます．

```bash
python3 ros/ros2/scripts/capture_1frame_ros2.py --include-header
```

## 実行例

```bash
python3 ros/ros2/scripts/capture_1frame_ros2.py \
  --topic /cepton_pcl2 \
  --output ./cepton_frame.csv \
  --timeout 10 \
  --include-header
```

## そのた

- `NaN` を含む点も CSV に出力されます．
- CSV に保存される点数は，publisher 側の距離・角度・フラグフィルタを通った後の点数です．
- `WITH_POLAR` や `WITH_TS_CH_F` の設定で `PointCloud2` のフィールドが変わった場合も，CSV の列は自動で追従します．

(ﾐ・ﻌ・ﾐ)