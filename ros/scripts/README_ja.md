# ROS1 1フレーム取得スクリプト

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
