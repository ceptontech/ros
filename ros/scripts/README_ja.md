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




## 注意事項

- `NaN` を含む点も CSV に出力されます。
