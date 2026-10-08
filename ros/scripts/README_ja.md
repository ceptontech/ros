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

## タイムスタンプモードの実機テスト

`test_timestamp_mode_ros1.sh` は、PTP を無効（`WITH_PTP=OFF`）にした指定タイムスタンプモードでビルドし、実機から 1 フレームを CSV に保存してから検証します。`--workspace` には、当リポジトリを `src` 配下に配置またはシンボリックリンクした catkin workspace を指定します。

```bash
bash scripts/test_timestamp_mode_ros1.sh \
  --workspace ~/catkin_ws \
  --mode frame_offset \
  --config /path/to/sensor_params.yaml \
  --min-max-offset-us 75000
```

`frame_offset` の `--min-max-offset-us 75000` は、10 Hz 製品の 100 ms フレームで 16-bit の 65,535 µs を超える offset を正しく扱えることを確認する例です。

結果は workspace の `timestamp_test_results/` に保存されます。

- `pointcloud_<mode>.csv`: `--include-header` 付きの PointCloud2 CSV
- `verification_<mode>.json`: 検証結果と timestamp 統計
- `roscore.log`、`manager.log`、`publisher.log`: 実機接続時のログ

### PTP 有効時のタイムスタンプモード別 1 フレーム取得

#### `ptp4l` の起動

テストを起動する前に、センサーと接続している NIC で `ptp4l` を起動したままにします。`enp3s0` は実際の NIC 名に置き換えてください。

```bash
sudo ptp4l -i enp3s0 -m
```

環境固有の設定ファイルを使う場合は `-f` を追加します。

```bash
sudo ptp4l -i enp3s0 -f /path/to/ptp4l.conf -m
```

`-m` は同期状態を標準出力へ表示します。PTP Grandmaster が到達可能で、同期が完了してからテストを開始してください。スクリプトはセンサーの `time_sync_offset` が非ゼロになるまで待機せず、値が `0` の時点で失敗します。

次の3本は、PTP を有効（`WITH_PTP=ON`）にして各タイムスタンプモードでビルドしてから1フレームを CSV に保存します。CSV内容の検証は行いません。すべて `--workspace` が必須で、必要に応じて `--config`、`--topic`、`--timeout`、`--output-dir` を渡せます。

PointCloud の取得前に `/cepton3/sensor_information`（INFZ 由来）から `time_sync_offset` を取得します。値が `0` の場合は CSV を取得せず、`ptp4l を起動してください` と表示して失敗します。必要なら `--info-topic` で確認対象の topic を変更できます。

| script | `TIMESTAMP_MODE` | `WITH_PTP` |
| --- | --- | --- |
| `capture_relative_ptp_on_ros1.sh` | `RELATIVE` | `ON` |
| `capture_frame_offset_ptp_on_ros1.sh` | `FRAME_OFFSET` | `ON` |
| `capture_absolute_ptp_on_ros1.sh` | `ABSOLUTE` | `ON` |

例:

```bash
bash scripts/capture_absolute_ptp_on_ros1.sh \
  --workspace ~/catkin_ws \
  --config /path/to/sensor_params.yaml
```

出力先の CSV 名には組合せが含まれます。たとえば `ABSOLUTE` と PTP 有効では `pointcloud_absolute_ptp_on.csv` です。

CSV 取得済みの場合は、ROS や実機を使わずに検証だけを再実行できます。

```bash
python3 scripts/verify_timestamp_csv.py \
  --mode frame_offset \
  --input timestamp_test_results/frame_offset_*/pointcloud_frame_offset.csv \
  --min-max-offset-us 75000
```

利用できるモードは `relative`、`frame_offset`、`absolute` です。`absolute` は Unix time ではなく、現時点ではセンサー時刻基準で検証します。
