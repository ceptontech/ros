# 前提条件

### 対応プラットフォーム

- Ubuntu 20.04 LTS `amd64`/`arm64` + ROS 2 Galactic Geochelone
- Ubuntu 22.04 LTS `amd64`/`arm64` + ROS 2 Humble Hawksbill
- Ubuntu 24.04 LTS `amd64`/`arm64` + ROS 2 Jazzy Jalisco

これらのパッケージで採用しているコーディングスタイル、およびフォーマッター／リンターの実行方法は、[CONTRIBUTING.md](./CONTRIBUTING.md) を参照してください。

# CeptonPublisher
このリポジトリは Cepton の LiDAR を Robot Operating System (ROS)で扱うためのドライバを提供しています．

## 基本設定パラメータ

[parameters.yaml](parameters.yaml)を書き換えることによってpublishするtopicの仕方やソースなどを設定できます．以下が具体的な書き方です．

### データソースの設定

#### `capture_file`（string、既定値: ""）

- **説明**: pcap再生のためのファイルパス．
- **用途**: 実機の代わりにpcapファイルで動作の確認ができます．
- **例**: `capture_file: "/path/to/capture.pcap"`

#### `capture_loop`（bool、既定値: false）

- **説明**: キャプチャの再生を繰り返すかどうか
- **用途**: `capture_file` を指定した場合にのみ適用されます。
- **注記**: 内部で `CEPTON_REPLAY_FLAG_PLAY_LOOPED` フラグを設定します。

#### `sensor_network_sources`（vector<string>、既定値: `["0.0.0.0:8808"]`）

- **説明**: ライブセンサーデータを受信するネットワークソースの一覧
- **用途**: キャプチャファイルを再生していない場合にのみ適用されます。各ソースは `ip:port` または `ip:port:multicast_group` として指定します。ここで指定する IP はセンサーの IP ではなく、このマシンのネットワークインターフェースの IP です。
- **形式**:
  - `"ip:port"`: 特定のインターフェース、または全インターフェース（`0.0.0.0`）でのユニキャスト
  - `"ip:port:multicast_group"`: マルチキャスト設定
- **注記**: Cepton センサーの標準通信ポートは 8808 です。
- **例**:

```yaml
# 全インターフェースのポート 8808 で待ち受ける
sensor_network_sources: ["0.0.0.0:8808"]

# 全インターフェースで複数ポートを使用する
sensor_network_sources: ["0.0.0.0:8808", "0.0.0.0:8809"]

# 特定のインターフェースを使用する
sensor_network_sources: ["192.168.32.32:8808"]

# マルチキャストを使用する
sensor_network_sources: ["192.168.1.100:8808:239.255.0.1"]
```

### 点群出力の設定

#### `pcl2_output_type`（string、既定値: `"BOTH"`）

- **説明**: 点群トピックの公開方法を制御します。
- **選択肢**:
  - `"NONE"`: 点群を公開しません。
  - `"IP"`: ハンドルごとのトピックのみを公開します。
  - `"SN"`: シリアル番号ごとのトピックのみを公開します。
  - `"BOTH"`: ハンドルごととシリアル番号ごとの両方のトピックを公開します。
- **作成されるトピック**:
  - メイントピック: `cepton_pcl2`
  - ハンドルごと: `handle_<HANDLE>`
  - シリアル番号ごと: `serial_<SERIAL_NUMBER>`

## 点群フィルタリングの設定

### 点フラグによるフィルタリング

LiDAR によっては、一部の点フラグがサポートされないことがあります。

#### `iclude_saturated_points`

- **説明**: 光子センサーが検出できる最大レベルを超える明るさの点です。
- **用途**: `true` に設定すると、この特徴を持つ点を公開します。

#### `iclude_second_return_points`

- **説明**: ガラス表面とその奥にある物体など、反射物体が 2 つ存在する点です。
- **用途**: `true` に設定すると、この特徴を持つ点を公開します。

#### `iclude_invalid_points`

- **説明**: 測定できなかった点です。
- **用途**: `true` に設定すると、この特徴を持つ点を公開します。

#### `iclude_blocked_points`

- **説明**: 汚れなどの異物が付着している箇所の点です。
- **用途**: `true` に設定すると、この特徴を持つ点を公開します。

#### `iclude_retro_points`

- **説明**: 再帰反射物体上の点です。道路標識などが該当します。
- **用途**: `true` に設定すると、この特徴を持つ点を公開します。

#### `iclude_retro_weak_points`

- **説明**: 再帰反射点のしきい値を超えない、弱い再帰反射物体上の点です。
- **用途**: `true` に設定すると、この特徴を持つ点を公開します。

#### `iclude_ambient_points`

- **説明**: レーザーが発光していないときに明るさが測定された点です。
- **用途**: `true` に設定すると、この特徴を持つ点を公開します。

### 空間フィルタリング

#### `min_altitude`（double、既定値: -90.0）

- **説明**: 仰角の下限値（度）
- **範囲**: 内部で `[-89.9°, 89.9°]` に制限されます。
- **用途**: この仰角より下にある点を除外します。

#### `max_altitude`（double、既定値: 90.0）

- **説明**: 仰角の上限値（度）
- **範囲**: 内部で `[-89.9°, 89.9°]` に制限されます。
- **用途**: この仰角より上にある点を除外します。

#### `min_azimuth`（double、既定値: -90.0）

- **説明**: 方位角の下限値（度）
- **範囲**: 内部で `[-89.9°, 89.9°]` に制限されます。
- **用途**: この方位角より左にある点を除外します。

#### `max_azimuth`（double、既定値: 90.0）

- **説明**: 方位角の上限値（度）
- **範囲**: 内部で `[-89.9°, 89.9°]` に制限されます。
- **用途**: この方位角より右にある点を除外します。

#### `min_distance`（double、既定値: 0.0）

- **説明**: 距離しきい値の下限（メートル）
- **用途**: この距離より近い点を除外します。

#### `max_distance`（double、既定値: `numeric_limits<float>::max()`）

- **説明**: 距離しきい値の上限（メートル）
- **用途**: この距離より遠い点を除外します。
- **注記**: 500 m を超える点は自動的に無効として除外されます。

### ネットワーク設定

#### `expected_sensor_ips`（vector<string>、既定値: 空）

- **説明**: 監視対象として想定するセンサー IP アドレスの一覧
- **用途**: 想定したセンサーに対して、積極的なタイムアウト検出を有効にします。
- **例**:

```yaml
expected_sensor_ips:
  - "192.168.1.10"
  - "192.168.1.11"
```

## 公開トピック

### 標準トピック

- **`cepton_pcl2`**: メインの点群トピック（`sensor_msgs::PointCloud2`）
- **`cepton_info`**: 統合センサー情報トピック
- **`cepton_sensor_status`**: センサーステータス監視トピック

### 条件付きトピック

- **`handle_<HANDLE>`**: ハンドルごとの点群（`pcl2_output_type` に `"IP"` を含む場合）
- **`serial_<SERIAL_NUMBER>`**: シリアル番号ごとの点群（`pcl2_output_type` に `"SN"` を含む場合）
- **`info_handle_<HANDLE>`**: センサーごとの情報トピック

## 点群フィールド

### 標準フィールド（常に含まれる）

- **x, y, z**（float32）: ROS 座標系における 3D 座標
- **intensity**（float32）: 反射率（0.01 倍にスケーリング）

### 任意フィールド（コンパイル時フラグ）

#### WITH_TS_CH_F フィールド

- **timestamp_s**（int32）: タイムスタンプの秒部分
- **timestamp_us**（int32）: タイムスタンプのマイクロ秒部分
- **flags**（uint16）: 点の品質フラグ
- **channel_id**（uint16）: センサーチャネル識別子

#### WITH_POLAR フィールド

- **azimuth**（float32）: 方位角（ラジアン）
- **elevation**（float32）: 仰角（ラジアン）
- **range**（float32）: 距離測定値（リターンなしの点では 0）

## 座標系変換

Publisher は Cepton の座標系から ROS 標準座標系へ自動的に変換します。

## メッセージ型

### CeptonSensorInfo

以下を含む包括的なセンサー情報です。

- `serial_number`、`handle`、`model_name`、`model`
- `part_number`、`firmware_version`
- `power_up_timestamp`、`time_sync_offset`、`time_sync_drift`
- `return_count`、`channel_count`、`status_flags`
- `temperature`、`fault_summary`、`fault_entries`

### CeptonSensorStatus

以下を含む監視メッセージです。

- `serial_number`、`handle`
- `status`（例: `SENSOR_TIMED_OUT`）

## ビルド設定

### CMake フラグ

- **`-DWITH_POLAR=OFF`**: 極座標フィールドを無効化します。
- **`-DWITH_TS_CH_F=OFF`**: タイムスタンプ、チャネル、フラグのフィールドを無効化します。

### ビルドコマンド

```bash
# すべてのフィールドを含む標準ビルド
colcon build --packages-select cepton_messages cepton_subscriber cepton_publisher

# フィールドを絞ったビルド
colcon build --packages-select cepton_messages cepton_subscriber cepton_publisher --cmake-args -DWITH_POLAR=OFF -DWITH_TS_CH_F=OFF
```

### 実行コマンド

```bash
source /opt/ros/<ros-version/setup.bash
source install/setup.bash

ros2 run cepton_publisher cepton_publisher_node --ros-args --params-file /path/to/parameters.yaml
```

## 監視とステータス

### センサーのタイムアウト検出

- **タイムアウト時間**: 3 秒（`SENSOR_POINTS_TIMEOUT` で設定可能）
- **監視スレッド**: 1 秒ごとにセンサーの状態を確認します。
- **タイムアウト時の動作**: `SENSOR_TIMED_OUT` ステータスを持つ `CeptonSensorStatus` メッセージを公開します。

## 注意事項

### パケットロス

特に複数の LiDAR ユニットを接続すると、データトラフィックが非常に大きくなり、パケットロスが発生しやすくなります。

受信バッファサイズを増やすなどして、設定を調整してください。

```bash
sudo sysctl -w net.core.rmem_max=2000000000
sudo sysctl -w net.core.rmem_default=536870912
sudo sysctl -w net.core.netdev_max_backlog=10000
```

上記はあくまで一例です。環境に合わせて値を調整してください。

### 不安定なフレームレート

フレームレートが不安定な場合は、[fastdds.xml](fastdds.xml) の使用を試してください。

```bash
# Fast DDS 2 の場合
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/ros/ros2/fastdds.xml
# Fast DDS 3 の場合
export FASTDDS_DEFAULT_PROFILES_FILE=/path/to/ros/ros2/fastdds.xml
```

