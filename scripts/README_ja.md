# 長期稼働試験スクリプト `stability_test.py`

Cepton LiDAR ドライバ（ROS1 / ROS2）を実機複数台で長時間連続稼働させ、点群配信の
**周期・点数安定性・フレームドロップ・SensorInfo レート・プロセス生存・CPU/メモリのリーク**
を自動評価するスクリプトです。ROS1 と ROS2 を `--ros-version` で切り替えられます（既定は環境変数
`ROS_VERSION` から自動判定）。

## 計測アーキテクチャ（なぜ C++ プローブが必要か）

計測は 2 層に分かれています。

- **データプレーン = C++ 計測ノード `stability_probe`**（`tools/stability_probe_ros1|ros2`）。
  台ごとのトピック（ROS1: `/cepton3/points_sn_<SN>` / ROS2: `/serial_<SN>`）を購読し、
  到着時刻・`header.stamp`・`width` をトピック別 CSV に逐次記録するだけの ~100 行のノード。
  ドライバ非改変・Cepton SDK 非依存。
- **コントロールプレーン = Python (`stability_test.py`)**。Publisher/プローブの起動と
  `/proc` によるリソース監視、CSV の評価、グラフ・レポート生成。加えて **SensorInfo**
  （ROS1: `/cepton3/sensor_information` / ROS2: `/cepton_info`）はこの Python プロセスから
  直接購読します。1 メッセージ数百バイト・公称 2Hz と点群より 4 桁以上軽く GIL が
  ボトルネックにならないため、C++ プローブを介する必要がありません（全台分が 1 本の
  トピックに流れるので、メッセージ内の `serial_number` で台ごとに振り分けます）。

Python 購読（`--rate-method inproc`）を実機で使ってはいけない理由：全点設定では
トピック実流量が **349,960 点 × 32 B × 20 Hz × 4 台 ≈ 900 MB/s（7.2 Gbps）** に達し、
rospy/rclpy は全バイトが GIL 配下を通るため（実効 ~100–200 MB/s）追従できません。
さらに購読詰まりの逆圧で **Publisher の送信キュー（queue_size=50 × 4 topic × ~11 MB ≈ 2.2 GB）
が滞留し、メモリ評価まで汚染**されます（実測：Python 購読の 1 時間ランで RSS 2.5 GB。
プローブ方式ではこの滞留は発生しません）。C++ プローブなら ~900 MB/s は 1 コアの数%で、
計測がドライバ（DUT）を歪めません。プローブ自身の CPU/RSS もレポートに併記され、
計測が追従できていたことを確認できます。

## 依存

- 選択したバージョンの `rospy`（ROS1）または `rclpy`（ROS2）— トピック検出等の制御用
- **C++ プローブのビルド**（下記。`--rate-method inproc` の低レート・ドライランでは不要）
- グラフ出力に `matplotlib`（未インストール時はグラフをスキップし、数値評価は継続）
- CPU/メモリ計測は `/proc` を直接読むため追加依存なし（Ubuntu 前提）

## 事前準備

### ROS1 (Noetic)
```bash
source /opt/ros/noetic/setup.bash

# プローブを catkin ワークスペースへリンクしてビルド（初回のみ）
ln -s /path/to/repo/tools/stability_probe_ros1 catkin_ws/src/stability_probe
cd catkin_ws && catkin_make

source catkin_ws/devel/setup.bash   # cepton_ros と stability_probe の両方が見えること
```

### ROS2 (Humble 等)
```bash
source /opt/ros/humble/setup.bash

# プローブを colcon ワークスペースへリンクしてビルド（初回のみ）
ln -s /path/to/repo/tools/stability_probe_ros2 <colcon_ws>/src/stability_probe
cd <colcon_ws> && colcon build --packages-select stability_probe

source <colcon_ws>/install/setup.bash
source ros2/install/setup.bash       # cepton_publisher をビルドしたワークスペース
```

いずれも `ROS_VERSION` が設定されるので、`--ros-version` は省略できます。

## 使い方

`--aggregation-frame-count` は 1 設定/実行です（`1` ≈ 20Hz, `2` ≈ 10Hz）。
ROS1 では内部で `aggregate_frames`（`false`/`true`）に、ROS2 では `aggregation_frame_count`
にマッピングされます。

```bash
# ROS1 環境を source した状態で、1 台・600 秒・count=1（20Hz）
python3 scripts/stability_test.py --duration 600 --aggregation-frame-count 1

# 続けて count=2（10Hz、期待点数は自動で 2 倍の 699,920 に）
python3 scripts/stability_test.py --duration 600 --aggregation-frame-count 2

# 4 台まとめて試験する場合は台数を明示（--expected-sensors の既定は 1）
python3 scripts/stability_test.py --duration 600 --aggregation-frame-count 1 \
  --expected-sensors 4

# ROS2 環境を source すれば同じコマンドで ROS2 側を試験（--ros-version は自動）
python3 scripts/stability_test.py --duration 600 --aggregation-frame-count 1
```

既定で**全点設定**（後述）の一時パラメータを自動生成して Publisher を起動します。

## 主なオプション

| オプション | 既定 | 説明 |
|---|---|---|
| `--duration` | (必須) | 計測時間（秒） |
| `--aggregation-frame-count` | `1` | `1`(≈20Hz) か `2`(≈10Hz) |
| `--ros-version` | `$ROS_VERSION` | `1` か `2` |
| `--rate-method` | `probe` | `probe`=C++ 計測ノード（実機は必須）/ `inproc`=Python 購読（低レートのドライラン専用） |
| `--expected-sensors` | `1` | 検出必須の台数（不足なら前提未達で終了コード 2）。複数台試験では台数を明示指定する |
| `--inst-tolerance` | `0.1` | **瞬時** 1/dt の許容 Hz 誤差（仕様どおり。20Hz では ±0.25ms の間隔予算） |
| `--rate-tolerance` | `0.1` | **窓平均**レートの許容 Hz 誤差 |
| `--rate-window` | `1.0` | 窓平均のスライディング窓長（秒） |
| `--rate-basis` | `arrival` | 合否の基準。`arrival`（購読側の実受信＝配信ケイデンス）か `stamp`（センサ供給周期・参考）。両方常時併記 |
| `--info-rate` | `2.0` | SensorInfo の公称レート（Hz） |
| `--info-rate-tolerance` | `0.5` | SensorInfo レートの許容 Hz 誤差（瞬時 1/dt・窓平均の両方に適用） |
| `--info-rate-window` | `5.0` | SensorInfo の窓平均の窓長（秒）。点群より 1 桁低レートなので既定は長め |
| `--no-info-check` | (off) | SensorInfo を購読せずレート判定をスキップ（ドライラン用） |
| `--expected-points` | `349960` | SDK 公称点数。合否 = 全フレームの width == この値 × aggregation_count。`0` で「全フレーム同一」のみ判定（ドライラン用） |
| `--no-all-points` | (off) | 全点設定の自動上書きを無効化（既定は include_* 全 true・フィルタ全開） |
| `--warmup` | `5.0` | 起動直後の除外秒数（点群・SensorInfo とも各系列の先頭から適用） |
| `--drop-factor` | `1.5` | 間隔 > `factor × 公称周期` をドロップ判定 |
| `--mem-growth-threshold` | `1.0` | RSS 増加の不合格閾値（MB/min） |
| `--cpu-growth-threshold` | `5.0` | CPU 増加の不合格閾値（%/min） |
| `--resource-interval` | `1.0` | Publisher の CPU/メモリ・プロセス内部のサンプリング間隔（秒） |
| `--system-interval` | `5.0` | マシン全体（CPU/UDP/NIC/上位プロセス）のサンプリング間隔（秒）。`/proc` 全走査を伴うため粗め |
| `--sensor-interface` | 自動検出 | センサデータが届く NIC 名。`environment.json` の記録と NIC 統計に使用 |
| `--perf-clock` | (off) | `perf stat` で Publisher の実効クロックを計測。**既定で無効**（下記参照） |
| `--startup-timeout` | `30.0` | 台ごとトピック検出の待機上限（秒） |
| `--config-path` | 版ごとの既定 YAML | 元にするパラメータファイル |
| `--output-dir` | `scripts/stability_output/<日時>` | 出力先 |

## 評価項目（合否）

- **周期（2 段判定）**: warmup 後の各台について
  - 瞬時 1/dt（`rostopic hz -w 1` 相当）の**全サンプル**が `公称 ± inst-tolerance` 内
  - `rate-window` 秒スライディング窓の平均レートが常に `公称 ± rate-tolerance` 内
  - 両方 PASS で合格。あわせて **dt 分布（p50/p99/p99.9/max）** をレポートし、
    `jitter.png` で許容境界に対するテールの実在を確認できる（瞬時 ±0.1Hz = ±0.25ms は
    OS ジッタで偽 FAIL し得るため、実測分布を見て `--inst-tolerance` の扱いを判断する）
- **点数安定性**: 全フレームの `width` が完全一致し、かつ **SDK 公称 349,960 ×
  aggregation_count に一致**すること（全点設定が前提。点の欠けは UDP ロス／フレーム
  組み立て欠損を意味する）。あわせて **仕様値と一致しないフレームの割合**
  （`off_spec_ratio` / `off_spec_frames`）を常に算出し、レポートと `summary.json` に
  出力する（合否とは別に「どれくらいの頻度で欠けたか」を定量化するための指標。
  `--expected-points 0` のときは仕様値の代わりに最頻 width を基準にする）
- **SensorInfo レート**: 台ごとに SensorInfo の受信間隔から求めた瞬時 1/dt と
  `--info-rate-window` 秒スライディング窓の平均が、いずれも
  `--info-rate ± --info-rate-tolerance`（既定 **2 ± 0.5 Hz**）内であること。点群トピックが
  見つかった台から SensorInfo が 1 通も届かない場合も不合格。判定は到着時刻基準のみ
  （ROS2 のメッセージに header は無く、ROS1 の `header.stamp` もドライバが publish 時に
  付ける時刻でセンサ時刻ではないため、点群のような stamp 基準の併記はしない）
- **フレームドロップ**: 間隔が `drop-factor × 公称周期` を超える箇所が 0 であること
- **プロセス生存**: duration 中に Publisher プロセスが異常終了しないこと
- **CPU/メモリ**: RSS / CPU の線形回帰の傾きが閾値を超えて増加し続けないこと

### 全点設定（`--all-points`、既定 ON）

点数を公称値と照合するため、生成する試験パラメータで include_*（ROS1:
saturated/second_return/invalid/noise/blocked/retro/retro_weak/ambient、ROS2 は宣言済み
キーのみ）を全 true、min/max altitude・azimuth を ±90、min_distance 0、max_distance 1000
に上書きします。**注意**: ドライバには設定で無効化できないハードコードの 500m フィルタ
（`publisher_nodelet.cpp` の `distance_squared >= 500*500`）があるため、公称値との厳密一致が
成立するかは初回実測で確認してください。一定のオフセットが出る場合は、その実測定数を
`--expected-points` に指定して運用します。

### 環境スナップショット `environment.json`

Publisher 起動直後・計測開始前に一度だけ収集します。**ドライバのパラメータには現れないが
結果を左右する設定**を、測定値と同じディレクトリに残すのが目的です。

- `host` … ホスト名、カーネル、**カーネル起動パラメータ**（`isolcpus`/`nohz_full`/`mitigations` は
  レイテンシに直結）、OS ディストリビューション、**マシンのベンダー・製品名・BIOS**（DMI）、CPU
- `sysctl` … `net.core.{r,w}mem_{max,default}`、`optmem_max`、`netdev_max_backlog`、`net.ipv4.udp_mem`
- `sockets` … **Publisher と購読側が実際に得たソケットバッファ**（`ss -ulmn` の `rb`/`tb`）。
  カーネルはソケット生成時の `*mem_default` を焼き込むため、sysctl の現在値ではなくこちらが実効値。
  送信側だけでなく**受信側も記録**する
- `topic_qos` … 各トピックの**publisher 側と subscriber 側の QoS**（reliability / durability /
  history / depth / liveliness）。両者の食い違いは配信の乱れの典型的な原因なので片側では足りない
- `rmw_loaded` … Publisher が実際にロードしている `librmw_*.so`。`RMW_IMPLEMENTATION` は未設定で
  ディストロ既定が使われることが多く、環境変数だけでは実体を特定できない
- `nic` … MTU、**ドライバ名/バージョン/ファームウェア**、**PHY リンク速度・Duplex・
  オートネゴシエーション**、ポート種別、**割り込みコアレシング**（`rx-usecs` 等）、
  **オフロード設定**（63 項目）、リングバッファ。コアレシングと GRO/LRO は、カーネルが
  パケットの束をいつ上位へ渡すかを決めるため、定常的なセンサ流を不均一な流れに変え得る
- `firewall` … ufw/firewalld/nftables のサービス状態と、ロード済み netfilter モジュール
- `min_send_buffer_bytes` / `message_per_send_buffer` … 1 メッセージが送信バッファの何倍かを算出。
  **この値が 1 を大きく超えていると、publish のたびにカーネルの送出待ちが発生する**
- `cpu_scaling` … governor / driver / EPP / intel_pstate の status・no_turbo・min_perf_pct。
  `intel_pstate` では `powersave` でもターボまで上がるため、governor 名だけでは判断できない
- `ros` … `RMW_IMPLEMENTATION`、DDS プロファイル、`ROS_DOMAIN_ID` など
- `dds_shm_segments` … Fast DDS の共有メモリセグメントとサイズ。
  **メッセージがセグメントに収まらなければ UDP にフォールバックし、送信バッファが効いてくる**
- `driver_revision` … `git describe`（どのドライバで測ったかの記録）

`udp_rcvbuf_err` と `nic_drop` は記録のみで、合否には含めていません（妥当な閾値をまだ
持たないため）。レポートには情報行として増分が出ます。

### レート/ドロップの 2 つの基準（arrival と stamp）

周期とドロップは **到着(arrival)** と **センサ時刻(stamp)** の両方で常に算出し併記します
（レポートの `*` が合否採用側、`--rate-basis` で選択）。

- **arrival（既定）**: プローブがメッセージを受信した実時刻の間隔。ROS 利用者が体感する
  配信ケイデンスそのもの。C++ プローブなら高帯域でも信頼できる。
- **stamp**: センサ FW が付与するタイムスタンプの間隔＝**センサの供給周期**。ドライバの
  publish 周期そのものではないため参考値。arrival が乱れたとき「入力（センサ供給）は
  正常だったか」の切り分けに使う。

## 出力

`--output-dir` に以下が生成されます。

- `summary.json` … 全評価結果と総合判定（arrival/stamp 両基準・inst/windowed・dt 分布を含む）
- `sensor_<...>.csv` … 台ごとの到着時刻・stamp・width（プローブが記録、評価後に瞬時 Hz 等の列を追記）
- `sensor_info.csv` … SensorInfo の全受信メッセージを到着順に記録（到着時刻・`serial_number`・
  `status_flags`・`temperature`・`fault_summary`・`fault_entries` など全フィールド）。
  ※ ROS1 ドライバは `temperature` を設定しないため ROS1 では常に 0
- `resource.csv` … Publisher の RSS(MB) / CPU(%) 時系列
- `resource_probe.csv` … プローブ自身の RSS / CPU 時系列（計測の信頼性確認用）
- `environment.json` … **計測時のマシン設定スナップショット**（後述）
- `resource_system.csv` … マシン全体の時系列。`cpu_percent` / `cpu_max_core_pct` / `load1` /
  `temp_c` / `net_rx_softirq_s` / `udp_in_s` / `udp_rcvbuf_err_s` / `nic_rx_mbps` /
  `nic_drop_s` / `freq_{min,median,max}_mhz` / `top_processes`（各窓で CPU を最も食った
  プロセス上位 3 つ）。周波数は**マシン全体の文脈**で、どのコアが被試験プロセスを
  実行したかは主張しない。`cpu_percent` は **1 コア = 100%** 表記（20 コア全負荷なら 2000%）で、
  `resource.csv` の Publisher CPU と同じ尺度なので直接読み比べられる。
  `cpu_max_core_pct` だけは「1 コアの使用率」なので 0〜100
- `resource_process.csv` … Publisher プロセスの内部。`threads` / `vmrss_mb` / `vmsize_mb` /
  `vmas` / `minflt_s` / `hot_thread_cpu_pct`（最も CPU を使ったスレッドの利用率）/
  `hot_thread_last_cpu`（読み取り時点で最後に載っていたコア。**滞在時間ではない**）
- `publisher_clock.csv` … `--perf-clock` 指定時のみ。`effective_ghz` = `cycles / task-clock` で、
  **プロセスに帰属した実効クロック**。スレッドがコアを渡り歩いても正しく積算される。
  `perf_event_paranoid` が高いと取得できず、その理由が `summary.json` に記録される。

  **既定で無効な理由**: 本スクリプトの計測はすべて `/proc`・`/sys` の読み取りで被試験プロセスに
  触れませんが、`perf stat -p` だけは例外で per-task イベントを対象プロセスに取り付けます。
  サンプリング割り込みは発生しない（カウンティングモード）ものの、対象のコンテキストスイッチ毎に
  カウンタの退避/復元が入り、さらに**対象が生成する全スレッドにイベントが複製されます**。
  ROS2 ドライバは publish 毎にスレッドを生成する（毎秒 10 個）ため、この点で不利な形です。
  オーバーヘッドは 1% を大きく下回ると見積もられますが未実測であり、publish が 1 メッセージ
  あたりの予算ぎりぎりで動いている状況では計測行為が現象を変え得ます。必要なときだけ
  明示的に有効化してください。被試験プロセスに一切触れない代替として `turbostat` の
  per-core `Bzy_MHz`（要 root）があり、マシンがほぼアイドルで Publisher が支配的な負荷なら
  良い近似になります
- `system.png` … マシン CPU・ホットコア周波数・温度・NIC 受信の経時変化。
  点群が束になって届いた区間を網掛けで重ねてあり、配信が乱れている間もマシン側が
  平坦かどうかを一目で確認できる
- `framerate.png` … 配信レートの経時変化。**窓平均が主系列（実線）、瞬時 1/dt は淡色**。
  許容範囲の上下限を点線で表示（スパイクは上限でクリップし件数注記）
- `info_framerate.png` … SensorInfo レートの経時変化（台ごと）。framerate.png と同じ体裁で
  窓平均が実線・瞬時 1/dt が淡色、`--info-rate ± --info-rate-tolerance` の上下限を点線表示
- `jitter.png` … dt ヒストグラム（対数 y 軸）。公称周期と瞬時許容境界を縦線表示
- `cpu.png` … Publisher CPU 使用率の経時変化
- `memory.png` … Publisher RSS メモリの経時変化（増加傾向の回帰直線つき）
- `params_ros1.yaml` / `params_ros2.yaml` … 実際に使用した生成パラメータ

終了コード: 全合格 `0` / いずれか不合格 `1` / 前提未達（台数不足・プローブ未ビルド・
プローブ異常終了等）`2`。

## 高スループット時の注意（環境側）

VistaUltra-N を使用する場合、生 UDP で 1 台あたり約 400 Mbps × 4 台 ≈ 1.6 Gbps、ROS トピック上では
全点設定で ≈ 7.2 Gbps を捌く前提です。Publisher（C++/SDK）と試験 PC 側の要件：

- 帯域を捌ける NIC
- カーネル**受信**バッファの拡大（`sysctl net.core.rmem_max` / `rmem_default` など）
- カーネル**送信**バッファの拡大（`sysctl net.core.wmem_max` / `wmem_default`、ROS1 の TCPROS
  では `net.ipv4.tcp_wmem` の最大値も）。**Publisher（送信側）自身のソケットに効く**ため、
  受信側だけを拡大しても不十分（下記参照）。ROS2/Fast DDS でも同様で、メッセージが共有メモリ
  セグメントに収まらず UDP にフォールバックする場合に効いてくる。実際に反映されたかは
  `environment.json` の `publisher_sockets` と `message_per_send_buffer` で確認できる
- 負荷が高すぎる場合はドライバをメッセージ縮小ビルドして 1 フレームのデータ量を減らせます
  （ROS1/ROS2 とも `catkin_make` / `colcon build` 時に `-DWITH_POLAR=OFF -DWITH_TS_CH_F=OFF`）。
  ※点数照合（width）はフィールド数に依存しないため縮小ビルドでも成立します

### なぜ送信バッファ（wmem）が重要か

ROS1 の TCPROS は TCP（双方向）なので、フロー制御は受信側だけでなく**送信側（Publisher自身）**の
ソケットバッファにも依存します。送信バッファが小さいと `publish()` の内部の `write()`/`send()` が
受信側の読み出し待ちでブロックし、それが **Publisher 自身の publish タイミングのジッタ**として
直接観測されます（購読側を高速化しても解消しない）。

1 メッセージのサイズは 349,960 点 × 32 B/点 ≈ **11.2 MB**（`aggregation_frame_count=2` なら
699,920 点 ≈ **22.4 MB**）で、センサ 1 台あたりのデータレートは集約設定によらず一定で
約 **224 MB/s**（11.2 MB × 20 Hz = 22.4 MB × 10 Hz）です。Ubuntu の典型的な既定値
（`wmem_default`/`wmem_max` ≈ 208 KB、TCP 自動チューニング上限 `tcp_wmem` 最大値 ≈ 4 MB）は、
**1 メッセージ自体より小さい**ため、受信側が正常でも `write()` はカーネルが実際にバイトを
送出するのを待たざるを得ません。「バッファが全く drain されない」と仮定した場合の充填時間
（バッファサイズ ÷ 224 MB/s）は：

| バッファ | サイズ | 充填時間 |
|---|---|---|
| 既定 `wmem_default`/`wmem_max` | ≈208 KB | ≈0.95 ms |
| TCP 自動チューニング上限（`tcp_wmem` 最大値） | ≈4 MB | ≈18.7 ms |
| 拡大後（例: 128 MB） | 128 MB | ≈600 ms（count=1 で約12フレーム分） |

つまり既定設定では数十ms程度の受信側の遅延でも即座に送信側がブロックし得ます。`rmem` と
同様に `wmem`/`tcp_wmem` も拡大することを推奨します：

```bash
sudo sysctl -w net.core.wmem_max=134217728
sudo sysctl -w net.core.wmem_default=134217728
sudo sysctl -w net.ipv4.tcp_wmem="4096 87380 134217728"
```

## 実機なしでの動作確認（ドライラン）

疑似トピックを流して検出・評価・レポート生成を確認できます。疑似トピックは点数が公称と
異なるため `--expected-points` を実際の width（または `0`）にします。プローブをビルド済みなら
`--rate-method probe` のまま、未ビルドなら `--rate-method inproc` を指定します。
SensorInfo は流れないので `--no-info-check` を付けます（付けないと SensorInfo レートが
不合格になります）。

### ROS1
```bash
# 別ターミナルで roscore を起動後
rostopic pub -r 20 /cepton3/points_sn_1 sensor_msgs/PointCloud2 '{width: 100, height: 1}'
# （複数台分はそれぞれ別トピック名で起動）
python3 scripts/stability_test.py --no-launch --duration 15 --expected-sensors 1 \
  --expected-points 100 --inst-tolerance 3 --rate-tolerance 3 --rate-method inproc \
  --no-info-check
```

### ROS2
```bash
ros2 topic pub -r 20 /serial_1 sensor_msgs/msg/PointCloud2 '{width: 100, height: 1}'
python3 scripts/stability_test.py --no-launch --duration 15 --expected-sensors 1 \
  --expected-points 100 --inst-tolerance 3 --rate-tolerance 3 --no-info-check
```

※ `ros2 topic pub` は `header.stamp` が常に 0 のため、stamp 基準の行は n/a になります
（実ドライバでは正しく出ます）。
