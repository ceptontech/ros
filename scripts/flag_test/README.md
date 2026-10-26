# ROS2 フィールド別グラフ

`flag-test-ros2.yml` の点数テストで受信した、基準と2フレーム集約の
`PointCloud2` を各1メッセージ保存し、フィールドごとのHTMLグラフを生成します。
点数の判定に使ったメッセージそのものを保存するので、別のフレームを描画しません。

- 横軸：メッセージ内の点の番号（0始まり）。並べ替え・間引きはしません。
- 縦軸：各フィールドの実際の値。
- フィールドごとのページに、基準と2フレーム集約を上下に表示します。
- 集約側も受信するのは1メッセージです。その中に2フレーム分の点が入ります。
- 拡大、移動、値のホバー表示、SVGでの画像保存ができます。

現在のCIは `WITH_TS_CH_F` と既定の `WITH_POLAR` を有効にしているので、
`x`, `y`, `z`, `intensity`, `timestamp_s`, `timestamp_us`, `flags`, `channel_id`,
`azimuth`, `elevation`, `range` の11ページを生成します。
フィールドは `PointCloud2.fields` から取得するため、将来追加されたフィールドも
自動で対象になります。配列のフィールドは要素ごとに別ページになります。
`flags` は受信した整数値、timestampの2フィールドは受信した値をそのまま描画します。
NaN・Infはその点の番号を維持して欠損として扱います。

## CIで確認する

PR実行では、Actionsの実行結果にある `ros2-point-fields` Artifactをダウンロードし、
展開した `index.html` をブラウザで開いてください。
PlotlyのJavaScriptとデータを同梱しているため、外部CDNやローカルサーバーは不要です。
Artifactは14日間保存します。

GitHub Pagesは最初にリポジトリの **Settings → Pages → Build and deployment →
Source → GitHub Actions** を選んで有効にします。
`main` へのpush、または `main` を選んだ手動実行が成功すると、
`github-pages` 環境へ公開します。URLはデプロイジョブに表示されます。
PRからの実行はArtifactのみを作成します。
Pagesは直近にデプロイしたレポートを表示し、実行履歴はArtifactで確認できます。
レポートには元の実行へのリンク、commit SHA、トピック、header stamp、点数とJSONを含みます。

GitHub公式：[カスタムワークフローによるPages公開](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages)

## ローカルで記録・生成する

ROS2 Jazzyとビルド済みドライバの環境を読み込み、publisherを起動する前にsubscriberを起動します。
既存のテストと同じく、両条件でheader stamp `80.557132000` のメッセージを選択します。
基準の例：

```bash
python3 scripts/flag_test/count_points_ros2.py \
  --timeout 120 --target-stamp 80.557132000 \
  --output-json artifacts/ros2-point-fields/data/baseline.json
```

別ターミナルで、同じ `ROS_DOMAIN_ID` とROS2環境を使ってpublisherを起動します。
CIと同じキャリブレーションの配置も必要です。

```bash
ros2 run cepton_publisher cepton_publisher_node \
  --ros-args --params-file scripts/flag_test/params/baseline.yaml
```

基準の記録後にpublisherを終了し、2フレーム集約ではsubscriberの出力先を
`artifacts/ros2-point-fields/data/aggregate_2.json` に変更します。
publisherには `-p aggregation_frame_count:=2` を追加します。
両方のJSONができたらHTMLを生成します。

```bash
sudo apt-get install --no-install-recommends python3-venv
python3 -m venv /tmp/cepton-pointcloud-plots
/tmp/cepton-pointcloud-plots/bin/pip install plotly==5.24.1
/tmp/cepton-pointcloud-plots/bin/python scripts/flag_test/render_point_fields.py \
  --data-dir artifacts/ros2-point-fields/data \
  --output-dir artifacts/ros2-point-fields
```

生成物は `artifacts/ros2-point-fields/` にまとまり、Git管理の対象外です。
デコーダーはROS2 Jazzyの全8データ型、エンディアン、フィールドのoffset、point_step、
row_step、配列のcountに従って読み出します。

```bash
python3 -m unittest discover -s scripts/flag_test -p 'test_*.py'
```
