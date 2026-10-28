#!/usr/bin/env python3

"""Build an offline-capable point-index/field-value report from CI snapshots."""

import argparse
from array import array
import base64
from datetime import datetime, timezone
import gzip
from html import escape
import json
import math
import os
from pathlib import Path
import sys

from plotly.offline import get_plotlyjs


CASES = (
    ("baseline", "基準（1フレーム）", "#2563eb"),
    ("aggregate_2", "2フレーム集約", "#d97706"),
)

STYLE = """
body { margin: 0; background: #f4f6fa; color: #1e293b;
       font: 16px/1.7 system-ui, sans-serif; }
main { max-width: 1280px; margin: auto; padding: 28px 20px 60px; }
h1 { font-size: 1.8rem; line-height: 1.4; }
h2 { font-size: 1.15rem; }
a { color: #1d4ed8; }
nav { display: flex; flex-wrap: wrap; gap: 8px 20px; margin: 20px 0; }
section, .chart { background: white; border: 1px solid #dbe2ed;
                  border-radius: 8px; padding: 18px; margin: 20px 0; }
table { border-collapse: collapse; width: 100%; }
th, td { text-align: left; padding: 10px; border-bottom: 1px solid #e2e8f0; }
.table { overflow-x: auto; }
code { overflow-wrap: anywhere; }
.muted { color: #475569; font-size: 0.9rem; }
"""


def document(title, body, use_plotly=False):
    script = (
        '<script src="plotly.min.js" defer></script>'
        '<script src="point_field_viewer.js" defer></script>'
    ) if use_plotly else ""
    return (
        '<!doctype html><html lang="ja"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1">'
        f"<title>{escape(title)}</title><style>{STYLE}</style>{script}"
        f"</head><body><main>{body}</main></body></html>"
    )


def encode_values(values):
    """Keep every original numeric value in a compact, portable float64 series."""
    series = array("d", (math.nan if value is None else float(value) for value in values))
    if sys.byteorder != "little":
        series.byteswap()
    return base64.b64encode(gzip.compress(series.tobytes(), compresslevel=6, mtime=0)).decode("ascii")


def render_report(data_dir, output_dir):
    snapshots = {}
    for name, _, _ in CASES:
        with (data_dir / f"{name}.json").open(encoding="utf-8") as source:
            snapshot = json.load(source)
        if snapshot["point_count"] <= 0 or not snapshot["fields"]:
            raise ValueError(f"{name}: snapshot has no points or fields")
        for field in snapshot["fields"]:
            if len(field["values"]) != snapshot["point_count"]:
                raise ValueError(f"{name}: wrong point count for {field['name']}")
        field_names = {field["name"] for field in snapshot["fields"]}
        for required_field in ("timestamp_s", "timestamp_us", "relative_time_us"):
            if required_field not in field_names:
                raise ValueError(f"{name}: {required_field} field is missing")
        snapshots[name] = snapshot

    field_names = list(dict.fromkeys(
        field["name"] for snapshot in snapshots.values() for field in snapshot["fields"]
    ))
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "plotly.min.js").write_text(get_plotlyjs(), encoding="utf-8")
    viewer_source = Path(__file__).with_name("point_field_viewer.js")
    (output_dir / viewer_source.name).write_bytes(viewer_source.read_bytes())
    links = [(field_name, f"field-{index:02d}.html")
             for index, field_name in enumerate(field_names)]
    navigation = '<nav><a href="index.html">一覧</a>' + "".join(
        f'<a href="{filename}">{escape(name)}</a>' for name, filename in links
    ) + "</nav>"

    rows = []
    for case_name, label, _ in CASES:
        snapshot = snapshots[case_name]
        rows.append(
            f"<tr><th>{label}</th><td>{snapshot['point_count']:,}</td>"
            f"<td><code>{escape(snapshot['stamp'])}</code></td>"
            f"<td>{escape(snapshot['topic'])}</td>"
            f"<td>{escape(snapshot['frame_id'])}</td>"
            f'<td><a href="data/{case_name}.json" download>JSON</a></td></tr>'
        )
        # Keep raw data beside the report, even when input/output dirs differ.
        target = output_dir / "data" / f"{case_name}.json"
        target.parent.mkdir(parents=True, exist_ok=True)
        source = data_dir / f"{case_name}.json"
        if source.resolve() != target.resolve():
            target.write_bytes(source.read_bytes())
    metadata = (
        '<section><h2>受信したメッセージ</h2><div class="table"><table>'
        '<tr><th>条件</th><th>点数</th><th>Header stamp</th><th>Topic</th>'
        '<th>Frame ID</th><th>データ</th></tr>' + "".join(rows) + "</table></div>"
        '<p>各条件でsubscribeした1メッセージを記録しています。2フレーム集約は、'
        '2フレーム分を連結してpublishした1メッセージです。</p></section>'
    )
    instructions = (
        '<p>横軸：受信メッセージ内の点の番号（0始まり）／縦軸：フィールドの値。'
        '受信順と全点の元データを保持します。NaN・Infはその番号を残して欠損として扱います。</p>'
        '<p class="muted">ドラッグで拡大、ダブルクリックで範囲をリセット。'
        '広い範囲では各区間の最小値と最大値を描画し、拡大すると個々の点を表示します。'
        '点にカーソルを合わせると番号と値を表示します。'
        'timestamp_s・timestamp_us・relative_time_usは受信した値をそのまま描画します。</p>'
    )
    generated = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    commit = os.environ.get("GITHUB_SHA", "local")
    provenance = f'<p class="muted">生成：{generated} / commit: <code>{escape(commit)}</code></p>'
    server = os.environ.get("GITHUB_SERVER_URL", "https://github.com")
    repository = os.environ.get("GITHUB_REPOSITORY")
    run_id = os.environ.get("GITHUB_RUN_ID")
    if repository and run_id:
        run_url = f"{server}/{repository}/actions/runs/{run_id}"
        provenance += f'<p><a href="{escape(run_url, quote=True)}">Actions実行結果</a></p>'

    field_rows = []
    for field_name, filename in links:
        types = []
        charts = []
        for case_name, label, color in CASES:
            snapshot = snapshots[case_name]
            field = next((f for f in snapshot["fields"] if f["name"] == field_name), None)
            if field is None:
                charts.append(f'<section><h2>{label}</h2>このフィールドはありません。</section>')
                continue
            types.append(field["datatype"])
            encoded = encode_values(field["values"])
            charts.append(
                f'<section class="chart"><h2>{label}</h2>'
                f'<div class="point-chart" data-field="{escape(field_name, quote=True)}" '
                f'data-color="{color}" data-count="{snapshot["point_count"]}">'
                '<p>グラフを読み込み中…</p></div>'
                f'<script type="application/octet-stream" class="point-series">{encoded}</script>'
                '</section>'
            )
        body = (
            f"<h1>{escape(field_name)}</h1>{navigation}{instructions}"
            f'{"".join(charts)}{metadata}{provenance}'
        )
        (output_dir / filename).write_text(document(field_name, body, use_plotly=True),
                                          encoding="utf-8")
        field_rows.append(
            f'<tr><td><a href="{filename}">{escape(field_name)}</a></td>'
            f'<td>{escape(" / ".join(dict.fromkeys(types)))}</td></tr>'
        )

    body = (
        f"<h1>ROS2 PointCloud2 フィールド別グラフ</h1>{instructions}{metadata}"
        '<section><h2>フィールドを選択</h2><p>各ページで1フレーム・'
        '2フレーム集約を上下に表示します。</p>'
        '<table><tr><th>フィールド</th><th>データ型</th></tr>' + "".join(field_rows)
        + f"</table></section>{provenance}"
    )
    (output_dir / "index.html").write_text(document("ROS2 フィールド別グラフ", body),
                                         encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    render_report(args.data_dir, args.output_dir)


if __name__ == "__main__":
    main()
