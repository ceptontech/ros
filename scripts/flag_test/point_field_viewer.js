"use strict";

// Keep all values in memory for zooming, but give Plotly at most 6,000 points.
// A broad view retains the minimum and maximum from every consecutive bucket.
const MAX_RENDER_POINTS = 6000;

function visiblePoints(values, lower, upper) {
  const first = Math.max(0, Math.floor(lower));
  const last = Math.min(values.length - 1, Math.ceil(upper));
  if (last < first) return {x: [], y: []};

  const x = [];
  const y = [];
  const add = index => {
    x.push(index);
    y.push(Number.isFinite(values[index]) ? values[index] : null);
  };
  const count = last - first + 1;
  if (count <= MAX_RENDER_POINTS) {
    for (let index = first; index <= last; index++) add(index);
    return {x, y};
  }

  add(first);
  const bucketSize = Math.ceil(count / Math.floor((MAX_RENDER_POINTS - 2) / 2));
  for (let begin = first; begin <= last; begin += bucketSize) {
    const end = Math.min(last + 1, begin + bucketSize);
    let minimum = Infinity;
    let maximum = -Infinity;
    let minIndex = -1;
    let maxIndex = -1;
    for (let index = begin; index < end; index++) {
      const value = values[index];
      if (!Number.isFinite(value)) continue;
      if (value < minimum) {
        minimum = value;
        minIndex = index;
      }
      if (value > maximum) {
        maximum = value;
        maxIndex = index;
      }
    }
    if (minIndex < 0) continue;
    if (minIndex === maxIndex) {
      if (minIndex !== first && minIndex !== last) add(minIndex);
    } else {
      for (const index of [Math.min(minIndex, maxIndex), Math.max(minIndex, maxIndex)]) {
        if (index !== first && index !== last) add(index);
      }
    }
  }
  add(last);
  return {x, y};
}

async function decodeSeries(encoded, count) {
  if (typeof DecompressionStream !== "function") {
    throw new Error("このブラウザはgzip展開に対応していません。新しいブラウザで開いてください。");
  }
  const raw = atob(encoded.trim());
  const compressed = Uint8Array.from(raw, character => character.charCodeAt(0));
  const stream = new Blob([compressed]).stream().pipeThrough(new DecompressionStream("gzip"));
  const buffer = await new Response(stream).arrayBuffer();
  if (buffer.byteLength !== count * 8) throw new Error("グラフの点数とデータ長が一致しません");
  const littleEndian = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;
  if (littleEndian) return new Float64Array(buffer);
  const view = new DataView(buffer);
  return Float64Array.from({length: count}, (_, index) => view.getFloat64(index * 8, true));
}

async function showChart(section) {
  const chart = section.querySelector(".point-chart");
  const encoded = section.querySelector(".point-series").textContent;
  const values = await decodeSeries(encoded, Number(chart.dataset.count));
  const last = values.length - 1;
  const field = chart.dataset.field;
  const samples = visiblePoints(values, 0, last);
  const trace = {
    type: "scattergl", mode: "markers", x: samples.x, y: samples.y,
    marker: {size: 4, color: chart.dataset.color},
    hovertemplate: "点の番号: %{x}<br>値: %{y}<extra></extra>",
  };
  const layout = {
    height: 420, template: "plotly_white", showlegend: false,
    margin: {l: 100, r: 30, t: 20, b: 65},
    font: {family: "system-ui, sans-serif"},
    xaxis: {title: "点の番号（0始まり）", range: [0, last], tickformat: ",d"},
    yaxis: {title: field, showexponent: "none", exponentformat: "none",
            separatethousands: true},
  };
  const config = {
    responsive: true, displaylogo: false,
    toImageButtonOptions: {format: "svg", filename: field},
  };
  chart.replaceChildren();
  await Plotly.newPlot(chart, [trace], layout, config);
  chart.on("plotly_relayout", event => {
    if (!Object.keys(event).some(key => key.startsWith("xaxis."))) return;
    const range = event["xaxis.autorange"] ? [0, last] : chart.layout.xaxis.range;
    if (!range) return;
    const next = visiblePoints(values, range[0], range[1]);
    void Plotly.restyle(chart, {x: [next.x], y: [next.y]}, [0]);
  });
}

if (typeof document !== "undefined") {
  document.addEventListener("DOMContentLoaded", () => {
    for (const section of document.querySelectorAll("section.chart")) {
      showChart(section).catch(error => {
        section.querySelector(".point-chart").textContent = `グラフの読み込みに失敗しました: ${error.message}`;
      });
    }
  });
}

if (typeof module !== "undefined") module.exports = {visiblePoints};
