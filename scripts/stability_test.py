#!/usr/bin/env python3
"""Long-duration stability test for the Cepton ROS1/ROS2 point cloud publisher.

Launches the publisher against live sensors, subscribes to the per-sensor point
cloud topics, and evaluates:
  - publish rate (instantaneous, rostopic-hz-`-w 1`-equivalent) within tolerance
  - per-frame point count stability (every frame identical)
  - frame drops (gaps in the observed publish cadence)
  - publisher process liveness (no abnormal exit within the duration)
  - publisher CPU / memory not growing without bound

ROS1 and ROS2 are switchable with --ros-version (default: $ROS_VERSION). The two
versions differ only inside the ROS backend (client API, launch, topic naming,
aggregation parameter); the evaluation, resource monitoring and reporting core is
shared and ROS-agnostic.

Only the point cloud message header + width are parsed (never the point payload),
so the measurement path stays light under high-bandwidth sensors.
"""

import argparse
import csv
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

# Okabe-Ito colorblind-safe categorical palette, used in fixed order per sensor.
SENSOR_COLORS = [
    "#0072B2", "#E69F00", "#009E73", "#D55E00",
    "#CC79A7", "#56B4E9", "#F0E442", "#000000",
]


# --------------------------------------------------------------------------- #
# PointCloud2 header parsing (header.stamp + width only, no point payload)
# --------------------------------------------------------------------------- #
def parse_ros1_pc2_header(buf):
    """Parse (stamp_sec, width) from a serialized ROS1 sensor_msgs/PointCloud2.

    TCPROS body layout: seq(u32) secs(u32) nsecs(u32) frame_len(u32)
    frame_id(frame_len) height(u32) width(u32) ...
    """
    secs, nsecs = struct.unpack_from("<II", buf, 4)
    (flen,) = struct.unpack_from("<I", buf, 12)
    off = 16 + flen
    _height, width = struct.unpack_from("<II", buf, off)
    return secs + nsecs * 1e-9, width


def parse_ros2_pc2_header(buf):
    """Parse (stamp_sec, width) from a CDR-serialized ROS2 PointCloud2.

    4-byte encapsulation header, then (aligned to the 4-byte body start):
    sec(i32) nanosec(u32) frame_len(u32) frame_id(frame_len) [pad] height(u32) width(u32)
    """
    endian = "<" if buf[1] == 1 else ">"
    sec, nanosec = struct.unpack_from(endian + "iI", buf, 4)
    (flen,) = struct.unpack_from(endian + "I", buf, 12)
    off = 16 + flen
    # Re-align to 4 bytes relative to the body start (offset 4) before height.
    rel = (off - 4 + 3) & ~3
    off = 4 + rel
    _height, width = struct.unpack_from(endian + "II", buf, off)
    return sec + nanosec * 1e-9, width


# --------------------------------------------------------------------------- #
# /proc based process monitoring (pure stdlib, Linux)
# --------------------------------------------------------------------------- #
def _read_ppid_comm(pid):
    with open("/proc/%d/stat" % pid) as f:
        data = f.read()
    rp = data.rfind(")")
    comm = data[data.find("(") + 1:rp]
    rest = data[rp + 2:].split()
    return int(rest[1]), comm  # ppid, comm


def read_rss_mb(pid):
    with open("/proc/%d/status" % pid) as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return 0.0


def read_cpu_ticks(pid):
    with open("/proc/%d/stat" % pid) as f:
        data = f.read()
    rp = data.rfind(")")
    rest = data[rp + 2:].split()
    return int(rest[11]) + int(rest[12])  # utime + stime (fields 14,15)


def resolve_target_pid(root_pid, name_hint, timeout=10.0):
    """Return a descendant of root_pid whose comm contains name_hint, else root."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        children = defaultdict(list)
        comms = {}
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            pid = int(entry)
            try:
                ppid, comm = _read_ppid_comm(pid)
            except (OSError, ValueError, IndexError):
                continue
            children[ppid].append(pid)
            comms[pid] = comm
        stack = [root_pid]
        seen = set()
        while stack:
            p = stack.pop()
            if p in seen:
                continue
            seen.add(p)
            if p != root_pid and name_hint in comms.get(p, ""):
                return p
            stack.extend(children.get(p, []))
        if name_hint in comms.get(root_pid, ""):
            return root_pid
        time.sleep(0.3)
    return root_pid


class _DummyPopen:
    """Stand-in for attach mode: liveness comes from /proc only."""

    def poll(self):
        return None


def _stop_process(popen, name):
    """Stop a launched process tree: SIGINT, then escalate if needed.

    Signals are addressed to the whole process group (launches use
    start_new_session=True) because wrappers like `ros2 run` may exit on a
    direct SIGINT without stopping the actual node, leaving it orphaned.
    """
    if popen is None or popen.poll() is not None:
        return

    def signal_group(sig):
        try:
            os.killpg(os.getpgid(popen.pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                popen.send_signal(sig)
            except Exception:
                pass

    signal_group(signal.SIGINT)
    try:
        popen.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        pass
    print("WARNING: %s did not stop on SIGINT; escalating" % name,
          file=sys.stderr, flush=True)
    signal_group(signal.SIGTERM)
    try:
        popen.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    signal_group(signal.SIGKILL)


class ResourceMonitor(threading.Thread):
    """Samples RSS/CPU of a pid and detects abnormal process exit."""

    def __init__(self, pid, popen, interval):
        super().__init__(daemon=True)
        self.pid = pid
        self.popen = popen
        self.interval = interval
        self.samples = []  # (t_rel, rss_mb, cpu_percent)
        self.crashed = False
        self.crash_time = None
        self._stop_event = threading.Event()
        self._start_wall = None

    def stop(self):
        self._stop_event.set()

    def _alive(self):
        if self.popen.poll() is not None:
            return False
        return os.path.exists("/proc/%d" % self.pid)

    def run(self):
        clk = os.sysconf("SC_CLK_TCK")
        self._start_wall = time.time()
        prev_ticks = None
        prev_t = None
        while not self._stop_event.is_set():
            if not self._alive():
                self.crashed = True
                self.crash_time = time.time() - self._start_wall
                return
            try:
                rss = read_rss_mb(self.pid)
                ticks = read_cpu_ticks(self.pid)
            except OSError:
                self.crashed = True
                self.crash_time = time.time() - self._start_wall
                return
            now = time.time()
            cpu = None
            if prev_ticks is not None and now > prev_t:
                cpu = (ticks - prev_ticks) / clk / (now - prev_t) * 100.0
            self.samples.append((now - self._start_wall, rss, cpu))
            prev_ticks, prev_t = ticks, now
            self._stop_event.wait(self.interval)


# --------------------------------------------------------------------------- #
# Per-sensor measurement buffers
# --------------------------------------------------------------------------- #
class SensorData:
    def __init__(self, topic):
        self.topic = topic
        self.arrivals = []  # wall-clock receive time (float sec)
        self.stamps = []    # sensor header.stamp (float sec) - diagnostic only
        self.widths = []    # per-frame point count

    def add(self, arrival, stamp, width):
        self.arrivals.append(arrival)
        self.stamps.append(stamp)
        self.widths.append(width)


# --------------------------------------------------------------------------- #
# ROS backends
# --------------------------------------------------------------------------- #
def _override_yaml_keys(lines_in, overrides, indent_for_new=""):
    """Replace scalar `key: value` lines; append keys that were absent."""
    lines = []
    remaining = dict(overrides)
    for line in lines_in:
        stripped = line.strip()
        matched = None
        for key in remaining:
            if stripped.startswith(key + ":"):
                matched = key
                break
        if matched is not None:
            indent = line[: len(line) - len(line.lstrip())]
            lines.append("%s%s: %s" % (indent, matched, remaining.pop(matched)))
        else:
            lines.append(line)
    for key, value in remaining.items():
        lines.append("%s%s: %s" % (indent_for_new, key, value))
    return lines


class RosBackend:
    node_hint = ""
    topic_prefix = ""

    def __init__(self, args):
        self.args = args

    def ensure_master(self):
        ...

    def write_params(self, dst):
        """Write a temp params file reflecting the test configuration."""
        raise NotImplementedError

    def launch_publisher(self):
        """Launch the publisher; return (popen, target_pid)."""
        raise NotImplementedError

    def probe_available(self):
        """True if the C++ stability_probe package is built and on the path."""
        raise NotImplementedError

    def probe_build_hint(self):
        raise NotImplementedError

    def launch_probe(self, topics, out_dir):
        """Launch the C++ probe; return (popen, target_pid)."""
        raise NotImplementedError

    def init_client(self):
        raise NotImplementedError

    def discover_sensor_topics(self, expected, timeout):
        raise NotImplementedError

    def subscribe(self, topic, on_message):
        """on_message(arrival_sec, stamp_sec, width)."""
        raise NotImplementedError

    def spin_background(self):
        ...

    def shutdown(self):
        ...


class Ros1Backend(RosBackend):
    node_hint = "nodelet"
    topic_prefix = "/cepton3/points_sn_"

    def __init__(self, args):
        super().__init__(args)
        self._rospy = None
        self._roscore = None
        self._subs = []

    def default_config(self):
        return REPO_ROOT / "ros" / "config" / "default_params.yaml"

    def ensure_master(self):
        import rosgraph
        if rosgraph.is_master_online():
            return
        print("roscore not running; starting one", flush=True)
        self._roscore = subprocess.Popen(["roscore"], start_new_session=True)
        deadline = time.time() + 15
        while time.time() < deadline:
            if rosgraph.is_master_online():
                return
            time.sleep(0.3)
        raise RuntimeError("failed to start roscore")

    # Point-inclusion keys the ROS1 driver understands (publisher_nodelet.cpp).
    ALL_POINTS_OVERRIDES = {
        "include_saturated_points": "true",
        "include_second_return_points": "true",
        "include_invalid_points": "true",
        "include_noise_points": "true",
        "include_blocked_points": "true",
        "include_retro_points": "true",
        "include_retro_weak_points": "true",
        "include_ambient_points": "true",
        "min_altitude": "-90.0",
        "max_altitude": "90.0",
        "min_azimuth": "-90.0",
        "max_azimuth": "90.0",
        "min_distance": "0.0",
        "max_distance": "1000.0",
    }

    def write_params(self, dst):
        src = Path(self.args.config_path) if self.args.config_path else self.default_config()
        overrides = {
            "aggregate_frames":
                "true" if self.args.aggregation_frame_count == 2 else "false",
        }
        if not self.args.no_all_points:
            # Pass every point through so per-frame width can be checked
            # against the SDK nominal (fixed-length frames).
            overrides.update(self.ALL_POINTS_OVERRIDES)
        lines = _override_yaml_keys(src.read_text().splitlines(), overrides)
        dst.write_text("\n".join(lines) + "\n")

    def launch_publisher(self):
        temp = Path(self.args.output_dir) / "params_ros1.yaml"
        self.write_params(temp)
        subprocess.check_call(["rosparam", "load", str(temp), "/cepton_publisher"])
        # Run the nodelet "standalone" rather than "load" into a separate
        # cepton_manager: standalone hosts its own manager in this one process,
        # so no external manager is needed and the publisher work runs in a
        # single process we own directly -- giving us one stable PID to monitor
        # for CPU/memory/liveness. (publisher.launch uses "load", which would
        # run the work in a separate cepton_manager process instead.)
        popen = subprocess.Popen(
            ["rosrun", "nodelet", "nodelet", "standalone",
             "cepton_ros/PublisherNodelet", "__name:=cepton_publisher"],
            start_new_session=True,
        )
        pid = resolve_target_pid(popen.pid, self.node_hint)
        return popen, pid

    def probe_available(self):
        try:
            return subprocess.run(
                ["rospack", "find", "stability_probe"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            ).returncode == 0
        except FileNotFoundError:
            return False

    def probe_build_hint(self):
        return (
            "stability_probe (ROS1) is not built. Build it with:\n"
            "  ln -s %s/tools/stability_probe_ros1 <catkin_ws>/src/stability_probe\n"
            "  cd <catkin_ws> && catkin_make && source devel/setup.bash\n"
            "or run with --rate-method inproc (low-rate dry runs only)."
            % REPO_ROOT
        )

    def launch_probe(self, topics, out_dir):
        popen = subprocess.Popen(
            ["rosrun", "stability_probe", "stability_probe_node",
             "__name:=stability_probe", "_output_dir:=%s" % out_dir]
            + list(topics),
            start_new_session=True,
        )
        pid = resolve_target_pid(popen.pid, "stability_probe")
        return popen, pid

    def init_client(self):
        import rospy
        self._rospy = rospy
        rospy.init_node("cepton_stability_test", anonymous=True, disable_signals=True)

    def discover_sensor_topics(self, expected, timeout):
        rospy = self._rospy
        deadline = time.time() + timeout
        found = set()
        while time.time() < deadline:
            for topic, _type in rospy.get_published_topics():
                if topic.startswith(self.topic_prefix):
                    found.add(topic)
            if len(found) >= expected:
                break
            time.sleep(0.5)
        return sorted(found)

    def subscribe(self, topic, on_message):
        rospy = self._rospy

        def cb(msg):
            arrival = time.time()
            try:
                stamp, width = parse_ros1_pc2_header(msg._buff)
            except (struct.error, IndexError):
                return
            on_message(arrival, stamp, width)

        self._subs.append(
            rospy.Subscriber(topic, rospy.AnyMsg, cb, queue_size=200,
                             buff_size=2 ** 26, tcp_nodelay=True)
        )

    def spin_background(self):
        # rospy delivers on background threads once subscribed; nothing to do.
        ...

    def shutdown(self):
        if self._rospy is not None:
            self._rospy.signal_shutdown("done")
        if self._roscore is not None:
            _stop_process(self._roscore, "roscore")


class Ros2Backend(RosBackend):
    node_hint = "cepton"
    topic_prefix = "/serial_"

    def __init__(self, args):
        super().__init__(args)
        self._rclpy = None
        self._node = None
        self._spin_thread = None
        self._pc2_type = None

    def default_config(self):
        return REPO_ROOT / "ros2" / "parameters.yaml"

    # Point-inclusion keys the ROS2 driver declares (cepton_publisher.cpp;
    # note there is no retro_weak key in ROS2). Doubles must stay doubles.
    ALL_POINTS_OVERRIDES = {
        "include_saturated_points": "true",
        "include_second_return_points": "true",
        "include_invalid_points": "true",
        "include_noise_points": "true",
        "include_blocked_points": "true",
        "include_retro_points": "true",
        "include_ambient_points": "true",
        "min_altitude": "-90.0",
        "max_altitude": "90.0",
        "min_azimuth": "-90.0",
        "max_azimuth": "90.0",
        "min_distance": "0.0",
        "max_distance": "1000.0",
    }

    def write_params(self, dst):
        src = Path(self.args.config_path) if self.args.config_path else self.default_config()
        # Drop expected-IP pinning so all sensors are discovered lazily.
        base = [line for line in src.read_text().splitlines()
                if not line.strip().startswith("expected_sensor_ips:")]
        overrides = {
            "aggregation_frame_count": str(self.args.aggregation_frame_count),
        }
        if not self.args.no_all_points:
            overrides.update(self.ALL_POINTS_OVERRIDES)
        # New keys must live under the ros__parameters block (4-space indent).
        lines = _override_yaml_keys(base, overrides, indent_for_new="    ")
        dst.write_text("\n".join(lines) + "\n")

    def launch_publisher(self):
        temp = Path(self.args.output_dir) / "params_ros2.yaml"
        self.write_params(temp)
        popen = subprocess.Popen(
            ["ros2", "run", "cepton_publisher", "cepton_publisher_node",
             "--ros-args", "--params-file", str(temp)],
            start_new_session=True,
        )
        pid = resolve_target_pid(popen.pid, self.node_hint)
        return popen, pid

    def probe_available(self):
        try:
            return subprocess.run(
                ["ros2", "pkg", "prefix", "stability_probe"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            ).returncode == 0
        except FileNotFoundError:
            return False

    def probe_build_hint(self):
        return (
            "stability_probe (ROS2) is not built. Build it with:\n"
            "  ln -s %s/tools/stability_probe_ros2 <colcon_ws>/src/stability_probe\n"
            "  cd <colcon_ws> && colcon build --packages-select stability_probe"
            " && source install/setup.bash\n"
            "or run with --rate-method inproc (low-rate dry runs only)."
            % REPO_ROOT
        )

    def launch_probe(self, topics, out_dir):
        popen = subprocess.Popen(
            ["ros2", "run", "stability_probe", "stability_probe", "--ros-args",
             "-p", "output_dir:=%s" % out_dir,
             "-p", "topics:=[%s]" % ",".join(topics)],
            start_new_session=True,
        )
        pid = resolve_target_pid(popen.pid, "stability_probe")
        return popen, pid

    def init_client(self):
        import rclpy
        from rclpy.node import Node
        self._rclpy = rclpy
        rclpy.init()
        self._node = Node("cepton_stability_test")

    def discover_sensor_topics(self, expected, timeout):
        deadline = time.time() + timeout
        found = set()
        while time.time() < deadline:
            for name, _types in self._node.get_topic_names_and_types():
                if name.startswith(self.topic_prefix):
                    found.add(name)
            if len(found) >= expected:
                break
            time.sleep(0.5)
        return sorted(found)

    def subscribe(self, topic, on_message):
        rclpy = self._rclpy

        def raw_cb(msg):
            arrival = time.time()
            if isinstance(msg, (bytes, bytearray, memoryview)):
                buf = bytes(msg)
            elif hasattr(msg, "buffer"):
                buf = bytes(msg.buffer)
            else:
                buf = bytes(msg)
            try:
                stamp, width = parse_ros2_pc2_header(buf)
            except (struct.error, IndexError):
                return
            on_message(arrival, stamp, width)

        from sensor_msgs.msg import PointCloud2
        try:
            self._node.create_subscription(PointCloud2, topic, raw_cb, 200, raw=True)
            return
        except TypeError:
            pass

        # Fallback: deserialize but read only stamp+width (no point iteration).
        def typed_cb(msg):
            arrival = time.time()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            on_message(arrival, stamp, msg.width)

        self._node.create_subscription(PointCloud2, topic, typed_cb, 200)

    def spin_background(self):
        rclpy = self._rclpy
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    def shutdown(self):
        if self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass


def make_backend(args):
    return Ros1Backend(args) if args.ros_version == 1 else Ros2Backend(args)


# --------------------------------------------------------------------------- #
# Evaluation
# --------------------------------------------------------------------------- #
def linreg(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0, (ys[0] if ys else 0.0)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    if denom == 0:
        return 0.0, sy / n
    slope = (n * sxy - sx * sy) / denom
    return slope, (sy - slope * sx) / n


def percentile(sorted_vals, q):
    """Nearest-rank percentile of an already sorted list (q in [0,100])."""
    if not sorted_vals:
        return None
    idx = int(round(q / 100.0 * (len(sorted_vals) - 1)))
    return sorted_vals[min(idx, len(sorted_vals) - 1)]


def _series_stats(vals):
    n = len(vals)
    mean = (sum(vals) / n) if n else None
    if n:
        variance = sum((v - mean) ** 2 for v in vals) / n
        std = variance ** 0.5
    else:
        std = None
    return {
        "min": min(vals) if vals else None,
        "max": max(vals) if vals else None,
        "mean": mean,
        "std": std,
    }


def _rate_and_drop(times, nominal_hz, inst_tol, win_tol, window, drop_factor):
    """Rate (instantaneous + sliding-window) and frame-drop from a time series.

    `times` may be arrival (wall-clock) or stamp (sensor) timestamps.
    - instantaneous: 1/dt per frame (rostopic hz -w 1 equivalent), judged
      against nominal +/- inst_tol
    - windowed: sliding window of at least `window` seconds; rate at frame i is
      (i-j)/(t[i]-t[j]) for the oldest j whose span stays >= window. Judged
      against nominal +/- win_tol. (No tumbling buckets: those quantize to
      +/-1 Hz at 1 s.)
    - drop: an interval longer than drop_factor * nominal period
    - dt_stats: distribution of the raw intervals, to judge whether the
      per-frame criterion is physically meaningful on this machine
    """
    period = 1.0 / nominal_hz

    dts = []
    inst = []
    drops = 0
    missing = 0
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            continue
        dts.append(dt)
        inst.append(1.0 / dt)
        if dt > drop_factor * period:
            drops += 1
            missing += max(1, round(dt / period) - 1)

    lo_i, hi_i = nominal_hz - inst_tol, nominal_hz + inst_tol
    inst_out = sum(1 for h in inst if h < lo_i or h > hi_i)
    inst_d = dict(_series_stats(inst), tolerance=inst_tol, lower=lo_i,
                  upper=hi_i, samples=len(inst), out_of_range=inst_out)
    inst_d["pass"] = bool(inst) and inst_out == 0

    win_rates = []
    j = 0
    for i in range(1, len(times)):
        while j < i - 1 and times[i] - times[j + 1] >= window:
            j += 1
        span = times[i] - times[j]
        if span >= window:
            win_rates.append((i - j) / span)
    lo_w, hi_w = nominal_hz - win_tol, nominal_hz + win_tol
    win_out = sum(1 for h in win_rates if h < lo_w or h > hi_w)
    win_d = dict(_series_stats(win_rates), tolerance=win_tol,
                 window_sec=window, lower=lo_w, upper=hi_w,
                 samples=len(win_rates), out_of_range=win_out)
    win_d["pass"] = bool(win_rates) and win_out == 0

    sdts = sorted(dts)
    dt_stats = {
        "p50_ms": _ms(percentile(sdts, 50)),
        "p99_ms": _ms(percentile(sdts, 99)),
        "p999_ms": _ms(percentile(sdts, 99.9)),
        "min_ms": _ms(sdts[0]) if sdts else None,
        "max_ms": _ms(sdts[-1]) if sdts else None,
        "out_of_inst_tolerance": inst_out,
    }

    rate = {
        "nominal_hz": nominal_hz,
        "instantaneous": inst_d,
        "windowed": win_d,
        "dt_stats": dt_stats,
        "pass": inst_d["pass"] and win_d["pass"],
    }
    drop = {
        "gap_events": drops, "estimated_missing_frames": missing,
        "threshold_sec": drop_factor * period, "pass": drops == 0,
    }
    return rate, drop


def _ms(v):
    return None if v is None else v * 1000.0


def evaluate_sensor(sd, nominal_hz, inst_tol, win_tol, window, warmup,
                    drop_factor, basis, expected_width):
    """Return a per-sensor result dict (rate/count/drop).

    Rate and frame-drop are computed on BOTH the arrival (wall-clock) and stamp
    (sensor time) series. `basis` selects which one is authoritative for
    pass/fail; the other is kept for reference (arrival reflects the cadence a
    ROS subscriber sees; stamp reflects the sensor's supply cadence).

    `expected_width` is the nominal per-message point count (SDK nominal x
    aggregation count) or None to require only that all frames are identical.
    """
    t0 = sd.arrivals[0] if sd.arrivals else 0.0
    # Keep frames received after the warmup window (filtered by arrival time).
    kept = [(a, s, w) for a, s, w in zip(sd.arrivals, sd.stamps, sd.widths)
            if a - t0 >= warmup]
    result = {"topic": sd.topic, "frames_total": len(sd.arrivals),
              "frames_evaluated": len(kept), "basis": basis}

    arr_times = [a for a, _s, _w in kept]
    stamp_times = [s for _a, s, _w in kept]
    rate_a, drop_a = _rate_and_drop(arr_times, nominal_hz, inst_tol, win_tol,
                                    window, drop_factor)
    rate_s, drop_s = _rate_and_drop(stamp_times, nominal_hz, inst_tol, win_tol,
                                    window, drop_factor)
    result["rate_arrival"] = rate_a
    result["rate_stamp"] = rate_s
    result["frame_drop_arrival"] = drop_a
    result["frame_drop_stamp"] = drop_s
    # Authoritative copies (used for the overall verdict and the report).
    result["rate"] = rate_s if basis == "stamp" else rate_a
    result["frame_drop"] = drop_s if basis == "stamp" else drop_a

    # Point count stability: identical every frame, and equal to the SDK
    # nominal when one is given (a shortfall = points lost somewhere).
    widths = [w for _a, _s, w in kept]
    uniq = sorted(set(widths))
    pc_pass = len(uniq) == 1
    if expected_width is not None:
        pc_pass = pc_pass and bool(widths) and widths[0] == expected_width
    result["point_count"] = {
        "unique_values": uniq[:20],
        "num_unique": len(uniq),
        "min": min(widths) if widths else None,
        "max": max(widths) if widths else None,
        "expected": expected_width,
        "pass": pc_pass,
    }
    return result


def evaluate_resources(samples, mem_thresh, cpu_thresh):
    ts = [s[0] for s in samples]
    rss = [s[1] for s in samples]
    cpu = [s[2] for s in samples if s[2] is not None]

    def growth(xs, ys, thresh_per_min):
        if len(ys) < 4:
            return {"pass": True, "slope_per_min": None, "note": "insufficient samples"}
        slope, _ = linreg(xs, ys)
        slope_min = slope * 60.0
        q = max(1, len(ys) // 4)
        first_q = sum(ys[:q]) / q
        last_q = sum(ys[-q:]) / q
        rising = last_q > first_q
        ok = not (slope_min > thresh_per_min and rising)
        return {
            "pass": ok, "slope_per_min": slope_min,
            "first_quartile_mean": first_q, "last_quartile_mean": last_q,
            "min": min(ys), "max": max(ys),
        }

    return {
        "memory_mb": growth(ts, rss, mem_thresh),
        "cpu_percent": growth([ts[i] for i in range(len(samples)) if samples[i][2] is not None],
                              cpu, cpu_thresh),
    }


# --------------------------------------------------------------------------- #
# Plotting
# --------------------------------------------------------------------------- #
def generate_plots(out_dir, sensors, monitor, nominal_hz, inst_tol, win_tol,
                   window, warmup, basis):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
    except ImportError:
        print("matplotlib not available; skipping graphs", file=sys.stderr, flush=True)
        return

    samples = monitor.samples if monitor is not None else []
    grid_kw = dict(color="0.85", linewidth=0.6)
    period = 1.0 / nominal_hz

    def basis_times(sd):
        return sd.arrivals if basis == "arrival" else sd.stamps

    def inst_series(times, t0):
        xs, ys = [], []
        for i in range(1, len(times)):
            dt = times[i] - times[i - 1]
            if dt > 0:
                xs.append(times[i] - t0)
                ys.append(1.0 / dt)
        return xs, ys

    def windowed_series(times, t0):
        xs, ys = [], []
        j = 0
        for i in range(1, len(times)):
            while j < i - 1 and times[i] - times[j + 1] >= window:
                j += 1
            span = times[i] - times[j]
            if span >= window:
                xs.append(times[i] - t0)
                ys.append((i - j) / span)
        return xs, ys

    # --- Publish rate over time (authoritative basis). Windowed mean is the
    # primary series (solid); the per-frame instantaneous rate is a faint
    # background layer. Y is clipped so the tolerance band stays readable. ---
    fig, ax = plt.subplots(figsize=(11, 5))
    ymax = nominal_hz * 2.0
    clipped = 0
    for idx, sd in enumerate(sorted(sensors.values(), key=lambda s: s.topic)):
        times = basis_times(sd)
        if len(times) < 2:
            continue
        t0 = times[0]
        color = SENSOR_COLORS[idx % len(SENSOR_COLORS)]
        label = sd.topic.rsplit("_", 1)[-1]
        ix, iy = inst_series(times, t0)
        wx, wy = windowed_series(times, t0)
        clipped += sum(1 for v in iy if v > ymax)
        ax.plot(ix, iy, linewidth=0.6, alpha=0.25, color=color)
        ax.plot(wx, wy, linewidth=1.2, color=color, label="SN %s" % label)
    ax.axhline(nominal_hz - win_tol, linestyle=":", color="#B00020",
               linewidth=1.2, label="lower bound %.2f Hz" % (nominal_hz - win_tol))
    ax.axhline(nominal_hz + win_tol, linestyle=":", color="#B00020",
               linewidth=1.2, label="upper bound %.2f Hz" % (nominal_hz + win_tol))
    if warmup > 0:
        ax.axvspan(0, warmup, color="0.9", label="warmup (excluded)")
    ax.set_ylim(0, ymax)
    style_handles = [
        Line2D([0], [0], color="0.4", linewidth=1.2,
               label="windowed mean (%.1fs)" % window),
        Line2D([0], [0], color="0.4", alpha=0.3, linewidth=0.6,
               label="instantaneous 1/dt"),
    ]
    handles = ax.get_legend_handles_labels()[0]
    ax.legend(handles=style_handles + handles, loc="best", fontsize=8,
              framealpha=0.9)
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel("Publish rate [Hz]")
    title = ("Point cloud publish rate (nominal %.1f Hz, win +/-%.2f, "
             "inst +/-%.2f, basis=%s)" % (nominal_hz, win_tol, inst_tol, basis))
    if clipped:
        title += "  [%d samples > %.0f Hz clipped]" % (clipped, ymax)
    ax.set_title(title)
    ax.grid(True, **grid_kw)
    fig.tight_layout()
    fig.savefig(out_dir / "framerate.png", dpi=120)
    plt.close(fig)

    # --- Interval jitter histogram (authoritative basis). Shows whether the
    # per-frame tolerance band is physically meaningful on this machine. ---
    fig, ax = plt.subplots(figsize=(11, 4.5))
    any_dt = False
    for idx, sd in enumerate(sorted(sensors.values(), key=lambda s: s.topic)):
        all_times = basis_times(sd)
        # Match the evaluation: drop the warmup window before binning.
        t0 = all_times[0] if all_times else 0.0
        times = [t for t in all_times if t - t0 >= warmup]
        dts = [(times[i] - times[i - 1]) * 1000.0
               for i in range(1, len(times)) if times[i] > times[i - 1]]
        if not dts:
            continue
        any_dt = True
        color = SENSOR_COLORS[idx % len(SENSOR_COLORS)]
        label = sd.topic.rsplit("_", 1)[-1]
        ax.hist(dts, bins=200, histtype="step", linewidth=1.0, color=color,
                label="SN %s" % label)
    lo_ms = 1000.0 / (nominal_hz + inst_tol)
    hi_ms = 1000.0 / (nominal_hz - inst_tol)
    ax.axvline(period * 1000.0, linestyle="-", color="0.6", linewidth=0.9,
               label="nominal %.2f ms" % (period * 1000.0))
    ax.axvline(lo_ms, linestyle=":", color="#B00020", linewidth=1.2,
               label="inst tolerance [%.2f, %.2f] ms" % (lo_ms, hi_ms))
    ax.axvline(hi_ms, linestyle=":", color="#B00020", linewidth=1.2)
    ax.set_yscale("log")
    ax.set_xlabel("Frame interval dt [ms]")
    ax.set_ylabel("Count (log)")
    ax.set_title("Interval jitter histogram (basis=%s)" % basis)
    ax.grid(True, **grid_kw)
    if any_dt:
        ax.legend(loc="best", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_dir / "jitter.png", dpi=120)
    plt.close(fig)

    # --- CPU over time. ---
    cpu_pts = [(t, c) for t, _r, c in samples if c is not None]
    fig, ax = plt.subplots(figsize=(11, 4))
    if cpu_pts:
        ax.plot([p[0] for p in cpu_pts], [p[1] for p in cpu_pts],
                linewidth=1.0, color=SENSOR_COLORS[0], label="CPU")
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel("CPU usage [%]")
    ax.set_title("Publisher CPU usage over time")
    ax.grid(True, **grid_kw)
    fig.tight_layout()
    fig.savefig(out_dir / "cpu.png", dpi=120)
    plt.close(fig)

    # --- Memory over time with regression trend. ---
    mem_pts = [(t, r) for t, r, _c in samples]
    fig, ax = plt.subplots(figsize=(11, 4))
    if mem_pts:
        xs = [p[0] for p in mem_pts]
        ys = [p[1] for p in mem_pts]
        ax.plot(xs, ys, linewidth=1.0, color=SENSOR_COLORS[2], label="RSS")
        slope, intercept = linreg(xs, ys)
        ax.plot(xs, [slope * x + intercept for x in xs], linestyle="--",
                color="#B00020", linewidth=1.2,
                label="trend (%.3f MB/min)" % (slope * 60.0))
        ax.legend(loc="best", fontsize=8)
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel("RSS memory [MB]")
    ax.set_title("Publisher memory (RSS) over time")
    ax.grid(True, **grid_kw)
    fig.tight_layout()
    fig.savefig(out_dir / "memory.png", dpi=120)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# CSV output
# --------------------------------------------------------------------------- #
def write_sensor_csvs(out_dir, sensors, nominal_hz):
    for sd in sensors.values():
        label = sd.topic.strip("/").replace("/", "_")
        path = out_dir / ("sensor_%s.csv" % label)
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["arrival_sec", "stamp_sec", "width", "inst_hz", "stamp_dt"])
            for i in range(len(sd.arrivals)):
                inst = ""
                sdt = ""
                if i > 0:
                    dt = sd.arrivals[i] - sd.arrivals[i - 1]
                    inst = 1.0 / dt if dt > 0 else ""
                    sdt = sd.stamps[i] - sd.stamps[i - 1]
                w.writerow([sd.arrivals[i], sd.stamps[i], sd.widths[i], inst, sdt])


def write_resource_csv(out_dir, monitor, filename="resource.csv"):
    path = out_dir / filename
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rel_sec", "rss_mb", "cpu_percent"])
        for t, rss, cpu in monitor.samples:
            w.writerow([t, rss, "" if cpu is None else cpu])


def read_probe_csvs(out_dir, topics):
    """Load the CSV files written by the C++ probe into SensorData buffers.

    The probe writes `sensor_<topic with / replaced by _>.csv` with columns
    arrival_sec (steady clock), stamp_sec, width — the same naming and column
    contract as write_sensor_csvs.
    """
    sensors = {}
    for topic in topics:
        label = topic.strip("/").replace("/", "_")
        path = out_dir / ("sensor_%s.csv" % label)
        sd = SensorData(topic)
        if path.exists():
            with path.open() as f:
                for row in csv.DictReader(f):
                    try:
                        sd.add(float(row["arrival_sec"]),
                               float(row["stamp_sec"]),
                               int(row["width"]))
                    except (KeyError, TypeError, ValueError):
                        continue  # tolerate a torn final line
        else:
            print("WARNING: probe output missing: %s" % path,
                  file=sys.stderr, flush=True)
        sensors[topic] = sd
    return sensors


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #
def print_report(summary):
    print("\n" + "=" * 76, flush=True)
    print("STABILITY TEST REPORT", flush=True)
    print("  ros_version            : %d" % summary["ros_version"])
    print("  aggregation_frame_count: %d" % summary["aggregation_frame_count"])
    print("  nominal_hz             : %.2f (inst ±%.2f, windowed %.1fs ±%.2f)" %
          (summary["nominal_hz"], summary["inst_tolerance"],
           summary["rate_window"], summary["rate_tolerance"]))
    print("  duration_sec           : %.1f" % summary["duration_sec"])
    print("  rate_method            : %s" % summary["rate_method"])
    print("  rate_basis (pass/fail) : %s  (the other basis is shown for reference)" %
          summary["rate_basis"])
    if summary.get("expected_points_total") is not None:
        print("  expected_points        : %d per message" %
              summary["expected_points_total"])
    print("  sensors                : %d" % len(summary["sensors"]))
    print("-" * 76, flush=True)
    basis = summary["rate_basis"]
    for s in summary["sensors"]:
        pc = s["point_count"]
        print("  %s  (frames=%d)" % (s["topic"], s["frames_evaluated"]))
        for b in ("arrival", "stamp"):
            r = s["rate_%s" % b]
            fd = s["frame_drop_%s" % b]
            inst, win, dts = r["instantaneous"], r["windowed"], r["dt_stats"]
            star = "*" if b == basis else " "
            print("   %s %-7s inst : %s  (out=%d/%d min=%s max=%s mean=%s std=%s)" % (
                star, b, _pf(inst["pass"]), inst["out_of_range"],
                inst["samples"], _fmt(inst["min"]), _fmt(inst["max"]),
                _fmt(inst["mean"]), _fmt(inst["std"])))
            print("   %s %-7s win  : %s  (out=%d/%d min=%s max=%s mean=%s std=%s)" % (
                star, b, _pf(win["pass"]), win["out_of_range"], win["samples"],
                _fmt(win["min"]), _fmt(win["max"]), _fmt(win["mean"]),
                _fmt(win["std"])))
            print("   %s %-7s drop : %s  (gaps=%d missing≈%d)" % (
                star, b, _pf(fd["pass"]), fd["gap_events"],
                fd["estimated_missing_frames"]))
            print("     %-7s dt   : p50=%sms p99=%sms p99.9=%sms max=%sms" % (
                b, _fmt(dts["p50_ms"]), _fmt(dts["p99_ms"]),
                _fmt(dts["p999_ms"]), _fmt(dts["max_ms"])))
        exp = pc.get("expected")
        detail = "unique=%d min=%s max=%s" % (
            pc["num_unique"], _fmt(pc["min"]), _fmt(pc["max"]))
        if exp is not None:
            detail += " expected=%d" % exp
        print("     points       : %s  (%s)" % (_pf(pc["pass"]), detail))
    print("  (* = authoritative basis for pass/fail)", flush=True)
    print("-" * 76, flush=True)
    proc = summary["process"]
    print("  process_alive : %s  %s" % (
        _pf(proc["pass"]), "" if proc["pass"] else "(crashed at %.1fs)" % proc["crash_time"]))
    mem = summary["resources"]["memory_mb"]
    cpu = summary["resources"]["cpu_percent"]
    print("  memory        : %s  (slope=%s MB/min, min=%s max=%s)" % (
        _pf(mem["pass"]), _fmt(mem.get("slope_per_min")),
        _fmt(mem.get("min")), _fmt(mem.get("max"))))
    print("  cpu           : %s  (slope=%s %%/min, max=%s%%)" % (
        _pf(cpu["pass"]), _fmt(cpu.get("slope_per_min")), _fmt(cpu.get("max"))))
    probe = summary.get("probe")
    if probe:
        print("  probe (info)  : cpu mean=%s%% max=%s%%, rss max=%sMB%s" % (
            _fmt(probe.get("cpu_mean")), _fmt(probe.get("cpu_max")),
            _fmt(probe.get("rss_max_mb")),
            "  ** CRASHED **" if probe.get("crashed") else ""))
    print("=" * 76, flush=True)
    print("  OVERALL: %s" % _pf(summary["overall_pass"]), flush=True)
    print("=" * 76 + "\n", flush=True)


def _pf(ok):
    return "PASS" if ok else "FAIL"


def _fmt(v):
    return "n/a" if v is None else ("%.2f" % v)


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args():
    default_version = os.environ.get("ROS_VERSION", "")
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, required=True,
                   help="Measurement duration in seconds")
    p.add_argument("--aggregation-frame-count", type=int, choices=[1, 2], default=1,
                   help="1 (~20Hz) or 2 (~10Hz). ROS1 maps this to aggregate_frames.")
    p.add_argument("--ros-version", type=int, choices=[1, 2],
                   default=(int(default_version) if default_version in ("1", "2") else None),
                   help="ROS version (default: $ROS_VERSION)")
    p.add_argument("--expected-sensors", type=int, default=4,
                   help="Number of sensor topics that must appear (default: 4)")
    p.add_argument("--rate-method", choices=["probe", "inproc"], default="probe",
                   help="Data-plane measurement: 'probe' launches the C++ "
                        "stability_probe node (required for real sensors; "
                        "point cloud traffic exceeds what a Python subscriber "
                        "can absorb) or 'inproc' subscribes from this Python "
                        "process (low-rate dry runs only). (default: probe)")
    p.add_argument("--rate-tolerance", type=float, default=0.1,
                   help="Allowed Hz deviation of the sliding-window mean rate "
                        "(default: 0.1)")
    p.add_argument("--inst-tolerance", type=float, default=0.1,
                   help="Allowed Hz deviation of the per-frame instantaneous "
                        "rate 1/dt (default: 0.1 = the spec as written; at "
                        "20Hz this is a +/-0.25ms interval budget)")
    p.add_argument("--rate-window", type=float, default=1.0,
                   help="Sliding window length in seconds for the windowed "
                        "mean rate (default: 1.0)")
    p.add_argument("--rate-basis", choices=["arrival", "stamp"], default="arrival",
                   help="Which timestamp drives rate/drop pass-fail: 'arrival' "
                        "(subscriber wall-clock, the cadence a ROS client sees) or "
                        "'stamp' (sensor supply cadence, reference). Both are "
                        "always reported. (default: arrival)")
    p.add_argument("--expected-points", type=int, default=349960,
                   help="Nominal SDK points per frame; pass requires width == "
                        "this x aggregation_frame_count on every frame. 0 "
                        "requires only that all frames are identical "
                        "(dry runs). (default: 349960)")
    p.add_argument("--no-all-points", action="store_true",
                   help="Do not override the driver config to pass all points "
                        "through. Default is to enable all include_* flags and "
                        "open the distance/angle filters so the per-frame "
                        "width can be checked against the SDK nominal.")
    p.add_argument("--warmup", type=float, default=5.0,
                   help="Seconds excluded from evaluation after start (default: 5.0)")
    p.add_argument("--drop-factor", type=float, default=1.5,
                   help="Interval > factor*period counts as a frame drop (default: 1.5)")
    p.add_argument("--mem-growth-threshold", type=float, default=1.0,
                   help="RSS growth fail threshold in MB/min (default: 1.0)")
    p.add_argument("--cpu-growth-threshold", type=float, default=5.0,
                   help="CPU growth fail threshold in %%/min (default: 5.0)")
    p.add_argument("--resource-interval", type=float, default=1.0,
                   help="Resource sampling interval in seconds (default: 1.0)")
    p.add_argument("--startup-timeout", type=float, default=30.0,
                   help="Seconds to wait for sensor topics to appear (default: 30)")
    p.add_argument("--no-launch", action="store_true",
                   help="Do not launch the publisher (attach mode / dry-run). "
                        "Use --attach-pid to still monitor an existing process.")
    p.add_argument("--attach-pid", type=int, default=None,
                   help="With --no-launch, monitor this already-running pid")
    p.add_argument("--config-path", default=None,
                   help="Base params YAML (default: version-specific driver default)")
    p.add_argument("--output-dir", default=None,
                   help="Output directory (default: scripts/stability_output/<ts>)")
    return p.parse_args()


def main():
    args = parse_args()
    if args.ros_version is None:
        print("--ros-version is required (or source a ROS environment so "
              "$ROS_VERSION is set)", file=sys.stderr)
        return 2

    if args.output_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output_dir = str(SCRIPT_DIR / "stability_output" / stamp)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    nominal_hz = 20.0 / args.aggregation_frame_count
    backend = make_backend(args)

    popen = None
    monitor = None
    probe_popen = None
    probe_monitor = None
    sensors = {}
    topics = []
    crashed_during_run = False
    probe_crashed = False
    try:
        backend.ensure_master()
        if args.no_launch:
            print("--no-launch: attaching to an externally started publisher",
                  flush=True)
            pid = args.attach_pid
            if pid is not None:
                monitor = ResourceMonitor(pid, _DummyPopen(), args.resource_interval)
                monitor.start()
        else:
            print("launching publisher (ros%d, aggregation_frame_count=%d)" %
                  (args.ros_version, args.aggregation_frame_count), flush=True)
            popen, pid = backend.launch_publisher()
            print("publisher launched; monitoring pid %d" % pid, flush=True)
            monitor = ResourceMonitor(pid, popen, args.resource_interval)
            monitor.start()

        backend.init_client()
        topics = backend.discover_sensor_topics(args.expected_sensors, args.startup_timeout)
        if len(topics) < args.expected_sensors:
            print("ERROR: found %d/%d sensor topics: %s" %
                  (len(topics), args.expected_sensors, topics), file=sys.stderr)
            return 2

        if args.rate_method == "probe":
            if not backend.probe_available():
                print("ERROR: %s" % backend.probe_build_hint(), file=sys.stderr)
                return 2
            print("launching C++ probe for %d sensor topics" % len(topics),
                  flush=True)
            probe_popen, probe_pid = backend.launch_probe(topics, str(out_dir))
            print("probe launched; monitoring pid %d" % probe_pid, flush=True)
            probe_monitor = ResourceMonitor(probe_pid, probe_popen,
                                            args.resource_interval)
            probe_monitor.start()
        else:
            print("subscribing to %d sensor topics (inproc)" % len(topics),
                  flush=True)
            for topic in topics:
                sd = SensorData(topic)
                sensors[topic] = sd
                backend.subscribe(topic, lambda a, s, w, _sd=sd: _sd.add(a, s, w))
            backend.spin_background()

        # Run for the requested duration, bailing early on a crash.
        deadline = time.time() + args.duration
        while time.time() < deadline:
            if monitor is not None and monitor.crashed:
                crashed_during_run = True
                print("ERROR: publisher process exited during the run", file=sys.stderr)
                break
            if probe_monitor is not None and probe_monitor.crashed:
                probe_crashed = True
                print("ERROR: probe process exited during the run; "
                      "measurement is invalid", file=sys.stderr)
                break
            time.sleep(0.5)
    finally:
        if monitor is not None:
            monitor.stop()
            monitor.join(timeout=5)
        if probe_monitor is not None:
            probe_monitor.stop()
            probe_monitor.join(timeout=5)
        # Stop the probe first so its CSV files are flushed and closed.
        _stop_process(probe_popen, "probe")
        backend.shutdown()
        _stop_process(popen, "publisher")

    if args.rate_method == "probe":
        sensors = read_probe_csvs(out_dir, topics)

    # ---- Evaluate ----
    expected_total = (args.expected_points * args.aggregation_frame_count
                      if args.expected_points > 0 else None)
    sensor_results = [
        evaluate_sensor(sd, nominal_hz, args.inst_tolerance,
                        args.rate_tolerance, args.rate_window, args.warmup,
                        args.drop_factor, args.rate_basis, expected_total)
        for sd in sorted(sensors.values(), key=lambda s: s.topic)
    ]
    res = evaluate_resources(monitor.samples if monitor else [],
                             args.mem_growth_threshold, args.cpu_growth_threshold)
    proc_ok = not (monitor and monitor.crashed) and not crashed_during_run
    process_result = {
        "pass": proc_ok,
        "crash_time": (monitor.crash_time if monitor else None),
    }

    probe_info = None
    if probe_monitor is not None:
        p_cpu = [c for _t, _r, c in probe_monitor.samples if c is not None]
        p_rss = [r for _t, r, _c in probe_monitor.samples]
        probe_info = {
            "cpu_mean": (sum(p_cpu) / len(p_cpu)) if p_cpu else None,
            "cpu_max": max(p_cpu) if p_cpu else None,
            "rss_max_mb": max(p_rss) if p_rss else None,
            "crashed": bool(probe_monitor.crashed or probe_crashed),
        }

    overall = (
        bool(sensor_results)
        and all(s["rate"]["pass"] and s["point_count"]["pass"] and s["frame_drop"]["pass"]
                for s in sensor_results)
        and process_result["pass"]
        and res["memory_mb"]["pass"] and res["cpu_percent"]["pass"]
        and not probe_crashed
    )

    summary = {
        "ros_version": args.ros_version,
        "aggregation_frame_count": args.aggregation_frame_count,
        "nominal_hz": nominal_hz,
        "rate_method": args.rate_method,
        "rate_tolerance": args.rate_tolerance,
        "inst_tolerance": args.inst_tolerance,
        "rate_window": args.rate_window,
        "rate_basis": args.rate_basis,
        "expected_points_total": expected_total,
        "duration_sec": args.duration,
        "sensors": sensor_results,
        "process": process_result,
        "resources": res,
        "probe": probe_info,
        "overall_pass": overall,
    }

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    write_sensor_csvs(out_dir, sensors, nominal_hz)
    if monitor is not None:
        write_resource_csv(out_dir, monitor)
    if probe_monitor is not None:
        write_resource_csv(out_dir, probe_monitor, "resource_probe.csv")
    generate_plots(out_dir, sensors, monitor, nominal_hz, args.inst_tolerance,
                   args.rate_tolerance, args.rate_window, args.warmup,
                   args.rate_basis)
    print_report(summary)
    print("Output written to %s" % out_dir, flush=True)
    if probe_crashed:
        return 2
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
