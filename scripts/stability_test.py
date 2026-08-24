#!/usr/bin/env python3
"""Long-duration stability test for the Cepton ROS1/ROS2 point cloud publisher.

Launches the publisher against live sensors, subscribes to the per-sensor point
cloud topics and to the sensor info topic, and evaluates:
  - publish rate (instantaneous, rostopic-hz-`-w 1`-equivalent) within tolerance
  - per-frame point count stability (every frame identical)
  - frame drops (gaps in the observed publish cadence)
  - sensor info publish rate per sensor (nominal 2 Hz +/- 0.5)
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
from collections import Counter, defaultdict
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
# System-wide and per-process sampling
# --------------------------------------------------------------------------- #
def _cpu_jiffies():
    """(total, idle) jiffies for the whole machine and for each core."""
    out = {}
    try:
        with open("/proc/stat") as f:
            for line in f:
                if not line.startswith("cpu"):
                    break
                fields = line.split()
                if fields[0] != "cpu" and not fields[0][3:].isdigit():
                    continue
                v = [int(x) for x in fields[1:9]]
                out[fields[0]] = (sum(v), v[3] + v[4])  # total, idle + iowait
    except (OSError, ValueError, IndexError):
        pass
    return out


def _udp_counters():
    """(InDatagrams, RcvbufErrors) from the second Udp: line of /proc/net/snmp."""
    try:
        with open("/proc/net/snmp") as f:
            seen = 0
            for line in f:
                if line.startswith("Udp:"):
                    seen += 1
                    if seen == 2:
                        fields = line.split()
                        return int(fields[1]), int(fields[5])
    except (OSError, ValueError, IndexError):
        pass
    return None, None


def _net_rx_softirq():
    try:
        with open("/proc/softirqs") as f:
            for line in f:
                if line.strip().startswith("NET_RX"):
                    return sum(int(x) for x in line.split()[1:])
    except (OSError, ValueError):
        pass
    return None


def _top_processes(prev, interval, count=3):
    """(comm, cpu_percent) of the processes that burned the most CPU.

    Also returns the fresh table so the caller can diff against it next time.
    """
    cur = {}
    try:
        entries = os.listdir("/proc")
    except OSError:
        return [], cur
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open("/proc/%s/stat" % entry) as f:
                data = f.read()
            rp = data.rfind(")")
            rest = data[rp + 2:].split()
            cur[int(entry)] = (data[data.find("(") + 1:rp],
                               int(rest[11]) + int(rest[12]))
        except (OSError, ValueError, IndexError):
            continue
    hz = os.sysconf("SC_CLK_TCK")
    deltas = [((cur[k][1] - prev[k][1]), cur[k][0]) for k in cur if k in prev]
    deltas.sort(reverse=True)
    top = [(name, 100.0 * d / hz / interval) for d, name in deltas[:count] if d > 0]
    return top, cur


def perf_available():
    """(usable, reason) for counting hardware events on this kernel."""
    if _run(["perf", "--version"]) is None:
        return False, "perf not installed"
    probe = None
    try:
        probe = subprocess.run(["perf", "stat", "-e", "cycles", "-x,", "--", "true"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                               timeout=10)
    except (OSError, subprocess.SubprocessError) as exc:
        return False, "perf failed to run (%s)" % exc
    if probe.returncode == 0:
        return True, None
    paranoid = _read_text("/proc/sys/kernel/perf_event_paranoid")
    return False, ("perf_event_paranoid=%s blocks unprivileged counting; "
                   "lower it or run as root for a process-attributed clock"
                   % paranoid)


class PerfCounter(threading.Thread):
    """Effective clock of the process under test: cycles / task-clock.

    This is the only frequency figure that can be attributed to the process:
    `cycles` counts what the process actually burned and `task-clock` the time
    it was on a CPU, so the ratio survives threads migrating between cores or
    being created per message. Needs CAP_PERFMON or a permissive
    perf_event_paranoid; when that is unavailable nothing is recorded rather
    than substituting a number that cannot be attributed.
    """

    def __init__(self, pid, interval):
        super().__init__(daemon=True)
        self.pid = pid
        self.interval = max(0.1, interval)
        self.samples = []
        self.unavailable_reason = None
        self._proc = None

    def start_if_possible(self):
        ok, reason = perf_available()
        if not ok:
            self.unavailable_reason = reason
            return False
        try:
            self._proc = subprocess.Popen(
                ["perf", "stat", "-p", str(self.pid), "-e", "cycles,task-clock",
                 "-x,", "-I", str(int(self.interval * 1000))],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        except (OSError, subprocess.SubprocessError) as exc:
            self.unavailable_reason = "perf could not be started (%s)" % exc
            return False
        self.start()
        return True

    def stop(self):
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()

    def run(self):
        # `perf stat -x, -I` emits one CSV line per event per interval:
        #   <elapsed>,<count>,<unit>,<event>,...
        pending = {}
        for raw in self._proc.stderr:
            fields = raw.decode("utf-8", "replace").strip().split(",")
            if len(fields) < 4:
                continue
            try:
                t_rel = float(fields[0])
                count = float(fields[1])
            except ValueError:
                continue
            event = fields[3]
            slot = pending.setdefault(round(t_rel, 3), {})
            slot[event] = count
            if "cycles" in slot and "task-clock" in slot:
                busy_sec = slot["task-clock"] / 1000.0  # perf reports msec
                self.samples.append({
                    "t_rel_sec": t_rel,
                    "effective_ghz": (slot["cycles"] / busy_sec / 1e9
                                      if busy_sec > 0 else None),
                    "busy_ratio": busy_sec / self.interval,
                })
                pending.pop(round(t_rel, 3), None)


class SystemMonitor(threading.Thread):
    """Whole-machine sampling: everything the per-process monitor cannot see.

    The publisher's own CPU time says nothing about a machine that is
    descheduling it, dropping its packets in the kernel, or clocking its core
    down, so those are recorded alongside. Everything here is a read of /proc
    or /sys; the process under test is never touched.
    """

    def __init__(self, interval, interface):
        super().__init__(daemon=True)
        self.interval = interval
        self.interface = interface
        self.samples = []  # dicts, see SYSTEM_CSV_COLUMNS
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def _nic(self, stat):
        if not self.interface:
            return None
        return _read_int("/sys/class/net/%s/statistics/%s" % (self.interface, stat))

    def run(self):
        t0 = time.time()
        prev_cpu = _cpu_jiffies()
        prev_udp = _udp_counters()
        prev_sirq = _net_rx_softirq()
        prev_rx = self._nic("rx_bytes")
        prev_drop = self._nic("rx_dropped")
        prev_procs = {}
        _top, prev_procs = _top_processes(prev_procs, self.interval)
        while not self._stop_event.wait(self.interval):
            now = time.time()
            cpu = _cpu_jiffies()
            udp = _udp_counters()
            sirq = _net_rx_softirq()
            rx = self._nic("rx_bytes")
            drop = self._nic("rx_dropped")
            top, prev_procs = _top_processes(prev_procs, self.interval)

            def busy(key):
                if key not in cpu or key not in prev_cpu:
                    return None
                d = cpu[key][0] - prev_cpu[key][0]
                return 100.0 * (1 - (cpu[key][1] - prev_cpu[key][1]) / d) if d else None

            per_core = [busy(k) for k in cpu if k != "cpu"]
            per_core = [v for v in per_core if v is not None]
            # Machine context only: which clocks the cores were reporting. No
            # claim is made about which of them ran the process under test.
            freqs = sorted(v for v in (_core_freq_mhz(i)
                                       for i in range(os.cpu_count() or 0))
                           if v is not None)

            def rate(now_v, prev_v):
                if now_v is None or prev_v is None:
                    return None
                return (now_v - prev_v) / self.interval

            self.samples.append({
                "t_rel_sec": now - t0,
                "cpu_busy_pct": busy("cpu"),
                "cpu_max_core_pct": max(per_core) if per_core else None,
                "load1": _read_text("/proc/loadavg", "").split()[0] or None,
                "temp_c": max_thermal_c(),
                "net_rx_softirq_s": rate(sirq, prev_sirq),
                "udp_in_s": rate(udp[0], prev_udp[0]),
                "udp_rcvbuf_err_s": rate(udp[1], prev_udp[1]),
                "nic_rx_mbps": ((rate(rx, prev_rx) * 8 / 1e6)
                                if None not in (rx, prev_rx) else None),
                "nic_drop_s": rate(drop, prev_drop),
                "freq_min_mhz": freqs[0] if freqs else None,
                "freq_median_mhz": freqs[len(freqs) // 2] if freqs else None,
                "freq_max_mhz": freqs[-1] if freqs else None,
                "top_processes": " ".join("%s:%.0f%%" % (n, c) for n, c in top),
            })
            prev_cpu, prev_udp, prev_sirq, prev_rx, prev_drop = cpu, udp, sirq, rx, drop


def max_thermal_c():
    best = None
    try:
        names = os.listdir("/sys/class/thermal")
    except OSError:
        return None
    for name in names:
        if not name.startswith("thermal_zone"):
            continue
        v = _read_int("/sys/class/thermal/%s/temp" % name)
        if v is not None:
            c = v / 1000.0
            best = c if best is None else max(best, c)
    return best


def _core_freq_mhz(cpu):
    v = _read_int("/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq" % cpu)
    return v / 1000.0 if v else None


class ProcessMonitor(threading.Thread):
    """Internals of the process under test.

    Deliberately records no per-core clock. `processor` in /proc/<pid>/task/*/stat
    is the last core the thread was seen on at read time, not where it spent the
    interval, and this driver spawns a fresh publish thread per message, so any
    frequency paired with it would be unattributable. Machine-wide clock context
    lives in SystemMonitor, and the process-attributed effective clock (the only
    honest one) comes from PerfCounter when the kernel permits it.
    """

    def __init__(self, pid, interval):
        super().__init__(daemon=True)
        self.pid = pid
        self.interval = interval
        self.samples = []
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def _threads(self):
        """{tid: (cpu, ticks, voluntary_cs, involuntary_cs)} for every thread."""
        out = {}
        try:
            tids = os.listdir("/proc/%d/task" % self.pid)
        except OSError:
            return out
        for tid in tids:
            try:
                with open("/proc/%d/task/%s/stat" % (self.pid, tid)) as f:
                    data = f.read()
                r = data[data.rfind(")") + 2:].split()
                out[int(tid)] = (int(r[36]), int(r[11]) + int(r[12]),
                                 int(r[7]), int(r[8]))  # cpu, ticks, minflt-ish
            except (OSError, ValueError, IndexError):
                continue
        return out

    def _vma_count(self):
        try:
            with open("/proc/%d/maps" % self.pid) as f:
                return sum(1 for _ in f)
        except OSError:
            return None

    def run(self):
        t0 = time.time()
        hz = os.sysconf("SC_CLK_TCK")
        prev = self._threads()
        prev_minflt = self._minflt()
        while not self._stop_event.wait(self.interval):
            now = time.time()
            cur = self._threads()
            if not cur:
                break
            hot_tid, hot_delta = None, -1
            for tid, (_cpu, ticks, _a, _b) in cur.items():
                d = ticks - prev.get(tid, (0, ticks, 0, 0))[1]
                if d > hot_delta:
                    hot_tid, hot_delta = tid, d
            hot_cpu = cur[hot_tid][0] if hot_tid is not None else None
            minflt = self._minflt()
            self.samples.append({
                "t_rel_sec": now - t0,
                "threads": len(cur),
                "vmrss_mb": read_rss_mb(self.pid),
                "vmsize_mb": self._vmsize_mb(),
                "vmas": self._vma_count(),
                "minflt_s": ((minflt - prev_minflt) / self.interval
                             if minflt is not None and prev_minflt is not None else None),
                "hot_thread_last_cpu": hot_cpu,
                "hot_thread_cpu_pct": (100.0 * hot_delta / hz / self.interval
                                       if hot_delta > 0 else 0.0),
            })
            prev, prev_minflt = cur, minflt

    def _minflt(self):
        try:
            with open("/proc/%d/stat" % self.pid) as f:
                data = f.read()
            return int(data[data.rfind(")") + 2:].split()[7])
        except (OSError, ValueError, IndexError):
            return None

    def _vmsize_mb(self):
        try:
            with open("/proc/%d/status" % self.pid) as f:
                for line in f:
                    if line.startswith("VmSize:"):
                        return int(line.split()[1]) / 1024.0
        except (OSError, ValueError, IndexError):
            pass
        return None


# --------------------------------------------------------------------------- #
# Environment snapshot
# --------------------------------------------------------------------------- #
def _read_text(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def _read_int(path):
    v = _read_text(path)
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _sysctl(name):
    """Read a sysctl through /proc/sys so no external binary is needed."""
    v = _read_text("/proc/sys/" + name.replace(".", "/"))
    if v is None:
        return None
    parts = v.split()
    if len(parts) == 1:
        try:
            return int(parts[0])
        except ValueError:
            return parts[0]
    return v


def _run(cmd, timeout=5.0):
    """Best-effort external command; None when it is unavailable or fails."""
    try:
        out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.decode("utf-8", "replace") if out.returncode == 0 else None


def socket_buffers(pid):
    """(local_addr, rcv_buf, snd_buf) of the process's UDP sockets, via `ss`.

    The kernel applies net.core.{r,w}mem_default when the socket is created, so
    these are the values the DDS transport actually got -- which is what
    matters, not what the sysctl says now.
    """
    text = _run(["ss", "-ulmnp"])
    if text is None:
        return None
    want = "pid=%d," % pid
    socks = []
    line = ""
    for raw in text.splitlines():
        s = raw.strip()
        if s.startswith("skmem:"):
            if want in line or (",%d," % pid) in line:
                fields = {}
                for part in s[len("skmem:("):].rstrip(")").split(","):
                    for key in ("rb", "tb"):
                        if part.startswith(key):
                            fields[key] = int(part[len(key):])
                # ss columns: State Recv-Q Send-Q Local:Port Peer:Port Process
                cols = line.split()
                socks.append({"local": cols[3] if len(cols) > 3 else "?",
                              "rcv_buf": fields.get("rb"),
                              "snd_buf": fields.get("tb")})
        else:
            line = raw
    return socks


def shm_segments():
    """Fast DDS shared-memory segments and their sizes (bytes)."""
    out = {}
    try:
        for name in os.listdir("/dev/shm"):
            if name.startswith("fastrtps") and not name.endswith("_el"):
                try:
                    out[name] = os.path.getsize("/dev/shm/" + name)
                except OSError:
                    pass
    except OSError:
        return None
    return out


def busiest_interface(sample_sec=1.0):
    """Interface receiving the most bytes right now (the sensor link)."""
    def rx():
        out = {}
        try:
            names = os.listdir("/sys/class/net")
        except OSError:
            return out
        for n in names:
            if n == "lo":
                continue
            v = _read_int("/sys/class/net/%s/statistics/rx_bytes" % n)
            if v is not None:
                out[n] = v
        return out
    a = rx()
    time.sleep(sample_sec)
    b = rx()
    deltas = {n: b[n] - a[n] for n in b if n in a}
    if not deltas:
        return None
    best = max(deltas, key=deltas.get)
    return best if deltas[best] > 0 else None


def collect_environment(pid, interface, message_bytes=None):
    """One-shot machine/driver/DDS profile, recorded next to the measurements.

    Written before the run so that a result can always be traced back to the
    configuration that produced it -- kernel socket buffers in particular,
    which are invisible in the driver's own parameters yet decide whether a
    multi-megabyte message can leave the publisher without stalling.
    """
    cpu0 = "/sys/devices/system/cpu/cpu0/cpufreq/"
    pstate = "/sys/devices/system/cpu/intel_pstate/"
    env = {
        "host": {
            "hostname": _read_text("/proc/sys/kernel/hostname"),
            "kernel": _read_text("/proc/sys/kernel/osrelease"),
            "cpu_model": next((line.split(":", 1)[1].strip()
                               for line in (_read_text("/proc/cpuinfo") or "").splitlines()
                               if line.startswith("model name")), None),
            "cpu_count": os.cpu_count(),
        },
        "cpu_scaling": {
            "governor": _read_text(cpu0 + "scaling_governor"),
            "driver": _read_text(cpu0 + "scaling_driver"),
            "energy_performance_preference": _read_text(cpu0 + "energy_performance_preference"),
            "intel_pstate_status": _read_text(pstate + "status"),
            "intel_pstate_no_turbo": _read_int(pstate + "no_turbo"),
            "intel_pstate_min_perf_pct": _read_int(pstate + "min_perf_pct"),
            "intel_pstate_max_perf_pct": _read_int(pstate + "max_perf_pct"),
        },
        "sysctl": {k: _sysctl(k) for k in (
            "net.core.rmem_max", "net.core.rmem_default",
            "net.core.wmem_max", "net.core.wmem_default",
            "net.core.optmem_max", "net.core.netdev_max_backlog",
            "net.ipv4.udp_mem", "net.ipv4.udp_rmem_min")},
        "ros": {k: os.environ.get(k) for k in (
            "ROS_DISTRO", "ROS_VERSION", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID",
            "ROS_LOCALHOST_ONLY", "FASTRTPS_DEFAULT_PROFILES_FILE",
            "FASTDDS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI")},
        "dds_shm_segments": shm_segments(),
        "sensor_interface": interface,
        "publisher_pid": pid,
        "publisher_sockets": socket_buffers(pid) if pid else None,
        "driver_revision": (_run(["git", "-C", str(REPO_ROOT), "describe",
                                  "--tags", "--always", "--dirty"]) or "").strip() or None,
    }
    if interface:
        env["nic_ring"] = (_run(["ethtool", "-g", interface]) or "").strip() or None

    # Smallest send buffer the publisher's sockets got, against one message.
    snd = [s["snd_buf"] for s in (env["publisher_sockets"] or []) if s.get("snd_buf")]
    if snd:
        env["min_send_buffer_bytes"] = min(snd)
        if message_bytes:
            env["message_bytes"] = message_bytes
            env["message_per_send_buffer"] = message_bytes / float(min(snd))
    return env


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


class SensorInfoData:
    """Sensor info messages received for one serial number."""

    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.arrivals = []  # wall-clock receive time (float sec)
        self.records = []   # decoded message fields (sensor_info_record)

    def add(self, arrival, record):
        self.arrivals.append(arrival)
        self.records.append(record)


# Integer fields carried by both cepton_ros/SensorInformation (ROS1) and
# cepton_messages/CeptonSensorInfo (ROS2).
INFO_INT_FIELDS = (
    "serial_number", "handle", "model", "part_number", "firmware_version",
    "power_up_timestamp", "time_sync_offset", "time_sync_drift",
    "return_count", "channel_count", "status_flags", "temperature",
    "fault_summary",
)

# CeptonSensor.status_flags / fault_summary bit names (cepton_sdk3.h).
STATUS_FLAG_BITS = ((1 << 0, "PTP_CONNECTED"), (1 << 1, "PPS_CONNECTED"),
                    (1 << 2, "NMEA_CONNECTED"))
FAULT_SUMMARY_BITS = ((1 << 0, "DATA_RATIONALITY"), (1 << 1, "DATA_CHECKSUM"),
                      (1 << 2, "TEMPERATURE_RANGE"), (1 << 3, "VOLTAGE_RANGE"))


def decode_bits(value, table):
    """Bit names set in `value`, with any leftover bits shown as hex."""
    names = [name for bit, name in table if value & bit]
    unknown = value & ~sum(bit for bit, _name in table)
    if unknown:
        names.append("0x%X" % unknown)
    return names


def _as_byte_list(value):
    """uint8[32] arrives as bytes (rospy) or an array/ndarray (rclpy)."""
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, str):
        return list(value.encode("latin-1"))
    try:
        return [int(v) for v in value]
    except TypeError:
        return []


def _header_stamp_sec(header):
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    if hasattr(stamp, "to_sec"):
        return stamp.to_sec()  # rospy.Time
    return stamp.sec + stamp.nanosec * 1e-9  # builtin_interfaces/Time


def sensor_info_record(msg):
    """Flatten a SensorInformation / CeptonSensorInfo message into a dict.

    The two versions differ slightly: only the ROS1 message has a Header (and
    its stamp is the driver's publish time, not a sensor time), and the ROS1
    driver never fills `temperature` (publisher_nodelet.cpp), so it stays 0
    there. Missing fields are recorded as 0 / None rather than raising.
    """
    rec = {f: int(getattr(msg, f, 0) or 0) for f in INFO_INT_FIELDS}
    rec["model_name"] = str(getattr(msg, "model_name", ""))
    rec["fault_entries"] = _as_byte_list(getattr(msg, "fault_entries", b""))
    rec["stamp_sec"] = _header_stamp_sec(getattr(msg, "header", None))
    return rec


def _collect_sensor_info(info_sensors, arrival, record):
    sn = record["serial_number"]
    info_sensors.setdefault(sn, SensorInfoData(sn)).add(arrival, record)


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
    info_topic = ""
    info_msg_hint = ""

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

    def subscribe_sensor_info(self, on_info):
        """Subscribe to the unified sensor info topic.

        Calls on_info(arrival_sec, record) with a sensor_info_record() dict.
        Unlike the point cloud, this stream is a few hundred bytes at a couple
        of Hz, so subscribing from this Python process costs nothing and needs
        no C++ probe. Returns False when the driver's message package is not
        importable (nothing was subscribed).
        """
        raise NotImplementedError

    def spin_background(self):
        ...

    def shutdown(self):
        ...


class Ros1Backend(RosBackend):
    node_hint = "nodelet"
    topic_prefix = "/cepton3/points_sn_"
    info_topic = "/cepton3/sensor_information"
    info_msg_hint = ("cannot import cepton_ros.msg. Source the catkin "
                     "workspace that built cepton_ros, or pass "
                     "--no-info-check to skip the sensor info rate check.")

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

    def subscribe_sensor_info(self, on_info):
        rospy = self._rospy
        try:
            from cepton_ros.msg import SensorInformation
        except ImportError:
            return False

        def cb(msg):
            on_info(time.time(), sensor_info_record(msg))

        self._subs.append(
            rospy.Subscriber(self.info_topic, SensorInformation, cb,
                             queue_size=50, tcp_nodelay=True)
        )
        return True

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
    info_topic = "/cepton_info"
    info_msg_hint = ("cannot import cepton_messages.msg. Source the colcon "
                     "workspace that built cepton_messages, or pass "
                     "--no-info-check to skip the sensor info rate check.")

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

    def subscribe_sensor_info(self, on_info):
        try:
            from cepton_messages.msg import CeptonSensorInfo
        except ImportError:
            return False

        def cb(msg):
            on_info(time.time(), sensor_info_record(msg))

        self._node.create_subscription(CeptonSensorInfo, self.info_topic, cb, 50)
        return True

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
    The fraction of frames whose width deviates from it is always reported.
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
    counts = Counter(widths)
    uniq = sorted(counts)
    pc_pass = len(uniq) == 1
    if expected_width is not None:
        pc_pass = pc_pass and bool(widths) and widths[0] == expected_width
    # Share of frames that are not the spec width. Without an SDK nominal
    # (--expected-points 0) the dominant width stands in for the spec, so the
    # ratio still reads as "frames deviating from the norm".
    reference = expected_width
    if reference is None and counts:
        reference = counts.most_common(1)[0][0]
    off_spec = sum(n for w, n in counts.items() if w != reference)
    result["point_count"] = {
        "unique_values": uniq[:20],
        "num_unique": len(uniq),
        "min": min(widths) if widths else None,
        "max": max(widths) if widths else None,
        "expected": expected_width,
        "reference": reference,
        "off_spec_frames": off_spec,
        "off_spec_ratio": (off_spec / len(widths)) if widths else None,
        "pass": pc_pass,
    }
    return result


def topic_serial(topic):
    """Serial number embedded in a per-sensor point cloud topic name.

    ROS1: /cepton3/points_sn_<SN>, ROS2: /serial_<SN>.
    """
    tail = topic.rsplit("_", 1)[-1]
    return int(tail) if tail.isdigit() else None


def evaluate_sensor_info(info_sensors, expected_serials, topic, nominal_hz,
                         tol, window, warmup, drop_factor):
    """Per-sensor rate check of the sensor info stream.

    Only arrival times are used: the ROS2 message has no header at all, and
    the ROS1 one is stamped by the driver at publish time rather than by the
    sensor, so there is no sensor-side time base to compare against (unlike
    the point cloud, which is judged on arrival *and* stamp).

    A sensor that was publishing points but never any info, and one that
    stopped mid-run, both fail: the missing/late messages either leave the
    sensor out of `info_sensors` or open a gap that puts 1/dt out of range.
    """
    results = []
    for sn in sorted(info_sensors):
        sd = info_sensors[sn]
        t0 = sd.arrivals[0] if sd.arrivals else 0.0
        kept = [(a, r) for a, r in zip(sd.arrivals, sd.records)
                if a - t0 >= warmup]
        times = [a for a, _r in kept]
        records = [r for _a, r in kept]
        rate, gaps = _rate_and_drop(times, nominal_hz, tol, tol, window,
                                    drop_factor)

        latest = records[-1] if records else {}
        flag_values = sorted({r["status_flags"] for r in records})
        fault_values = sorted({r["fault_summary"] for r in records})
        # Decode the union of everything seen, so a flag that was set only
        # part of the run still shows up in the report.
        flag_union = 0
        fault_union = 0
        for value in flag_values:
            flag_union |= value
        for value in fault_values:
            fault_union |= value
        # ROS1 leaves temperature at 0 (never filled by the driver); skip it
        # rather than reporting -273 C.
        temps = [r["temperature"] for r in records if r["temperature"]]
        temperature_c = None
        if temps:
            celsius = [t / 100.0 - 273.15 for t in temps]  # unit: 0.01 Kelvin
            temperature_c = {
                "min": min(celsius), "max": max(celsius),
                "mean": sum(celsius) / len(celsius),
            }

        results.append({
            "serial_number": sn,
            "model_name": latest.get("model_name", ""),
            "firmware_version": latest.get("firmware_version"),
            "messages_total": len(sd.arrivals),
            "messages_evaluated": len(times),
            "rate": rate,
            "gaps": gaps,
            "status_flags": {"values": flag_values,
                             "decoded": decode_bits(flag_union, STATUS_FLAG_BITS)},
            "fault_summary": {
                "values": fault_values,
                "messages_with_fault": sum(1 for r in records if r["fault_summary"]),
                "decoded": decode_bits(fault_union, FAULT_SUMMARY_BITS),
            },
            "temperature_c": temperature_c,
            "pass": rate["pass"],
        })

    missing = sorted(set(expected_serials) - set(info_sensors))
    return {
        "topic": topic,
        "nominal_hz": nominal_hz,
        "tolerance": tol,
        "window_sec": window,
        "missing_serials": missing,
        "sensors": results,
        "pass": bool(results) and not missing and all(r["pass"] for r in results),
    }


def evaluate_resources(samples, mem_thresh, cpu_thresh, warmup=0.0):
    """Growth analysis of RSS/CPU samples, excluding the startup warmup window.

    `samples` are (t_rel_sec, rss_mb, cpu_percent) tuples with t_rel_sec
    measured from when the resource monitor started (i.e. from process
    launch), matching the same warmup convention used for the rate/drop
    evaluation so that startup transients (allocation, connection setup)
    don't get counted as a leak trend.
    """
    samples = [s for s in samples if s[0] >= warmup]
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
_matplotlib_warned = False


def _import_matplotlib():
    """Return (pyplot, Line2D), or (None, None) when matplotlib is missing."""
    global _matplotlib_warned
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
    except ImportError:
        if not _matplotlib_warned:
            print("matplotlib not available; skipping graphs", file=sys.stderr,
                  flush=True)
            _matplotlib_warned = True
        return None, None
    return plt, Line2D


def inst_series(times, t0):
    """(elapsed, 1/dt) per interval - the per-message instantaneous rate."""
    xs, ys = [], []
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt > 0:
            xs.append(times[i] - t0)
            ys.append(1.0 / dt)
    return xs, ys


def windowed_series(times, t0, window):
    """(elapsed, mean rate) over a sliding window of at least `window` sec."""
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


GRID_KW = dict(color="0.85", linewidth=0.6)


def generate_plots(out_dir, sensors, monitor, nominal_hz, inst_tol, win_tol,
                   window, warmup, basis):
    plt, Line2D = _import_matplotlib()
    if plt is None:
        return

    samples = monitor.samples if monitor is not None else []
    period = 1.0 / nominal_hz

    def basis_times(sd):
        return sd.arrivals if basis == "arrival" else sd.stamps

    # --- Publish rate over time (authoritative basis). Windowed mean is the
    # primary series (solid); the per-frame instantaneous rate is a faint
    # background layer. Y is clipped so the tolerance band stays readable. ---
    fig, ax = plt.subplots(figsize=(11, 5))
    ymax = nominal_hz * 1.5
    ymin = nominal_hz / 1.5
    clipped = 0
    for idx, sd in enumerate(sorted(sensors.values(), key=lambda s: s.topic)):
        times = basis_times(sd)
        if len(times) < 2:
            continue
        t0 = times[0]
        color = SENSOR_COLORS[idx % len(SENSOR_COLORS)]
        label = sd.topic.rsplit("_", 1)[-1]
        ix, iy = inst_series(times, t0)
        wx, wy = windowed_series(times, t0, window)
        clipped += sum(1 for v in iy if v > ymax)
        ax.plot(ix, iy, linewidth=0.6, alpha=0.25, color=color)
        ax.plot(wx, wy, linewidth=1.2, color=color, label="SN %s" % label)
    ax.axhline(nominal_hz - win_tol, linestyle=":", color="#B00020",
               linewidth=1.2, label="lower bound %.2f Hz" % (nominal_hz - win_tol))
    ax.axhline(nominal_hz + win_tol, linestyle=":", color="#B00020",
               linewidth=1.2, label="upper bound %.2f Hz" % (nominal_hz + win_tol))
    if warmup > 0:
        ax.axvspan(0, warmup, color="0.9", label="warmup (excluded)")
    ax.set_ylim(ymin, ymax)
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
    ax.grid(True, **GRID_KW)
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
    ax.grid(True, **GRID_KW)
    if any_dt:
        ax.legend(loc="best", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_dir / "jitter.png", dpi=120)
    plt.close(fig)

    # --- CPU over time. Trend/growth stats exclude the warmup window (as does
    # evaluate_resources), so the regression is drawn only over that range;
    # the full series is still plotted so startup transients remain visible. ---
    cpu_pts = [(t, c) for t, _r, c in samples if c is not None]
    fig, ax = plt.subplots(figsize=(11, 4))
    if cpu_pts:
        ax.plot([p[0] for p in cpu_pts], [p[1] for p in cpu_pts],
                linewidth=1.0, color=SENSOR_COLORS[0], label="CPU")
    if warmup > 0:
        ax.axvspan(0, warmup, color="0.9", label="warmup (excluded)")
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel("CPU usage [%]")
    ax.set_title("Publisher CPU usage over time")
    ax.grid(True, **GRID_KW)
    if cpu_pts:
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / "cpu.png", dpi=120)
    plt.close(fig)

    # --- Memory over time with regression trend (fit only on post-warmup
    # samples, matching evaluate_resources's growth-rate calculation). ---
    mem_pts = [(t, r) for t, r, _c in samples]
    fig, ax = plt.subplots(figsize=(11, 4))
    if mem_pts:
        xs = [p[0] for p in mem_pts]
        ys = [p[1] for p in mem_pts]
        ax.plot(xs, ys, linewidth=1.0, color=SENSOR_COLORS[2], label="RSS")
        fit_xy = [(x, y) for x, y in zip(xs, ys) if x >= warmup]
        if len(fit_xy) >= 2:
            fit_x = [x for x, _y in fit_xy]
            slope, intercept = linreg(fit_x, [y for _x, y in fit_xy])
            ax.plot(fit_x, [slope * x + intercept for x in fit_x], linestyle="--",
                    color="#B00020", linewidth=1.2,
                    label="trend (%.3f MB/min)" % (slope * 60.0))
    if warmup > 0:
        ax.axvspan(0, warmup, color="0.9", label="warmup (excluded)")
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel("RSS memory [MB]")
    ax.set_title("Publisher memory (RSS) over time")
    ax.grid(True, **GRID_KW)
    if mem_pts:
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / "memory.png", dpi=120)
    plt.close(fig)


def generate_sensor_info_plot(out_dir, info_sensors, nominal_hz, tol, window,
                              warmup):
    """Sensor info publish rate over time, one series per sensor.

    Same layout as framerate.png (windowed mean solid, instantaneous 1/dt
    faint, tolerance bounds dotted), on the arrival time base -- the only one
    the info stream has.
    """
    if not any(len(sd.arrivals) >= 2 for sd in info_sensors.values()):
        print("no sensor info messages; skipping info_framerate.png",
              file=sys.stderr, flush=True)
        return
    plt, Line2D = _import_matplotlib()
    if plt is None:
        return

    fig, ax = plt.subplots(figsize=(11, 5))
    # Keep the tolerance band comfortably inside the view even when it is
    # wide relative to the nominal rate (2 +/- 0.5 Hz is a quarter of it).
    ymax = max(nominal_hz * 1.5, nominal_hz + tol * 2)
    ymin = min(nominal_hz / 1.5, nominal_hz - tol * 2)
    clipped = 0
    for idx, sn in enumerate(sorted(info_sensors)):
        times = info_sensors[sn].arrivals
        if len(times) < 2:
            continue
        t0 = times[0]
        color = SENSOR_COLORS[idx % len(SENSOR_COLORS)]
        ix, iy = inst_series(times, t0)
        wx, wy = windowed_series(times, t0, window)
        clipped += sum(1 for v in iy if v > ymax or v < ymin)
        ax.plot(ix, iy, linewidth=0.6, alpha=0.25, color=color)
        ax.plot(wx, wy, linewidth=1.2, color=color, label="SN %s" % sn)
    ax.axhline(nominal_hz - tol, linestyle=":", color="#B00020", linewidth=1.2,
               label="lower bound %.2f Hz" % (nominal_hz - tol))
    ax.axhline(nominal_hz + tol, linestyle=":", color="#B00020", linewidth=1.2,
               label="upper bound %.2f Hz" % (nominal_hz + tol))
    if warmup > 0:
        ax.axvspan(0, warmup, color="0.9", label="warmup (excluded)")
    ax.set_ylim(ymin, ymax)
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
    ax.set_ylabel("Sensor info rate [Hz]")
    title = ("Sensor info publish rate (nominal %.2f Hz +/-%.2f, basis=arrival)"
             % (nominal_hz, tol))
    if clipped:
        title += "  [%d samples outside [%.1f, %.1f] Hz clipped]" % (
            clipped, ymin, ymax)
    ax.set_title(title)
    ax.grid(True, **GRID_KW)
    fig.tight_layout()
    fig.savefig(out_dir / "info_framerate.png", dpi=120)
    plt.close(fig)


def generate_system_plot(out_dir, system_samples, process_samples, sensors,
                         nominal_hz, warmup, basis):
    """Machine state over time, with the point cloud's bunched periods shaded.

    The point of the figure is the overlay: if the delivery cadence degrades
    while machine CPU, the hot core's clock and temperature stay flat, the
    cause is not the host, and that is worth being able to see at a glance.
    """
    if not system_samples and not process_samples:
        return
    plt, _Line2D = _import_matplotlib()
    if plt is None:
        return

    # Shade the stretches where arrivals were bunched (dt well under nominal).
    period = 1.0 / nominal_hz
    spans = []
    for sd in sensors.values():
        times = sd.arrivals if basis == "arrival" else sd.stamps
        if len(times) < 2:
            continue
        t0 = times[0]
        bucket = {}
        for i in range(1, len(times)):
            k = int((times[i] - t0) // 60)
            hit, total = bucket.get(k, (0, 0))
            bucket[k] = (hit + (1 if times[i] - times[i - 1] < 0.1 * period else 0),
                         total + 1)
        for k in sorted(bucket):
            hit, total = bucket[k]
            if total and 100.0 * hit / total > 5.0:
                spans.append((k * 60.0, (k + 1) * 60.0))
        break  # one sensor is enough to mark the periods

    panels = [
        ("Machine CPU [%]", system_samples, "cpu_busy_pct", SENSOR_COLORS[0]),
        ("Core clock [MHz]", system_samples, "freq_max_mhz", SENSOR_COLORS[1]),
        ("Max temp [degC]", system_samples, "temp_c", SENSOR_COLORS[2]),
        ("NIC rx [Mbps]", system_samples, "nic_rx_mbps", SENSOR_COLORS[3]),
    ]
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 9), sharex=True)
    for ax, (label, samples, key, color) in zip(axes, panels):
        pts = [(s["t_rel_sec"], s[key]) for s in (samples or [])
               if s.get(key) is not None]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], linewidth=1.0,
                    color=color)
        for lo, hi in spans:
            ax.axvspan(lo, hi, color="#D55E00", alpha=0.12, linewidth=0)
        if warmup > 0:
            ax.axvspan(0, warmup, color="0.9", linewidth=0)
        ax.set_ylabel(label, fontsize=9)
        ax.grid(True, **GRID_KW)
    axes[-1].set_xlabel("Elapsed time [s]")
    axes[0].set_title("Machine state (shaded = point cloud arriving bunched, "
                      "grey = warmup)")
    fig.tight_layout()
    fig.savefig(out_dir / "system.png", dpi=120)
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


# Column order of sensor_info.csv (arrival_sec is added by the writer).
INFO_CSV_COLUMNS = (
    "arrival_sec", "stamp_sec", "serial_number", "handle", "model_name",
    "model", "part_number", "firmware_version", "power_up_timestamp",
    "time_sync_offset", "time_sync_drift", "return_count", "channel_count",
    "status_flags", "temperature", "fault_summary", "fault_entries",
)


def write_sensor_info_csv(out_dir, info_sensors, filename="sensor_info.csv"):
    """One row per received info message, all sensors interleaved by arrival."""
    rows = sorted(
        ((arrival, rec) for sd in info_sensors.values()
         for arrival, rec in zip(sd.arrivals, sd.records)),
        key=lambda row: row[0],
    )
    path = out_dir / filename
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(INFO_CSV_COLUMNS)
        for arrival, rec in rows:
            values = [arrival]
            for col in INFO_CSV_COLUMNS[1:]:
                if col == "fault_entries":
                    values.append("".join("%02x" % b for b in rec["fault_entries"]))
                else:
                    value = rec.get(col)
                    values.append("" if value is None else value)
            w.writerow(values)


def write_resource_csv(out_dir, monitor, filename="resource.csv"):
    path = out_dir / filename
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rel_sec", "rss_mb", "cpu_percent"])
        for t, rss, cpu in monitor.samples:
            w.writerow([t, rss, "" if cpu is None else cpu])


SYSTEM_CSV_COLUMNS = (
    "t_rel_sec", "cpu_busy_pct", "cpu_max_core_pct", "load1", "temp_c",
    "net_rx_softirq_s", "udp_in_s", "udp_rcvbuf_err_s", "nic_rx_mbps",
    "nic_drop_s", "freq_min_mhz", "freq_median_mhz", "freq_max_mhz",
    "top_processes",
)
PROCESS_CSV_COLUMNS = (
    "t_rel_sec", "threads", "vmrss_mb", "vmsize_mb", "vmas", "minflt_s",
    "hot_thread_last_cpu", "hot_thread_cpu_pct",
)
PERF_CSV_COLUMNS = ("t_rel_sec", "effective_ghz", "busy_ratio")


def write_dict_csv(out_dir, samples, columns, filename):
    path = out_dir / filename
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(columns)
        for s in samples:
            w.writerow(["" if s.get(c) is None else s.get(c) for c in columns])


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


def summarise_samples(samples, keys):
    """min/mean/max per key, plus the total change for the *_s rate columns.

    Recorded, not judged: there is no defensible threshold for these yet, so
    they carry no pass/fail of their own.
    """
    out = {}
    for k in keys:
        vals = [s[k] for s in samples if s.get(k) is not None]
        if not vals:
            out[k] = None
            continue
        entry = {"min": min(vals), "max": max(vals),
                 "mean": sum(vals) / len(vals)}
        if k.endswith("_s"):
            entry["total"] = sum(vals)  # rate x interval is folded in by caller
        out[k] = entry
    return out


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
    env = summary.get("environment") or {}
    if env:
        cs = env.get("cpu_scaling", {})
        sc = env.get("sysctl", {})
        print("  host                   : %s  kernel %s  %d cores" % (
            env.get("host", {}).get("hostname"), env.get("host", {}).get("kernel"),
            env.get("host", {}).get("cpu_count") or 0))
        print("  cpu scaling            : governor=%s driver=%s epp=%s turbo=%s" % (
            cs.get("governor"), cs.get("driver"),
            cs.get("energy_performance_preference"),
            "off" if cs.get("intel_pstate_no_turbo") else "on"))
        print("  socket buffers (sysctl): rmem %s/%s  wmem %s/%s  (default/max)" % (
            sc.get("net.core.rmem_default"), sc.get("net.core.rmem_max"),
            sc.get("net.core.wmem_default"), sc.get("net.core.wmem_max")))
        ratio = env.get("message_per_send_buffer")
        if ratio is not None:
            print("  publisher send buffer  : %s B for a %s B message (message is "
                  "%.1fx the buffer)" % (env.get("min_send_buffer_bytes"),
                                         env.get("message_bytes"), ratio))
        elif env.get("min_send_buffer_bytes") is not None:
            print("  publisher send buffer  : %s B" % env["min_send_buffer_bytes"])
        print("  sensor interface       : %s" % env.get("sensor_interface"))
        print("  driver revision        : %s" % env.get("driver_revision"))
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
        ratio = pc.get("off_spec_ratio")
        ref = "spec" if exp is not None else "dominant width %s" % pc.get("reference")
        print("     off-spec     : %s  (%d/%d frames != %s)" % (
            "n/a" if ratio is None else "%.4f%%" % (ratio * 100.0),
            pc.get("off_spec_frames", 0), s["frames_evaluated"], ref))
    print("  (* = authoritative basis for pass/fail)", flush=True)
    print("-" * 76, flush=True)
    info = summary.get("sensor_info")
    if info is None:
        print("  sensor_info   : skipped (--no-info-check)", flush=True)
    else:
        print("  sensor_info %s  (nominal %.2f Hz ±%.2f, window %.1fs)" % (
            info["topic"], info["nominal_hz"], info["tolerance"],
            info["window_sec"]), flush=True)
        for s in info["sensors"]:
            inst = s["rate"]["instantaneous"]
            win = s["rate"]["windowed"]
            print("    SN %-10s : %s  (msgs=%d)" % (
                s["serial_number"], _pf(s["pass"]), s["messages_evaluated"]))
            print("      inst rate  : %s  (out=%d/%d min=%s max=%s mean=%s)" % (
                _pf(inst["pass"]), inst["out_of_range"], inst["samples"],
                _fmt(inst["min"]), _fmt(inst["max"]), _fmt(inst["mean"])))
            print("      win  rate  : %s  (out=%d/%d min=%s max=%s mean=%s)" % (
                _pf(win["pass"]), win["out_of_range"], win["samples"],
                _fmt(win["min"]), _fmt(win["max"]), _fmt(win["mean"])))
            extra = []
            if s["status_flags"]["decoded"]:
                extra.append("flags=%s" % "|".join(s["status_flags"]["decoded"]))
            if s["fault_summary"]["messages_with_fault"]:
                extra.append("faults=%s (%d msgs)" % (
                    "|".join(s["fault_summary"]["decoded"]),
                    s["fault_summary"]["messages_with_fault"]))
            if s["temperature_c"]:
                extra.append("temp=%s..%s°C" % (_fmt(s["temperature_c"]["min"]),
                                                _fmt(s["temperature_c"]["max"])))
            if extra:
                print("      info       : %s" % "  ".join(extra))
        if not info["sensors"]:
            print("    FAIL  (no sensor info messages received)", flush=True)
        if info["missing_serials"]:
            print("    FAIL  no info from SN: %s" % ", ".join(
                str(sn) for sn in info["missing_serials"]), flush=True)
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
    sysm = summary.get("system") or {}
    if sysm:
        def rng(key, fmt="%.1f"):
            e = sysm.get(key)
            return "n/a" if not e else ("%s..%s" % (fmt % e["min"], fmt % e["max"]))
        print("  machine (info): cpu %s%%  temp %s degC  nic %s Mbps" % (
            rng("cpu_busy_pct"), rng("temp_c"), rng("nic_rx_mbps")))
        for key, label in (("udp_rcvbuf_err_s", "udp rcvbuf err"),
                           ("nic_drop_s", "nic drops")):
            e = sysm.get(key)
            if e:
                print("  %-14s: %.0f/s peak (recorded only, not a pass criterion)"
                      % (label, e["max"]))
    procm = summary.get("publisher_process") or {}
    if procm:
        def pv(key, fmt="%.0f"):
            e = procm.get(key)
            return "n/a" if not e else ("%s..%s" % (fmt % e["min"], fmt % e["max"]))
        print("  publisher (info): threads %s  vmas %s  busiest thread %s%%" % (
            pv("threads"), pv("vmas"), pv("hot_thread_cpu_pct")))
    clock = summary.get("publisher_clock") or {}
    if clock.get("effective_ghz"):
        e = clock["effective_ghz"]
        print("  publisher clock : %.2f..%.2f GHz effective (cycles/task-clock)"
              % (e["min"], e["max"]))
    elif clock.get("unavailable_reason"):
        print("  publisher clock : n/a (%s)" % clock["unavailable_reason"])
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
    p.add_argument("--expected-sensors", type=int, default=1,
                   help="Number of sensor topics that must appear (default: 1)")
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
    p.add_argument("--info-rate", type=float, default=2.0,
                   help="Nominal sensor info publish rate per sensor in Hz "
                        "(default: 2.0)")
    p.add_argument("--info-rate-tolerance", type=float, default=0.5,
                   help="Allowed Hz deviation of the sensor info rate, applied "
                        "to both the per-message 1/dt and the windowed mean "
                        "(default: 0.5)")
    p.add_argument("--info-rate-window", type=float, default=5.0,
                   help="Sliding window length in seconds for the windowed "
                        "mean sensor info rate. Longer than the point cloud "
                        "window because the info stream is ~10x slower "
                        "(default: 5.0)")
    p.add_argument("--no-info-check", action="store_true",
                   help="Do not subscribe to the sensor info topic and skip "
                        "the info rate check (dry runs without the driver)")
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
    p.add_argument("--system-interval", type=float, default=5.0,
                   help="Sampling interval in seconds for the machine-wide "
                        "metrics (CPU, kernel UDP counters, NIC, top "
                        "processes). Coarser than --resource-interval because "
                        "it walks /proc (default: 5.0)")
    p.add_argument("--sensor-interface", default=None,
                   help="NIC the sensor data arrives on, recorded in "
                        "environment.json (default: auto-detect the busiest "
                        "interface at startup)")
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
    system_monitor = None
    process_monitor = None
    perf_counter = None
    probe_popen = None
    probe_monitor = None
    sensors = {}
    environment = None
    info_sensors = {}
    info_enabled = not args.no_info_check
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

        # Snapshot the machine before measuring: kernel socket buffers, CPU
        # scaling and DDS settings are invisible in the driver's parameters but
        # decide whether a multi-megabyte message leaves the publisher cleanly.
        interface = args.sensor_interface or busiest_interface()
        environment = collect_environment(
            pid, interface,
            message_bytes=(args.expected_points * args.aggregation_frame_count * 32
                           if args.expected_points > 0 else None))
        (out_dir / "environment.json").write_text(
            json.dumps(environment, indent=2, ensure_ascii=False))

        system_monitor = SystemMonitor(args.system_interval, interface)
        system_monitor.start()
        if pid is not None:
            process_monitor = ProcessMonitor(pid, args.resource_interval)
            process_monitor.start()
            perf_counter = PerfCounter(pid, args.resource_interval)
            if not perf_counter.start_if_possible():
                print("note: no process-attributed clock (%s)"
                      % perf_counter.unavailable_reason, flush=True)

        backend.init_client()
        topics = backend.discover_sensor_topics(args.expected_sensors, args.startup_timeout)
        if len(topics) < args.expected_sensors:
            print("ERROR: found %d/%d sensor topics: %s" %
                  (len(topics), args.expected_sensors, topics), file=sys.stderr)
            return 2

        if info_enabled:
            if not backend.subscribe_sensor_info(
                    lambda a, rec: _collect_sensor_info(info_sensors, a, rec)):
                print("ERROR: %s" % backend.info_msg_hint, file=sys.stderr)
                return 2
            print("subscribing to %s (sensor info rate check)" %
                  backend.info_topic, flush=True)

        if args.rate_method == "probe":
            if not backend.probe_available():
                print("ERROR: %s" % backend.probe_build_hint(), file=sys.stderr)
                return 2
            # Spin before launching the probe: the info stream is already
            # flowing and its queue would otherwise sit undrained for the
            # several seconds the probe takes to come up.
            backend.spin_background()
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
        for extra in (system_monitor, process_monitor, perf_counter):
            if extra is not None:
                extra.stop()
                extra.join(timeout=5)
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
    info_result = None
    if info_enabled:
        expected_serials = [sn for sn in (topic_serial(t) for t in topics)
                            if sn is not None]
        info_result = evaluate_sensor_info(
            info_sensors, expected_serials, backend.info_topic, args.info_rate,
            args.info_rate_tolerance, args.info_rate_window, args.warmup,
            args.drop_factor)
    res = evaluate_resources(monitor.samples if monitor else [],
                             args.mem_growth_threshold, args.cpu_growth_threshold,
                             args.warmup)
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
        and (info_result is None or info_result["pass"])
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
        "info_rate": args.info_rate,
        "info_rate_tolerance": args.info_rate_tolerance,
        "expected_points_total": expected_total,
        "duration_sec": args.duration,
        "environment": environment,
        "system": (summarise_samples(
            system_monitor.samples,
            ("cpu_busy_pct", "cpu_max_core_pct", "temp_c", "udp_in_s",
             "udp_rcvbuf_err_s", "nic_rx_mbps", "nic_drop_s",
             "freq_min_mhz", "freq_median_mhz", "freq_max_mhz"))
            if system_monitor is not None else None),
        "publisher_process": (summarise_samples(
            process_monitor.samples,
            ("threads", "vmrss_mb", "vmsize_mb", "vmas", "minflt_s",
             "hot_thread_cpu_pct"))
            if process_monitor is not None else None),
        "publisher_clock": ({"unavailable_reason": perf_counter.unavailable_reason,
                             "samples": len(perf_counter.samples),
                             **summarise_samples(perf_counter.samples,
                                                 ("effective_ghz", "busy_ratio"))}
                            if perf_counter is not None else None),
        "sensors": sensor_results,
        "sensor_info": info_result,
        "process": process_result,
        "resources": res,
        "probe": probe_info,
        "overall_pass": overall,
    }

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    write_sensor_csvs(out_dir, sensors, nominal_hz)
    if info_enabled:
        write_sensor_info_csv(out_dir, info_sensors)
    if system_monitor is not None:
        write_dict_csv(out_dir, system_monitor.samples, SYSTEM_CSV_COLUMNS,
                       "resource_system.csv")
    if process_monitor is not None:
        write_dict_csv(out_dir, process_monitor.samples, PROCESS_CSV_COLUMNS,
                       "resource_process.csv")
    if perf_counter is not None and perf_counter.samples:
        write_dict_csv(out_dir, perf_counter.samples, PERF_CSV_COLUMNS,
                       "publisher_clock.csv")
    if monitor is not None:
        write_resource_csv(out_dir, monitor)
    if probe_monitor is not None:
        write_resource_csv(out_dir, probe_monitor, "resource_probe.csv")
    generate_plots(out_dir, sensors, monitor, nominal_hz, args.inst_tolerance,
                   args.rate_tolerance, args.rate_window, args.warmup,
                   args.rate_basis)
    generate_system_plot(out_dir,
                         system_monitor.samples if system_monitor else [],
                         process_monitor.samples if process_monitor else [],
                         sensors, nominal_hz, args.warmup, args.rate_basis)
    if info_enabled:
        generate_sensor_info_plot(out_dir, info_sensors, args.info_rate,
                                  args.info_rate_tolerance,
                                  args.info_rate_window, args.warmup)
    print_report(summary)
    print("Output written to %s" % out_dir, flush=True)
    if probe_crashed:
        return 2
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
