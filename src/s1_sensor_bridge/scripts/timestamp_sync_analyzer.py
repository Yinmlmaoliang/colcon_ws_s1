#!/usr/bin/env python3
"""
Timestamp Synchronization Analyzer
Analyzes timestamp synchronization between camera (left/right), LiDAR, and IMU sensors.
Subscribes to: /{prefix}/left/image_raw, /{prefix}/right/image_raw,
               /{prefix}/lidar, /{prefix}/imu
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from collections import deque
import statistics
import time


RESET  = '\033[0m'
BOLD   = '\033[1m'
GREEN  = '\033[92m'
YELLOW = '\033[93m'
RED    = '\033[91m'
CYAN   = '\033[96m'
BLUE   = '\033[94m'


def colorize(text, color):
    return f'{color}{text}{RESET}'


class SensorStats:
    """Per-sensor rolling statistics tracker."""

    def __init__(self, name: str, window: int):
        self.name = name
        self.timestamps = deque(maxlen=window)
        self.intervals = deque(maxlen=window)
        self.last_stamp: float | None = None
        self.last_wall: float | None = None
        self.recv_count = 0
        self.delay_samples = deque(maxlen=window)  # header stamp vs wall clock

    def update(self, stamp_sec: float):
        self.recv_count += 1
        now = time.time()

        if self.last_stamp is not None:
            interval = stamp_sec - self.last_stamp
            if 0 < interval < 10.0:  # sanity filter
                self.intervals.append(interval * 1000)  # ms

        delay = (now - stamp_sec) * 1000  # reception delay in ms
        self.delay_samples.append(delay)

        self.timestamps.append(stamp_sec)
        self.last_stamp = stamp_sec
        self.last_wall = now

    def freq_stats(self):
        if len(self.intervals) < 2:
            return None
        mean_interval = statistics.mean(self.intervals)
        freq = 1000.0 / mean_interval if mean_interval > 0 else 0
        return {
            'freq': freq,
            'interval_mean_ms': mean_interval,
            'interval_std_ms': statistics.stdev(self.intervals),
            'interval_min_ms': min(self.intervals),
            'interval_max_ms': max(self.intervals),
        }

    def delay_stats(self):
        if len(self.delay_samples) < 2:
            return None
        return {
            'mean': statistics.mean(self.delay_samples),
            'std': statistics.stdev(self.delay_samples),
            'min': min(self.delay_samples),
            'max': max(self.delay_samples),
        }


class PairSyncStats:
    """
    Tracks timestamp difference between two sensors.

    Matching strategy (controlled by match_tolerance_ms):
    - match_tolerance_ms == 0: exact stamp match only. Any pair whose stamps
      differ by more than 1 ns is counted as "unmatched" and excluded from
      the diff statistics.  Use this for sensors that share the same stamp
      (e.g. left/right cameras split from the same TCP frame).
    - match_tolerance_ms > 0: nearest-neighbour within that tolerance window.
      Use this for sensors with independent clocks (e.g. camera vs lidar).
    """

    EXACT_TOL_SEC = 1e-6  # 1 µs — treat as "same stamp"

    def __init__(self, name_a: str, name_b: str, window: int,
                 match_tolerance_ms: float = 0.0):
        self.label = f'{name_a} <-> {name_b}'
        self.diffs = deque(maxlen=window)
        self.unmatched = 0       # frames with no counterpart
        self._match_tol = match_tolerance_ms / 1000.0
        self._buf_a: deque = deque(maxlen=64)
        self._buf_b: deque = deque(maxlen=64)

    def _try_match(self, stamp: float, buf: deque):
        if not buf:
            self.unmatched += 1
            return
        nearest = min(buf, key=lambda s: abs(s - stamp))
        diff = abs(stamp - nearest)
        tol = self._match_tol if self._match_tol > 0 else self.EXACT_TOL_SEC
        if diff <= tol:
            self.diffs.append(diff * 1000)
        else:
            self.unmatched += 1

    def add_a(self, stamp: float):
        self._buf_a.append(stamp)
        self._try_match(stamp, self._buf_b)

    def add_b(self, stamp: float):
        self._buf_b.append(stamp)
        self._try_match(stamp, self._buf_a)

    def stats(self):
        if len(self.diffs) < 2:
            return None
        return {
            'mean': statistics.mean(self.diffs),
            'std': statistics.stdev(self.diffs),
            'min': min(self.diffs),
            'max': max(self.diffs),
            'p95': sorted(self.diffs)[int(len(self.diffs) * 0.95)],
            'count': len(self.diffs),
        }

    def quality_label(self, mean_ms: float) -> str:
        if mean_ms < 5:
            return colorize('EXCELLENT (<5ms)', GREEN)
        elif mean_ms < 20:
            return colorize('GOOD (<20ms)', GREEN)
        elif mean_ms < 50:
            return colorize('ACCEPTABLE (<50ms)', YELLOW)
        else:
            return colorize('POOR (>=50ms)', RED)


class TimestampSyncAnalyzer(Node):
    def __init__(self):
        super().__init__('timestamp_sync_analyzer')

        self.declare_parameter('topic_prefix', 's1_01')
        self.declare_parameter('window_size', 200)
        self.declare_parameter('report_interval', 5.0)
        self.declare_parameter('enable_imu', True)

        topic_prefix = self.get_parameter('topic_prefix').value
        window = self.get_parameter('window_size').value
        self.report_interval = self.get_parameter('report_interval').value
        enable_imu = self.get_parameter('enable_imu').value

        self.sensors = {
            'left':  SensorStats('Left Camera',  window),
            'right': SensorStats('Right Camera', window),
            'lidar': SensorStats('LiDAR',        window),
        }
        if enable_imu:
            self.sensors['imu'] = SensorStats('IMU', window)

        self.pairs = {
            # Camera vs lidar: different clocks, use 600ms tolerance (>1 lidar period)
            'left_lidar':  PairSyncStats('Left Cam',  'LiDAR',     window, match_tolerance_ms=600.0),
            'right_lidar': PairSyncStats('Right Cam', 'LiDAR',     window, match_tolerance_ms=600.0),
            # Left vs right: same TCP frame → same stamp, use exact match
            'left_right':  PairSyncStats('Left Cam',  'Right Cam', window, match_tolerance_ms=0.0),
        }

        # Match publisher QoS to avoid silent frame drops:
        # - camera and lidar publishers use RELIABLE → subscribe RELIABLE
        # - IMU publisher uses BEST_EFFORT → subscribe BEST_EFFORT
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Image, f'/{topic_prefix}/left/image_raw',
            self._make_cb('left'), reliable_qos)
        self.create_subscription(
            Image, f'/{topic_prefix}/right/image_raw',
            self._make_cb('right'), reliable_qos)
        self.create_subscription(
            PointCloud2, f'/{topic_prefix}/lidar',
            self._make_cb('lidar'), reliable_qos)
        if enable_imu:
            self.create_subscription(
                Imu, f'/{topic_prefix}/imu',
                self._make_cb('imu'), best_effort_qos)

        self.create_timer(self.report_interval, self._report)
        self._start_wall = time.time()

        print(colorize('=' * 72, CYAN))
        print(colorize('  Timestamp Sync Analyzer  --  S1 Sensor Suite', BOLD + CYAN))
        print(colorize('=' * 72, CYAN))
        self.get_logger().info(
            f'Subscribed topics under /{topic_prefix}/: '
            f'left/image_raw, right/image_raw, lidar'
            + (', imu' if enable_imu else ''))
        self.get_logger().info(
            f'Window={window} frames  |  Report every {self.report_interval}s')

    # ------------------------------------------------------------------ #
    def _stamp_sec(self, msg) -> float:
        h = msg.header.stamp
        return h.sec + h.nanosec * 1e-9

    def _make_cb(self, key: str):
        def cb(msg):
            stamp = self._stamp_sec(msg)
            self.sensors[key].update(stamp)
            # Use nearest-neighbor matching: each side pushes its stamp into
            # the pair buffer and the pair finds the closest stamp on the
            # other side.  This avoids false large diffs from rate mismatch.
            if key == 'left':
                self.pairs['left_lidar'].add_a(stamp)
                self.pairs['left_right'].add_a(stamp)
            elif key == 'right':
                self.pairs['right_lidar'].add_a(stamp)
                self.pairs['left_right'].add_b(stamp)
            elif key == 'lidar':
                self.pairs['left_lidar'].add_b(stamp)
                self.pairs['right_lidar'].add_b(stamp)
        return cb

    # ------------------------------------------------------------------ #
    def _fmt_stats(self, s, unit='ms') -> str:
        return (f'mean={s["mean"]:.2f}{unit}  std={s["std"]:.2f}{unit}  '
                f'min={s["min"]:.2f}{unit}  max={s["max"]:.2f}{unit}')

    def _report(self):
        elapsed = time.time() - self._start_wall
        sep = colorize('─' * 72, BLUE)

        lines = []
        lines.append(colorize('=' * 72, CYAN))
        lines.append(colorize(
            f'  Timestamp Sync Report  |  Running {elapsed:.0f}s', BOLD + CYAN))
        lines.append(colorize('=' * 72, CYAN))

        # ── Per-sensor frequency & delay ─────────────────────────────── #
        lines.append(colorize('\n[1] Per-Sensor Publish Rate & Reception Delay', BOLD))
        lines.append(sep)
        any_data = False
        for key, s in self.sensors.items():
            fs = s.freq_stats()
            ds = s.delay_stats()
            count_str = f'recv={s.recv_count}'
            if fs is None and ds is None:
                lines.append(f'  {s.name:<18} {colorize("(no data)", YELLOW)}  {count_str}')
                continue
            any_data = True
            freq_str = (f'{fs["freq"]:.2f} Hz  '
                        f'interval: {fs["interval_mean_ms"]:.1f}±{fs["interval_std_ms"]:.1f}ms'
                        f'  [{fs["interval_min_ms"]:.1f}, {fs["interval_max_ms"]:.1f}]ms'
                        if fs else 'rate: N/A')
            delay_str = (f'delay: {ds["mean"]:.1f}±{ds["std"]:.1f}ms'
                         f'  max={ds["max"]:.1f}ms'
                         if ds else '')
            lines.append(f'  {colorize(s.name, BOLD):<27} {count_str}')
            lines.append(f'    Rate  : {freq_str}')
            lines.append(f'    Delay : {delay_str}')

        if not any_data:
            lines.append(colorize(
                '  [!] No data received on any topic. Check if sensors are publishing.',
                RED + BOLD))
            print('\n'.join(lines))
            return

        # ── Pairwise timestamp difference ────────────────────────────── #
        lines.append(colorize('\n[2] Pairwise Timestamp Difference (|Δt|)', BOLD))
        lines.append(sep)
        for pair in self.pairs.values():
            s = pair.stats()
            unmatched_str = (f'  unmatched={pair.unmatched}' if pair.unmatched > 0 else '')
            if s is None:
                lines.append(f'  {pair.label:<30} {colorize("(insufficient data)", YELLOW)}{unmatched_str}')
                continue
            quality = pair.quality_label(s['mean'])
            lines.append(
                f'  {colorize(pair.label, BOLD):<39}  n={s["count"]}{unmatched_str}  {quality}')
            lines.append(
                f'    mean={s["mean"]:.2f}ms  std={s["std"]:.2f}ms  '
                f'p95={s["p95"]:.2f}ms  '
                f'[{s["min"]:.2f}, {s["max"]:.2f}]ms')

        # ── Synchronization verdict ───────────────────────────────────── #
        lines.append(colorize('\n[3] Synchronization Verdict', BOLD))
        lines.append(sep)
        verdicts = []
        for pair in self.pairs.values():
            s = pair.stats()
            if s:
                verdicts.append((pair.label, s['mean'], s['p95']))

        if verdicts:
            worst = max(verdicts, key=lambda x: x[2])
            best  = min(verdicts, key=lambda x: x[1])
            lines.append(f'  Best  pair : {best[0]}  mean={best[1]:.2f}ms')
            lines.append(f'  Worst pair : {worst[0]}  p95={worst[2]:.2f}ms')
            if worst[2] < 20:
                verdict = colorize('SYNC OK — all pairs within 20ms p95', GREEN + BOLD)
            elif worst[2] < 50:
                verdict = colorize('SYNC MARGINAL — some pairs exceed 20ms p95', YELLOW + BOLD)
            else:
                verdict = colorize('SYNC POOR — pairs exceed 50ms p95, check trigger/PTP', RED + BOLD)
            lines.append(f'\n  Overall: {verdict}')
        else:
            lines.append(colorize('  Insufficient data for verdict.', YELLOW))

        lines.append(colorize('=' * 72, CYAN))
        print('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = TimestampSyncAnalyzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
