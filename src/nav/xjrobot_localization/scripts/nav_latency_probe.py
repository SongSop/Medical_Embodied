#!/usr/bin/env python3
"""Probe navigation point-cloud and TF latency.

This tool subscribes to the point-cloud processing chain and estimates:
  - per-topic header age: now - msg.header.stamp
  - receive frequency and header-stamp frequency
  - same-stamp hop latency between configured upstream/downstream topics
  - latest TF age for configured frame pairs
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterable, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time as RclTime
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


TopicKind = str
StampKey = Tuple[int, int]
TfPair = Tuple[str, str]
Hop = Tuple[str, str]
TimedSample = Tuple[float, float]


DEFAULT_POINTCLOUD_TOPICS = [
    "/livox/lidar_rotated/points",
    "/ground_segmentation/raw_points",
    "/ground_segmentation/ground_points",
    "/ground_segmentation/obstacle_points",
]

DEFAULT_SCAN_TOPICS = ["/scan"]
DEFAULT_ODOM_TOPICS = ["/odom_fastlio", "/odom"]

DEFAULT_TF_PAIRS = [
    ("map_fastlio", "lidar_3d_frame_fastlio"),
    ("odom", "lidar_3d_link"),
    ("map", "odom"),
]

DEFAULT_HOPS = [
    ("/livox/lidar_rotated/points", "/ground_segmentation/raw_points"),
    ("/livox/lidar_rotated/points", "/ground_segmentation/obstacle_points"),
    ("/ground_segmentation/obstacle_points", "/scan"),
    ("/livox/lidar_rotated/points", "/scan"),
]


def stamp_to_key(stamp: Time) -> StampKey:
    return int(stamp.sec), int(stamp.nanosec)


def stamp_to_seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite_ms(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value * 1000.0:8.2f}"


def finite_hz(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value) or value <= 0.0:
        return "-"
    return f"{value:7.2f}"


def prune_time_window(samples: Deque[TimedSample], now: float, window_sec: float) -> None:
    cutoff = now - window_sec
    while samples and samples[0][0] < cutoff:
        samples.popleft()


def sample_values(samples: Iterable[TimedSample]) -> List[float]:
    return [value for _, value in samples]


@dataclass
class TopicStats:
    name: str
    kind: TopicKind
    window_sec: float
    count: int = 0
    last_header_stamp: Optional[float] = None
    last_receive_time: Optional[float] = None
    last_age: Optional[float] = None
    ages: Deque[TimedSample] = field(default_factory=deque)
    header_periods: Deque[TimedSample] = field(default_factory=deque)
    receive_periods: Deque[TimedSample] = field(default_factory=deque)

    def prune(self, now: float) -> None:
        prune_time_window(self.ages, now, self.window_sec)
        prune_time_window(self.header_periods, now, self.window_sec)
        prune_time_window(self.receive_periods, now, self.window_sec)

    def update(self, header_stamp: Time, receive_time: float) -> None:
        header_time = stamp_to_seconds(header_stamp)
        self.count += 1
        self.last_age = receive_time - header_time
        self.ages.append((receive_time, self.last_age))

        if self.last_header_stamp is not None:
            dt = header_time - self.last_header_stamp
            if dt > 0.0:
                self.header_periods.append((receive_time, dt))
        if self.last_receive_time is not None:
            dt = receive_time - self.last_receive_time
            if dt > 0.0:
                self.receive_periods.append((receive_time, dt))

        self.last_header_stamp = header_time
        self.last_receive_time = receive_time
        self.prune(receive_time)

    @staticmethod
    def hz(periods: Iterable[TimedSample]) -> Optional[float]:
        values = sample_values(periods)
        if not values:
            return None
        avg = sum(values) / len(values)
        if avg <= 0.0:
            return None
        return 1.0 / avg

    @property
    def receive_hz(self) -> Optional[float]:
        return self.hz(self.receive_periods)

    @property
    def header_hz(self) -> Optional[float]:
        return self.hz(self.header_periods)

    @staticmethod
    def average(values: Iterable[TimedSample]) -> Optional[float]:
        values = sample_values(values)
        if not values:
            return None
        return sum(values) / len(values)

    @property
    def average_age(self) -> Optional[float]:
        return self.average(self.ages)

    @property
    def max_age(self) -> Optional[float]:
        if not self.ages:
            return None
        return max(sample_values(self.ages))


@dataclass
class HopStats:
    source: str
    target: str
    window_sec: float
    samples: Deque[TimedSample] = field(default_factory=deque)

    def prune(self, now: float) -> None:
        prune_time_window(self.samples, now, self.window_sec)

    def add(self, latency: float, receive_time: float) -> None:
        if latency >= 0.0:
            self.samples.append((receive_time, latency))
            self.prune(receive_time)

    @property
    def latest(self) -> Optional[float]:
        return self.samples[-1][1] if self.samples else None

    @property
    def average(self) -> Optional[float]:
        if not self.samples:
            return None
        values = sample_values(self.samples)
        return sum(values) / len(values)

    @property
    def maximum(self) -> Optional[float]:
        if not self.samples:
            return None
        return max(sample_values(self.samples))


class NavLatencyProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("nav_latency_probe")
        self.args = args
        self.topic_stats: Dict[str, TopicStats] = {}
        self.first_seen_by_stamp: Dict[StampKey, Dict[str, float]] = defaultdict(dict)
        self.hop_stats: Dict[Hop, HopStats] = {
            hop: HopStats(*hop, args.window_sec) for hop in args.hop
        }
        self.tf_age_samples: Dict[TfPair, Deque[TimedSample]] = {
            pair: deque() for pair in args.tf
        }

        self.tf_buffer = Buffer(cache_time=Duration(seconds=max(10.0, args.tf_cache_sec)))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for topic in args.pointcloud_topic:
            self._subscribe(topic, "PointCloud2", PointCloud2)
        for topic in args.scan_topic:
            self._subscribe(topic, "LaserScan", LaserScan)
        for topic in args.odom_topic:
            self._subscribe(topic, "Odometry", Odometry)

        self.timer = self.create_timer(args.period, self.print_report)
        self.get_logger().info(
            f"Probing {len(self.topic_stats)} topics, {len(self.hop_stats)} hops, "
            f"{len(args.tf)} TF pairs. Press Ctrl-C to stop."
        )

    def _subscribe(self, topic: str, kind: TopicKind, msg_type) -> None:
        self.topic_stats[topic] = TopicStats(topic, kind, self.args.window_sec)
        self.create_subscription(
            msg_type,
            topic,
            lambda msg, topic=topic: self._on_header_msg(topic, msg),
            qos_profile_sensor_data,
        )

    def _on_header_msg(self, topic: str, msg) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp
        key = stamp_to_key(stamp)

        self.topic_stats[topic].update(stamp, now)
        self.first_seen_by_stamp[key][topic] = now

        seen = self.first_seen_by_stamp[key]
        for source, target in self.hop_stats:
            if source in seen and target in seen:
                self.hop_stats[(source, target)].add(seen[target] - seen[source], now)

        if len(self.first_seen_by_stamp) > self.args.max_stamps:
            oldest = next(iter(self.first_seen_by_stamp))
            self.first_seen_by_stamp.pop(oldest, None)

    def _tf_age(self, target: str, source: str) -> Tuple[Optional[float], Optional[str]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                target,
                source,
                RclTime(),
                timeout=Duration(seconds=self.args.tf_timeout),
            )
        except TransformException as exc:
            return None, str(exc)

        now = self.get_clock().now().nanoseconds * 1e-9
        return now - stamp_to_seconds(tf.header.stamp), None

    @staticmethod
    def average(values: Iterable[float]) -> Optional[float]:
        values = list(values)
        if not values:
            return None
        return sum(values) / len(values)

    def print_report(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        print("\n=== nav latency probe ===")
        print(f"node_now: {now:.6f}s")

        print(f"\nTopics: rolling window over last {self.args.window_sec:.2f}s")
        print(
            f"{'topic':46s} {'kind':12s} {'count':>7s} "
            f"{'last_ms':>9s} {'avg_ms':>9s} {'max_ms':>9s} {'rx_hz':>7s} {'stamp_hz':>8s}"
        )
        for topic in sorted(self.topic_stats):
            stats = self.topic_stats[topic]
            stats.prune(now)
            print(
                f"{topic:46s} {stats.kind:12s} {stats.count:7d} "
                f"{finite_ms(stats.last_age):>9s} "
                f"{finite_ms(stats.average_age):>9s} "
                f"{finite_ms(stats.max_age):>9s} "
                f"{finite_hz(stats.receive_hz):>7s} "
                f"{finite_hz(stats.header_hz):>8s}"
            )

        print("\nSame-stamp hop latency estimated from subscriber receive times")
        print(f"{'hop':74s} {'latest_ms':>10s} {'avg_ms':>10s} {'max_ms':>10s} {'n':>5s}")
        for (source, target), stats in self.hop_stats.items():
            stats.prune(now)
            hop_name = f"{source} -> {target}"
            print(
                f"{hop_name:74s} "
                f"{finite_ms(stats.latest):>10s} "
                f"{finite_ms(stats.average):>10s} "
                f"{finite_ms(stats.maximum):>10s} "
                f"{len(stats.samples):5d}"
            )

        print("\nTF latest-stamp age")
        print(f"{'tf':46s} {'last_ms':>9s} {'avg_ms':>9s} {'max_ms':>9s} {'n':>5s}  status")
        for target, source in self.args.tf:
            age, error = self._tf_age(target, source)
            samples = self.tf_age_samples[(target, source)]
            if age is not None:
                samples.append((now, age))
            prune_time_window(samples, now, self.args.window_sec)
            name = f"{target} <- {source}"
            status = "ok" if error is None else error[:100]
            avg = self.average(sample_values(samples))
            maximum = max(sample_values(samples)) if samples else None
            print(
                f"{name:46s} {finite_ms(age):>9s} {finite_ms(avg):>9s} "
                f"{finite_ms(maximum):>9s} {len(samples):5d}  {status}"
            )


def parse_pair(value: str) -> Tuple[str, str]:
    if ":" in value:
        left, right = value.split(":", 1)
    elif "," in value:
        left, right = value.split(",", 1)
    else:
        raise argparse.ArgumentTypeError("pair must be TARGET:SOURCE or SOURCE:TARGET for hops")
    left = left.strip()
    right = right.strip()
    if not left or not right:
        raise argparse.ArgumentTypeError("pair entries must not be empty")
    return left, right


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Measure header age, same-stamp hop latency, and TF age in the nav point-cloud chain."
    )
    parser.add_argument(
        "--pointcloud-topic",
        action="append",
        default=list(DEFAULT_POINTCLOUD_TOPICS),
        help="PointCloud2 topic to monitor. Can be repeated.",
    )
    parser.add_argument(
        "--scan-topic",
        action="append",
        default=list(DEFAULT_SCAN_TOPICS),
        help="LaserScan topic to monitor. Can be repeated.",
    )
    parser.add_argument(
        "--odom-topic",
        action="append",
        default=list(DEFAULT_ODOM_TOPICS),
        help="Odometry topic to monitor. Can be repeated.",
    )
    parser.add_argument(
        "--tf",
        action="append",
        type=parse_pair,
        default=list(DEFAULT_TF_PAIRS),
        metavar="TARGET:SOURCE",
        help="TF pair to monitor with lookup_transform(target, source). Can be repeated.",
    )
    parser.add_argument(
        "--hop",
        action="append",
        type=parse_pair,
        default=list(DEFAULT_HOPS),
        metavar="SOURCE:TARGET",
        help="Same-stamp topic hop to estimate. Can be repeated.",
    )
    parser.add_argument("--period", type=float, default=1.0, help="Print period in seconds.")
    parser.add_argument(
        "--window-sec",
        type=float,
        default=2.0,
        help="Rolling time window in seconds for topic, hop, and TF latency statistics.",
    )
    parser.add_argument("--tf-timeout", type=float, default=0.02, help="TF lookup timeout in seconds.")
    parser.add_argument("--tf-cache-sec", type=float, default=10.0, help="TF buffer cache seconds.")
    parser.add_argument("--max-stamps", type=int, default=500, help="Maximum stamp entries to keep.")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = NavLatencyProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
