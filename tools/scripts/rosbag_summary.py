#!/usr/bin/env python3
"""
rosbag_summary.py

rosbag (MCAP / SQLite3) の内容を解析し、トピック通信ヘルス、GNSS品質・精度、
オドメトリ、LiDAR、IMU、TF、エラーログなどの統計サマリーを出力するデバッグツール。
"""

import argparse
import datetime
import json
import math
import os
import sys
from collections import Counter, defaultdict
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from tf2_msgs.msg import TFMessage
from ublox_msgs.msg import NavPVT

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import detect_storage_id, load_metadata, open_reader  # noqa: E402
from tools.common.cli import resolve_output_path  # noqa: E402
from tools.common.geo import haversine_distance  # noqa: E402


def calculate_stats(values: List[float]) -> Dict[str, float]:
    """数値リストから主要な統計量を計算する。"""
    if not values:
        return {
            "min": 0.0,
            "max": 0.0,
            "mean": 0.0,
            "median": 0.0,
            "p90": 0.0,
            "p95": 0.0,
            "p99": 0.0,
            "std": 0.0,
            "count": 0,
        }
    arr = np.array(values, dtype=np.float64)
    return {
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "std": float(np.std(arr)),
        "count": len(values),
    }


def build_ascii_bar(ratio: float, max_len: int = 25) -> str:
    """割合 (0.0 - 1.0) から ASCII バー文字列を生成する。"""
    clamped = max(0.0, min(1.0, ratio))
    bar_len = int(round(clamped * max_len))
    return "█" * bar_len


class GeneralTopicCollector:
    """全トピック共通の通信健全性（件数、周波数、ギャップ、ドロップ警告）コレクター。"""

    def __init__(self):
        self.counts: Dict[str, int] = defaultdict(int)
        self.first_ts: Dict[str, int] = {}
        self.last_ts: Dict[str, int] = {}
        self.prev_ts: Dict[str, int] = {}
        self.intervals_s: Dict[str, List[float]] = defaultdict(list)
        self.max_gap_s: Dict[str, float] = defaultdict(float)
        self.drop_warnings: Dict[str, int] = defaultdict(int)

    def process(self, topic: str, timestamp_ns: int):
        self.counts[topic] += 1
        if topic not in self.first_ts:
            self.first_ts[topic] = timestamp_ns
        else:
            prev = self.prev_ts[topic]
            interval = (timestamp_ns - prev) / 1e9
            if interval > 0:
                self.intervals_s[topic].append(interval)
                if interval > self.max_gap_s[topic]:
                    self.max_gap_s[topic] = interval

        self.prev_ts[topic] = timestamp_ns
        self.last_ts[topic] = timestamp_ns

    def finalize(self) -> Dict[str, Any]:
        result = {}
        for topic, count in self.counts.items():
            first_s = self.first_ts[topic] / 1e9
            last_s = self.last_ts[topic] / 1e9
            duration_s = max(0.0, last_s - first_s)
            intervals = self.intervals_s[topic]
            avg_rate_hz = count / duration_s if duration_s > 0 else 0.0
            mean_interval = float(np.mean(intervals)) if intervals else 0.0

            # 想定間隔の3倍以上かつ50ms以上の遅延をドロップ警告とする
            threshold = max(0.05, mean_interval * 3.0)
            drops = sum(1 for iv in intervals if iv > threshold) if mean_interval > 0 else 0

            result[topic] = {
                "count": count,
                "duration_s": duration_s,
                "avg_rate_hz": avg_rate_hz,
                "mean_interval_s": mean_interval,
                "max_gap_s": self.max_gap_s[topic],
                "drop_warnings": drops,
            }
        return result


class GnssCollector:
    """GNSS (NavSatFix & NavPVT) 品質・精度・ヒストグラムコレクター。"""

    def __init__(self):
        self.has_data = False
        self.total_msgs = 0
        self.fix_statuses: Counter = Counter()
        self.carr_solutions: Counter = Counter()
        self.fix_types: Counter = Counter()

        self.last_rtk_fixed = False
        self.fixed_drop_events = 0

        self.h_acc_list: List[float] = []
        self.v_acc_list: List[float] = []
        self.num_sv_list: List[int] = []
        self.pdop_list: List[float] = []

        self.lats: List[float] = []
        self.lons: List[float] = []
        self.alts: List[float] = []
        self.prev_coord: Optional[Tuple[float, float]] = None
        self.trajectory_dist_m: float = 0.0

    def process_navpvt(self, msg: NavPVT):
        self.has_data = True
        self.total_msgs += 1

        carr_soln = (msg.flags >> 6) & 0b11
        # 0: None, 1: Float, 2: Fixed
        carr_names = {0: "None / Single", 1: "RTK Float", 2: "RTK Fixed"}
        self.carr_solutions[carr_names.get(carr_soln, f"Unknown({carr_soln})")] += 1

        fix_type_names = {
            0: "No Fix",
            1: "Dead Reckoning",
            2: "2D Fix",
            3: "3D Fix",
            4: "GNSS+DR",
            5: "Time Only",
        }
        self.fix_types[fix_type_names.get(msg.fix_type, f"Type({msg.fix_type})")] += 1

        # ドロップアウト検出 (Fixed -> それ以外への劣化)
        is_fixed = (carr_soln == 2)
        if self.total_msgs > 1 and self.last_rtk_fixed and not is_fixed:
            self.fixed_drop_events += 1
        self.last_rtk_fixed = is_fixed

        # 精度 (mm -> m)
        if msg.h_acc > 0:
            self.h_acc_list.append(msg.h_acc * 1e-3)
        if msg.v_acc > 0:
            self.v_acc_list.append(msg.v_acc * 1e-3)

        self.num_sv_list.append(msg.num_sv)
        if msg.p_dop > 0:
            self.pdop_list.append(msg.p_dop * 0.01)

        # 座標 (1e-7 deg)
        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7
        alt = msg.height * 1e-3
        if abs(lat) > 0.1 and abs(lon) > 0.1:
            self.lats.append(lat)
            self.lons.append(lon)
            self.alts.append(alt)
            if self.prev_coord is not None:
                d = haversine_distance(self.prev_coord[0], self.prev_coord[1], lat, lon)
                if d < 100.0:  # 物理的に妥当な変位のみ積算
                    self.trajectory_dist_m += d
            self.prev_coord = (lat, lon)

    def process_navsatfix(self, msg: NavSatFix):
        self.has_data = True
        # NavPVT がない場合のバックアップとして機能
        status_names = {
            -1: "No Fix",
            0: "Fix",
            1: "SBAS Fix",
            2: "GBAS/RTK Fix",
        }
        status_str = status_names.get(msg.status.status, f"Status({msg.status.status})")
        self.fix_statuses[status_str] += 1

        # 共分散からの水平・垂直精度算出
        if msg.position_covariance_type > 0:
            cov = msg.position_covariance
            var_e = cov[0]
            var_n = cov[4]
            var_u = cov[8]
            if var_e >= 0 and var_n >= 0:
                h_acc = math.sqrt(var_e + var_n)
                if not self.h_acc_list or len(self.h_acc_list) < self.total_msgs:
                    self.h_acc_list.append(h_acc)
            if var_u >= 0:
                v_acc = math.sqrt(var_u)
                if not self.v_acc_list or len(self.v_acc_list) < self.total_msgs:
                    self.v_acc_list.append(v_acc)

        if not self.lats and abs(msg.latitude) > 0.1:
            self.lats.append(msg.latitude)
            self.lons.append(msg.longitude)
            self.alts.append(msg.altitude)

    def get_accuracy_histogram(self) -> List[Dict[str, Any]]:
        """水平精度のテキストヒストグラムバケット集計を返す。"""
        if not self.h_acc_list:
            return []
        buckets = [
            ("< 0.05m (RTK Fixed)", 0.0, 0.05),
            ("0.05m - 0.10m", 0.05, 0.10),
            ("0.10m - 0.20m", 0.10, 0.20),
            ("0.20m - 0.50m", 0.20, 0.50),
            ("0.50m - 1.00m", 0.50, 1.00),
            ("> 1.00m", 1.00, float("inf")),
        ]
        total = len(self.h_acc_list)
        hist = []
        for label, low, high in buckets:
            count = sum(1 for v in self.h_acc_list if low <= v < high)
            ratio = count / total if total > 0 else 0.0
            hist.append({
                "label": label,
                "count": count,
                "ratio": ratio,
                "bar": build_ascii_bar(ratio),
            })
        return hist

    def finalize(self) -> Dict[str, Any]:
        if not self.has_data:
            return {"has_data": False}

        total = max(1, self.total_msgs or sum(self.fix_statuses.values()))
        fixed_count = self.carr_solutions.get("RTK Fixed", 0)
        float_count = self.carr_solutions.get("RTK Float", 0)
        gbas_count = self.fix_statuses.get("GBAS/RTK Fix", 0)

        rtk_fixed_rate = (fixed_count if fixed_count > 0 else gbas_count) / total * 100.0
        rtk_float_rate = float_count / total * 100.0

        lat_min = float(np.min(self.lats)) if self.lats else 0.0
        lat_max = float(np.max(self.lats)) if self.lats else 0.0
        lon_min = float(np.min(self.lons)) if self.lons else 0.0
        lon_max = float(np.max(self.lons)) if self.lons else 0.0

        return {
            "has_data": True,
            "total_msgs": total,
            "carr_solutions": dict(self.carr_solutions),
            "fix_types": dict(self.fix_types),
            "fix_statuses": dict(self.fix_statuses),
            "rtk_fixed_rate": rtk_fixed_rate,
            "rtk_float_rate": rtk_float_rate,
            "fixed_drop_events": self.fixed_drop_events,
            "h_acc_stats": calculate_stats(self.h_acc_list),
            "v_acc_stats": calculate_stats(self.v_acc_list),
            "h_acc_histogram": self.get_accuracy_histogram(),
            "num_sv_stats": calculate_stats([float(x) for x in self.num_sv_list]),
            "pdop_stats": calculate_stats(self.pdop_list),
            "trajectory_dist_m": self.trajectory_dist_m,
            "lat_range": (lat_min, lat_max),
            "lon_range": (lon_min, lon_max),
        }


class OdometryCollector:
    """オドメトリ (/odom) 走行距離・速度・共分散健全性コレクター。"""

    def __init__(self):
        self.has_data = False
        self.total_msgs = 0
        self.total_distance_m = 0.0
        self.prev_pos: Optional[Tuple[float, float]] = None
        self.linear_speeds: List[float] = []
        self.angular_speeds: List[float] = []
        self.stopped_count = 0
        self.zero_covariance_count = 0

    def process(self, msg: Odometry):
        self.has_data = True
        self.total_msgs += 1

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        if self.prev_pos is not None:
            dist = math.hypot(px - self.prev_pos[0], py - self.prev_pos[1])
            if dist < 5.0:  # 1ステップ5m以上の急激なジャンプは除外
                self.total_distance_m += dist
        self.prev_pos = (px, py)

        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.linear_speeds.append(abs(vx))
        self.angular_speeds.append(abs(wz))

        if abs(vx) < 0.01 and abs(wz) < 0.01:
            self.stopped_count += 1

        # 共分散が全て0かチェック
        pose_cov_zero = all(c == 0.0 for c in msg.pose.covariance)
        twist_cov_zero = all(c == 0.0 for c in msg.twist.covariance)
        if pose_cov_zero or twist_cov_zero:
            self.zero_covariance_count += 1

    def finalize(self) -> Dict[str, Any]:
        if not self.has_data:
            return {"has_data": False}
        total = max(1, self.total_msgs)
        stopped_rate = (self.stopped_count / total) * 100.0
        cov_valid = (self.zero_covariance_count == 0)

        return {
            "has_data": True,
            "total_msgs": total,
            "total_distance_m": self.total_distance_m,
            "linear_speed_stats": calculate_stats(self.linear_speeds),
            "angular_speed_stats": calculate_stats(self.angular_speeds),
            "stopped_rate": stopped_rate,
            "covariance_valid": cov_valid,
            "zero_covariance_count": self.zero_covariance_count,
        }


class LaserScanCollector:
    """LiDAR (/scan_*) 有効点率・Inf/NaN・最小障害物距離コレクター。"""

    def __init__(self):
        self.has_data = False
        self.total_msgs = 0
        self.total_rays = 0
        self.valid_rays = 0
        self.inf_rays = 0
        self.nan_rays = 0
        self.closest_range_m = float("inf")
        self.beam_counts: List[int] = []

    def process(self, msg: LaserScan):
        self.has_data = True
        self.total_msgs += 1
        num_rays = len(msg.ranges)
        self.beam_counts.append(num_rays)
        self.total_rays += num_rays

        for r in msg.ranges:
            if math.isnan(r):
                self.nan_rays += 1
            elif math.isinf(r):
                self.inf_rays += 1
            elif msg.range_min <= r <= msg.range_max:
                self.valid_rays += 1
                if r < self.closest_range_m:
                    self.closest_range_m = r

    def finalize(self) -> Dict[str, Any]:
        if not self.has_data:
            return {"has_data": False}
        total = max(1, self.total_rays)
        valid_rate = (self.valid_rays / total) * 100.0
        inf_rate = (self.inf_rays / total) * 100.0
        nan_rate = (self.nan_rays / total) * 100.0
        closest = self.closest_range_m if self.closest_range_m != float("inf") else 0.0

        return {
            "has_data": True,
            "total_msgs": self.total_msgs,
            "avg_beam_count": float(np.mean(self.beam_counts)) if self.beam_counts else 0,
            "valid_rate": valid_rate,
            "inf_rate": inf_rate,
            "nan_rate": nan_rate,
            "closest_range_m": closest,
        }


class ImuCollector:
    """IMU (*imu*) 3軸加速度・角速度・重力ノルム健全性コレクター。"""

    def __init__(self):
        self.has_data = False
        self.total_msgs = 0
        self.acc_norms: List[float] = []
        self.ang_vel_norms: List[float] = []
        self.nan_count = 0

    def process(self, msg: Imu):
        self.has_data = True
        self.total_msgs += 1

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        if any(math.isnan(v) for v in [ax, ay, az, wx, wy, wz]):
            self.nan_count += 1
            return

        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        ang_norm = math.sqrt(wx * wx + wy * wy + wz * wz)
        self.acc_norms.append(acc_norm)
        self.ang_vel_norms.append(ang_norm)

    def finalize(self) -> Dict[str, Any]:
        if not self.has_data:
            return {"has_data": False}
        return {
            "has_data": True,
            "total_msgs": self.total_msgs,
            "acc_norm_stats": calculate_stats(self.acc_norms),
            "ang_vel_stats": calculate_stats(self.ang_vel_norms),
            "nan_count": self.nan_count,
        }


class TfCollector:
    """TF (/tf, /tf_static) フレーム親子関係・配信周波数コレクター。"""

    def __init__(self):
        self.has_data = False
        self.pairs_count: Counter = Counter()
        self.static_pairs: Counter = Counter()

    def process(self, msg: TFMessage, is_static: bool):
        self.has_data = True
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            pair = f"{parent} -> {child}"
            if is_static:
                self.static_pairs[pair] += 1
            else:
                self.pairs_count[pair] += 1

    def finalize(self) -> Dict[str, Any]:
        if not self.has_data:
            return {"has_data": False}
        # 主要フレームの存在チェック
        all_frames = set()
        for p in list(self.pairs_count.keys()) + list(self.static_pairs.keys()):
            parts = p.split(" -> ")
            all_frames.update(parts)

        critical_frames = ["map", "odom", "base_link", "base_footprint"]
        found_critical = {f: (f in all_frames) for f in critical_frames}

        return {
            "has_data": True,
            "dynamic_pairs": dict(self.pairs_count.most_common(20)),
            "static_pairs": dict(self.static_pairs.most_common(20)),
            "critical_frames": found_critical,
        }


class DiagnosticsCollector:
    """ログ (/rosout) & 診断 (/diagnostics) エラー収集コレクター。"""

    def __init__(self):
        self.has_data = False
        self.log_level_counts: Counter = Counter()
        self.error_messages: List[Dict[str, str]] = []

    def process_log(self, msg: Log):
        self.has_data = True
        level_map = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = level_map.get(msg.level, f"LEVEL_{msg.level}")
        self.log_level_counts[level_str] += 1

        if msg.level >= 40:  # ERROR or FATAL
            if len(self.error_messages) < 10:
                self.error_messages.append({
                    "name": msg.name,
                    "level": level_str,
                    "msg": msg.msg.strip(),
                })

    def process_diagnostics(self, msg: DiagnosticArray):
        self.has_data = True
        for status in msg.status:
            if status.level >= 2:  # WARN or ERROR
                if len(self.error_messages) < 10:
                    self.error_messages.append({
                        "name": f"diag:{status.name}",
                        "level": "ERROR" if status.level >= 2 else "WARN",
                        "msg": status.message,
                    })

    def finalize(self) -> Dict[str, Any]:
        if not self.has_data:
            return {"has_data": False}
        return {
            "has_data": True,
            "log_levels": dict(self.log_level_counts),
            "errors": self.error_messages,
        }


class ReportFormatter:
    """集計結果をターミナル、Markdown、JSON向けに整形するフォーマッタ。"""

    @staticmethod
    def format_terminal(data: Dict[str, Any], use_color: bool = True, show_all: bool = False) -> str:
        # ANSIカラー定義
        G = "\033[32m" if use_color else ""
        Y = "\033[33m" if use_color else ""
        R = "\033[31m" if use_color else ""
        C = "\033[36m" if use_color else ""
        B = "\033[1m" if use_color else ""
        RESET = "\033[0m" if use_color else ""

        lines = []
        meta = data["metadata"]
        lines.append(f"{B}{'=' * 80}{RESET}")
        lines.append(f"{B}ROSBAG SUMMARY REPORT{RESET}")
        lines.append(f"{B}{'=' * 80}{RESET}")
        lines.append(f"Bag Path:      {meta['path']}")
        lines.append(f"Storage ID:    {C}{meta['storage_id']}{RESET}")
        duration_fmt = str(datetime.timedelta(seconds=int(meta['duration_s'])))
        lines.append(f"Duration:      {meta['duration_s']:.2f} s ({duration_fmt})")
        lines.append(f"Start Time:    {meta['start_time']}")
        lines.append(f"End Time:      {meta['end_time']}")
        lines.append(
            f"Total Topics:  {meta['total_topics']} | Total Messages: {meta['total_messages']:,}"
        )
        lines.append("")

        # 1. Topic Overview
        lines.append(f"{B}{'-' * 80}{RESET}")
        lines.append(f"{B}1. TOPIC OVERVIEW & HEALTH{RESET}")
        lines.append(f"{B}{'-' * 80}{RESET}")
        lines.append(
            f"{'Topic Name':<32} {'Type':<26} {'Count':>7} {'Rate(Hz)':>9} {'Max Gap(s)':>11} {'Drops':>6}"
        )
        lines.append("-" * 95)
        for t_info in data["topics"]:
            t_name = t_info["name"]
            t_type = t_info["type"].split("/")[-1]
            cnt = t_info["count"]
            rate = f"{t_info['avg_rate_hz']:.2f}" if t_info["count"] > 1 else "---"
            gap = f"{t_info['max_gap_s']:.3f}" if t_info["max_gap_s"] > 0 else "---"
            drops = str(t_info["drop_warnings"])
            drop_color = R if t_info["drop_warnings"] > 0 else ""
            lines.append(
                f"{t_name:<32} {t_type:<26} {cnt:>7} {rate:>9} {gap:>11} {drop_color}{drops:>6}{RESET}"
            )
        lines.append("")

        # 2. GNSS Summary
        gnss = data.get("gnss", {})
        if gnss.get("has_data"):
            lines.append(f"{B}{'-' * 80}{RESET}")
            lines.append(f"{B}2. GNSS QUALITY & ACCURACY (/gps/fix & /navpvt){RESET}")
            lines.append(f"{B}{'-' * 80}{RESET}")

            # Fix Distribution
            lines.append(f"{B}[Fix Status Distribution]{RESET}")
            total_gnss = gnss["total_msgs"]
            for name, count in gnss["carr_solutions"].items():
                ratio = count / total_gnss if total_gnss > 0 else 0.0
                bar = build_ascii_bar(ratio, max_len=30)
                color = G if "Fixed" in name else (Y if "Float" in name else R)
                lines.append(f"  {name:<24}: {count:>6} msgs ({ratio * 100:>5.1f}%) |{color}{bar}{RESET}")

            if gnss["fix_statuses"] and not gnss["carr_solutions"]:
                for name, count in gnss["fix_statuses"].items():
                    ratio = count / total_gnss if total_gnss > 0 else 0.0
                    bar = build_ascii_bar(ratio, max_len=30)
                    color = G if "GBAS" in name else Y
                    lines.append(f"  {name:<24}: {count:>6} msgs ({ratio * 100:>5.1f}%) |{color}{bar}{RESET}")

            fix_color = G if gnss["rtk_fixed_rate"] >= 90 else (Y if gnss["rtk_fixed_rate"] >= 50 else R)
            lines.append(
                f"  --> Overall RTK Fixed Rate: {fix_color}{gnss['rtk_fixed_rate']:.1f}%{RESET} "
                f"(Float+Fixed: {gnss['rtk_fixed_rate'] + gnss['rtk_float_rate']:.1f}%)"
            )
            if gnss["fixed_drop_events"] > 0:
                lines.append(
                    f"  --> {R}Fix Degradation Events (Fixed -> Float/NoFix): {gnss['fixed_drop_events']} times{RESET}"
                )
            lines.append("")

            # Accuracy Statistics
            h_stat = gnss["h_acc_stats"]
            v_stat = gnss["v_acc_stats"]
            lines.append(f"{B}[Accuracy Statistics (meters)]{RESET}")
            lines.append(f"  {'Metric':<14} {'Min':>8} {'Max':>8} {'Mean':>8} {'Median':>8} {'95%':>8} {'Std':>8}")
            lines.append("  " + "-" * 62)
            lines.append(
                f"  {'Horizontal':<14} {h_stat['min']:>7.3f}m {h_stat['max']:>7.3f}m "
                f"{h_stat['mean']:>7.3f}m {h_stat['median']:>7.3f}m {h_stat['p95']:>7.3f}m {h_stat['std']:>7.3f}m"
            )
            lines.append(
                f"  {'Vertical':<14} {v_stat['min']:>7.3f}m {v_stat['max']:>7.3f}m "
                f"{v_stat['mean']:>7.3f}m {v_stat['median']:>7.3f}m {v_stat['p95']:>7.3f}m {v_stat['std']:>7.3f}m"
            )
            lines.append("")

            # Histogram
            if gnss["h_acc_histogram"]:
                lines.append(f"{B}[Horizontal Accuracy Histogram]{RESET}")
                for b in gnss["h_acc_histogram"]:
                    color = G if "< 0.05" in b["label"] else (Y if "0.05" in b["label"] or "0.10" in b["label"] else R)
                    lines.append(
                        f"  {b['label']:<24}: {b['count']:>6} ({b['ratio'] * 100:>5.1f}%) |{color}{b['bar']}{RESET}"
                    )
                lines.append("")

            # Satellites & DOP
            num_sv = gnss["num_sv_stats"]
            pdop = gnss["pdop_stats"]
            if num_sv["count"] > 0:
                lines.append(f"{B}[Satellites & DOP]{RESET}")
                lines.append(
                    f"  Satellites (numSV):   Min: {int(num_sv['min'])}, Max: {int(num_sv['max'])}, "
                    f"Mean: {num_sv['mean']:.1f}, Median: {num_sv['median']:.1f}"
                )
                if pdop["count"] > 0:
                    lines.append(
                        f"  pDOP:                 Min: {pdop['min']:.2f}, Max: {pdop['max']:.2f}, "
                        f"Mean: {pdop['mean']:.2f}, Median: {pdop['median']:.2f}"
                    )
                lines.append("")

            # Coverage
            lat_r = gnss["lat_range"]
            lon_r = gnss["lon_range"]
            if lat_r[0] != 0.0:
                lines.append(f"{B}[Geographic Trajectory]{RESET}")
                lines.append(f"  Latitude:  [{lat_r[0]:.6f}, {lat_r[1]:.6f}]")
                lines.append(f"  Longitude: [{lon_r[0]:.6f}, {lon_r[1]:.6f}]")
                lines.append(f"  Estimated Trajectory Distance: {gnss['trajectory_dist_m']:.1f} m")
                lines.append("")

        # 3. Odometry
        odom = data.get("odometry", {})
        if odom.get("has_data") and (show_all or not gnss.get("has_data")):
            lines.append(f"{B}{'-' * 80}{RESET}")
            lines.append(f"{B}3. ODOMETRY & MOTION (/odom){RESET}")
            lines.append(f"{B}{'-' * 80}{RESET}")
            lines.append(f"  Estimated Distance:     {odom['total_distance_m']:.2f} m")
            lin = odom["linear_speed_stats"]
            ang = odom["angular_speed_stats"]
            lines.append(
                f"  Linear Speed (vx):      Max: {lin['max']:.2f} m/s | Mean: {lin['mean']:.2f} m/s | "
                f"Stopped Time: {odom['stopped_rate']:.1f}%"
            )
            lines.append(
                f"  Angular Speed (wz):     Max: {ang['max']:.2f} rad/s ({math.degrees(ang['max']):.1f} deg/s) | "
                f"Mean: {ang['mean']:.2f} rad/s"
            )
            cov_status = (
                f"{G}VALID (Configured non-zero){RESET}"
                if odom["covariance_valid"]
                else f"{R}WARNING: All zeros detected{RESET}"
            )
            lines.append(f"  Covariance Sanity:      {cov_status}")
            lines.append("")

        # 4. LiDAR
        scan = data.get("lidar", {})
        if scan.get("has_data") and show_all:
            lines.append(f"{B}{'-' * 80}{RESET}")
            lines.append(f"{B}4. LIDAR QUALITY (/scan_*){RESET}")
            lines.append(f"{B}{'-' * 80}{RESET}")
            lines.append(f"  Average Beam Count:     {scan['avg_beam_count']:.0f} beams/scan")
            lines.append(
                f"  Valid Points:           {G}{scan['valid_rate']:.1f}%{RESET} | "
                f"Inf (Out of range): {scan['inf_rate']:.1f}% | "
                f"NaN (Invalid): {R if scan['nan_rate'] > 0 else ''}{scan['nan_rate']:.1f}%{RESET}"
            )
            lines.append(f"  Closest Obstacle:       {scan['closest_range_m']:.2f} m")
            lines.append("")

        # 5. IMU
        imu = data.get("imu", {})
        if imu.get("has_data") and show_all:
            lines.append(f"{B}{'-' * 80}{RESET}")
            lines.append(f"{B}5. IMU INTEGRITY (*imu*){RESET}")
            lines.append(f"{B}{'-' * 80}{RESET}")
            acc = imu["acc_norm_stats"]
            lines.append(
                f"  Gravity Norm (|a|):     Mean: {acc['mean']:.2f} m/s² (Target: ~9.81) | "
                f"Min: {acc['min']:.2f} | Max: {acc['max']:.2f}"
            )
            nan_status = f"{R}{imu['nan_count']} detected{RESET}" if imu["nan_count"] > 0 else f"{G}None{RESET}"
            lines.append(f"  NaN Values:             {nan_status}")
            lines.append("")

        # 6. TF
        tf = data.get("tf", {})
        if tf.get("has_data") and show_all:
            lines.append(f"{B}{'-' * 80}{RESET}")
            lines.append(f"{B}6. TF FRAMES & HEALTH (/tf, /tf_static){RESET}")
            lines.append(f"{B}{'-' * 80}{RESET}")
            crit = tf["critical_frames"]
            crit_strs = [f"{k}: {G+'OK'+RESET if v else R+'MISSING'+RESET}" for k, v in crit.items()]
            lines.append("  Critical Frames: " + " | ".join(crit_strs))
            lines.append("  Top Dynamic Transform Pairs:")
            for pair, count in list(tf["dynamic_pairs"].items())[:8]:
                lines.append(f"    - {pair:<35} ({count} msgs)")
            lines.append("")

        # 7. Diagnostics & Logs
        diag = data.get("diagnostics", {})
        if diag.get("has_data"):
            levels = diag.get("log_levels", {})
            has_errors = levels.get("ERROR", 0) > 0 or levels.get("FATAL", 0) > 0
            if has_errors or show_all:
                lines.append(f"{B}{'-' * 80}{RESET}")
                lines.append(f"{B}7. SYSTEM LOGS & DIAGNOSTICS (/rosout, /diagnostics){RESET}")
                lines.append(f"{B}{'-' * 80}{RESET}")
                lvl_str = " | ".join([f"{k}: {v}" for k, v in levels.items()])
                lines.append(f"  Log Levels: {lvl_str}")
                if diag.get("errors"):
                    lines.append(f"  {R}Recent Error / Warning Messages:{RESET}")
                    for err in diag["errors"][:5]:
                        lines.append(f"    - [{err['level']}] {err['name']}: {err['msg']}")
                lines.append("")

        lines.append(f"{B}{'=' * 80}{RESET}")
        return "\n".join(lines)

    @staticmethod
    def format_markdown(data: Dict[str, Any]) -> str:
        md = []
        meta = data["metadata"]
        md.append("# Rosbag Summary Report")
        md.append("")
        md.append("| Property | Value |")
        md.append("| :--- | :--- |")
        md.append(f"| **Bag Path** | `{meta['path']}` |")
        md.append(f"| **Storage ID** | `{meta['storage_id']}` |")
        md.append(f"| **Duration** | {meta['duration_s']:.2f} s |")
        md.append(f"| **Start Time** | {meta['start_time']} |")
        md.append(f"| **End Time** | {meta['end_time']} |")
        md.append(f"| **Total Topics** | {meta['total_topics']} |")
        md.append(f"| **Total Messages** | {meta['total_messages']:,} |")
        md.append("")

        # Topics
        md.append("## 1. Topic Overview & Health")
        md.append("")
        md.append("| Topic Name | Type | Count | Rate (Hz) | Max Gap (s) | Drop Warnings |")
        md.append("| :--- | :--- | :---: | :---: | :---: | :---: |")
        for t in data["topics"]:
            rate = f"{t['avg_rate_hz']:.2f}" if t["count"] > 1 else "---"
            gap = f"{t['max_gap_s']:.3f}" if t["max_gap_s"] > 0 else "---"
            md.append(f"| `{t['name']}` | `{t['type']}` | {t['count']} | {rate} | {gap} | {t['drop_warnings']} |")
        md.append("")

        # GNSS
        gnss = data.get("gnss", {})
        if gnss.get("has_data"):
            md.append("## 2. GNSS Quality & Accuracy")
            md.append("")
            md.append(f"- **Overall RTK Fixed Rate**: **{gnss['rtk_fixed_rate']:.1f}%**")
            md.append(f"- **RTK Float Rate**: {gnss['rtk_float_rate']:.1f}%")
            md.append(f"- **Fix Degradation Events**: {gnss['fixed_drop_events']} times")
            if gnss.get("trajectory_dist_m"):
                md.append(f"- **Estimated Trajectory Distance**: {gnss['trajectory_dist_m']:.1f} m")
            md.append("")

            # Accuracy table
            h = gnss["h_acc_stats"]
            v = gnss["v_acc_stats"]
            md.append("### Accuracy Statistics")
            md.append("| Metric | Min | Max | Mean | Median | 95% | Std |")
            md.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: |")
            md.append(
                f"| Horizontal | {h['min']:.3f} m | {h['max']:.3f} m | {h['mean']:.3f} m | "
                f"{h['median']:.3f} m | {h['p95']:.3f} m | {h['std']:.3f} m |"
            )
            md.append(
                f"| Vertical | {v['min']:.3f} m | {v['max']:.3f} m | {v['mean']:.3f} m | "
                f"{v['median']:.3f} m | {v['p95']:.3f} m | {v['std']:.3f} m |"
            )
            md.append("")

            # Histogram
            if gnss["h_acc_histogram"]:
                md.append("### Horizontal Accuracy Histogram")
                md.append("| Range | Count | Ratio | Distribution |")
                md.append("| :--- | :---: | :---: | :--- |")
                for b in gnss["h_acc_histogram"]:
                    md.append(f"| {b['label']} | {b['count']} | {b['ratio']*100:.1f}% | `{b['bar']}` |")
                md.append("")

        # Odometry
        odom = data.get("odometry", {})
        if odom.get("has_data"):
            md.append("## 3. Odometry Summary")
            md.append("")
            md.append(f"- **Total Distance**: {odom['total_distance_m']:.2f} m")
            md.append(f"- **Max Linear Velocity**: {odom['linear_speed_stats']['max']:.2f} m/s")
            md.append(f"- **Mean Linear Velocity**: {odom['linear_speed_stats']['mean']:.2f} m/s")
            md.append(f"- **Stopped Time**: {odom['stopped_rate']:.1f}%")
            md.append(f"- **Covariance Valid**: {'YES' if odom['covariance_valid'] else 'NO (Zero Detected)'}")
            md.append("")

        return "\n".join(md)

    @staticmethod
    def format_json(data: Dict[str, Any]) -> str:
        return json.dumps(data, indent=2, ensure_ascii=False, default=str)


def parse_args():
    parser = argparse.ArgumentParser(
        description="rosbag (MCAP / SQLite3) の統計サマリーを出力するデバッグツール"
    )
    parser.add_argument("bag_path", type=str, help="rosbagディレクトリまたはファイルパス")
    parser.add_argument(
        "-a", "--all", action="store_true", help="Odom, LiDAR, IMU, TF, ログ診断まで全詳細を表示"
    )
    parser.add_argument(
        "-t", "--topics", nargs="+", default=None, help="走査するトピック名を限定（スペース区切り）"
    )
    parser.add_argument(
        "-f",
        "--format",
        choices=["text", "markdown", "json"],
        default="text",
        help="出力フォーマット (デフォルト: text)",
    )
    parser.add_argument(
        "-o", "--output", type=str, default=None, help="出力ファイルパス（指定時はファイルに保存）"
    )
    parser.add_argument(
        "--output-to-bag-dir",
        action="store_true",
        help="bagと同じディレクトリに summary.md を保存し、端末にもサマリーを表示",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="指定したディレクトリに summary.md を保存し、端末にもサマリーを表示",
    )
    parser.add_argument(
        "--info-only",
        action="store_true",
        help="メッセージ走査を行わず、メタ情報のみを高速出力",
    )
    parser.add_argument(
        "--no-color", action="store_true", help="ターミナル出力時のANSIカラーを無効化"
    )
    parser.add_argument(
        "--storage-id",
        type=str,
        default=None,
        choices=["mcap", "sqlite3"],
        help="ストレージ形式の手動指定（通常は自動判定）",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    bag_path = os.path.abspath(args.bag_path)

    if not os.path.exists(bag_path):
        print(f"エラー: 指定されたパスが存在しません: {bag_path}", file=sys.stderr)
        sys.exit(1)

    storage_id = args.storage_id or detect_storage_id(bag_path)

    try:
        reader = open_reader(bag_path, storage_id=storage_id)
    except Exception as e:
        print(f"エラー: rosbagを開けませんでした: {e}", file=sys.stderr)
        sys.exit(1)

    all_topics = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in all_topics}

    # metadata.yaml からの事前情報取得
    meta = load_metadata(bag_path)
    meta_info = meta.get("rosbag2_bagfile_information", {}) if meta else {}
    start_ns = meta_info.get("starting_time", {}).get("nanoseconds_since_epoch", 0)
    duration_ns = meta_info.get("duration", {}).get("nanoseconds", 0)
    total_msgs = meta_info.get("message_count", 0)

    start_dt = (
        datetime.datetime.fromtimestamp(start_ns / 1e9)
        if start_ns > 0
        else datetime.datetime.now()
    )
    end_dt = (
        datetime.datetime.fromtimestamp((start_ns + duration_ns) / 1e9)
        if start_ns > 0
        else start_dt
    )

    # コレクターの初期化
    topic_collector = GeneralTopicCollector()
    gnss_collector = GnssCollector()
    odom_collector = OdometryCollector()
    laser_collector = LaserScanCollector()
    imu_collector = ImuCollector()
    tf_collector = TfCollector()
    diag_collector = DiagnosticsCollector()

    filter_topics = set(args.topics) if args.topics else None

    # メッセージ走査
    if not args.info_only:
        msg_type_cache: Dict[str, Any] = {}
        first_bag_ts = None
        last_bag_ts = None
        actual_total_msgs = 0

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if filter_topics and topic not in filter_topics:
                continue

            actual_total_msgs += 1
            if first_bag_ts is None:
                first_bag_ts = timestamp
            last_bag_ts = timestamp

            topic_collector.process(topic, timestamp)

            type_name = topic_type_map.get(topic)
            if not type_name:
                continue

            # 型解決とデシリアライズ
            if type_name not in msg_type_cache:
                msg_type_cache[type_name] = get_message(type_name)
            msg_class = msg_type_cache[type_name]

            # 主要トピックの仕分け
            if type_name == "ublox_msgs/msg/NavPVT":
                msg = deserialize_message(data, msg_class)
                gnss_collector.process_navpvt(msg)
            elif type_name == "sensor_msgs/msg/NavSatFix":
                msg = deserialize_message(data, msg_class)
                gnss_collector.process_navsatfix(msg)
            elif type_name == "nav_msgs/msg/Odometry":
                msg = deserialize_message(data, msg_class)
                odom_collector.process(msg)
            elif type_name == "sensor_msgs/msg/LaserScan":
                msg = deserialize_message(data, msg_class)
                laser_collector.process(msg)
            elif type_name == "sensor_msgs/msg/Imu":
                msg = deserialize_message(data, msg_class)
                imu_collector.process(msg)
            elif type_name == "tf2_msgs/msg/TFMessage":
                msg = deserialize_message(data, msg_class)
                tf_collector.process(msg, is_static=(topic == "/tf_static"))
            elif type_name == "rcl_interfaces/msg/Log":
                msg = deserialize_message(data, msg_class)
                diag_collector.process_log(msg)
            elif type_name == "diagnostic_msgs/msg/DiagnosticArray":
                msg = deserialize_message(data, msg_class)
                diag_collector.process_diagnostics(msg)

        if first_bag_ts is not None and last_bag_ts is not None:
            start_dt = datetime.datetime.fromtimestamp(first_bag_ts / 1e9)
            end_dt = datetime.datetime.fromtimestamp(last_bag_ts / 1e9)
            duration_ns = last_bag_ts - first_bag_ts
            total_msgs = actual_total_msgs

    # サマリーデータの構築
    topic_summary = topic_collector.finalize()
    meta_topic_counts = {}
    if meta_info.get("topics_with_message_count"):
        for twmc in meta_info["topics_with_message_count"]:
            t_meta_name = twmc.get("topic_metadata", {}).get("name")
            m_cnt = twmc.get("message_count", 0)
            if t_meta_name:
                meta_topic_counts[t_meta_name] = m_cnt

    topics_list = []
    dur_sec = duration_ns / 1e9 if duration_ns > 0 else 0.0
    for t in all_topics:
        t_name = t.name
        t_stat = topic_summary.get(t_name)
        if t_stat is not None and t_stat["count"] > 0:
            cnt = t_stat["count"]
            rate = t_stat["avg_rate_hz"]
            max_gap = t_stat["max_gap_s"]
            drops = t_stat["drop_warnings"]
        elif t_name in meta_topic_counts:
            cnt = meta_topic_counts[t_name]
            rate = cnt / dur_sec if dur_sec > 0 else 0.0
            max_gap = 0.0
            drops = 0
        else:
            cnt = 0
            rate = 0.0
            max_gap = 0.0
            drops = 0

        topics_list.append({
            "name": t_name,
            "type": t.type,
            "count": cnt,
            "avg_rate_hz": rate,
            "max_gap_s": max_gap,
            "drop_warnings": drops,
        })

    # トピック名順でソート
    topics_list.sort(key=lambda x: x["name"])

    report_data = {
        "metadata": {
            "path": bag_path,
            "storage_id": storage_id,
            "duration_s": duration_ns / 1e9,
            "start_time": start_dt.strftime("%Y-%m-%d %H:%M:%S"),
            "end_time": end_dt.strftime("%Y-%m-%d %H:%M:%S"),
            "total_topics": len(all_topics),
            "total_messages": total_msgs,
        },
        "topics": topics_list,
        "gnss": gnss_collector.finalize(),
        "odometry": odom_collector.finalize(),
        "lidar": laser_collector.finalize(),
        "imu": imu_collector.finalize(),
        "tf": tf_collector.finalize(),
        "diagnostics": diag_collector.finalize(),
    }

    target_output_file = None
    if args.output_to_bag_dir or args.output_dir or args.output:
        target_output_file = resolve_output_path(
            bag_path,
            default_filename="summary.md",
            output=args.output,
            output_to_bag_dir=args.output_to_bag_dir,
            output_dir=args.output_dir,
        )

    # 出力生成
    if args.format == "json":
        formatted = ReportFormatter.format_json(report_data)
    elif args.format == "markdown":
        formatted = ReportFormatter.format_markdown(report_data)
    else:
        formatted = ReportFormatter.format_terminal(
            report_data, use_color=(not args.no_color), show_all=args.all
        )

    if target_output_file:
        save_content = formatted
        if (args.output_to_bag_dir or args.output_dir) and args.format == "text":
            save_content = ReportFormatter.format_markdown(report_data)

        target_dir = os.path.dirname(target_output_file)
        if target_dir:
            os.makedirs(target_dir, exist_ok=True)

        with open(target_output_file, "w", encoding="utf-8") as f:
            f.write(save_content + "\n")
        print(f"サマリーを保存しました: {target_output_file}")

        if args.output_to_bag_dir or args.output_dir:
            print(formatted)
    else:
        print(formatted)


if __name__ == "__main__":
    main()
