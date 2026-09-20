#!/usr/bin/env python3
"""
plot_gnss_trajectory.py

rosbag (MCAP / SQLite3) から GNSS 軌跡 (NavPVT / NavSatFix) を抽出し、
測位品質 (RTK Fix / Float / Single) や精度誤差円、オドメトリ軌跡、
および SLAM 等の地図画像 (OccupancyGrid) との重ね合わせプロットを生成するツール。
"""

import argparse
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as patches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from ublox_msgs.msg import NavPVT  # noqa: E402

# tools パッケージルートの解決
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from tools.common.bag import open_reader  # noqa: E402
from tools.common.cli import add_output_args, resolve_output_path  # noqa: E402
from tools.common.geo import lat_lon_to_utm, quaternion_to_yaw  # noqa: E402
from tools.common.map import load_map_info, load_transform_from_yaml  # noqa: E402


def extract_bag_data(
    bag_path: str,
    utm_zone: int = 54,
    start_sec: Optional[float] = None,
    end_sec: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    rosbag から GNSS および Odometry データを抽出して返す。
    Returns:
      gnss_arr: shape (N, 7) -> [t_rel, easting, northing, status, h_acc, lat, lon]
                status: 2=RTK Fix, 1=RTK Float, 0=Single/3D, -1=NoFix
      odom_arr: shape (M, 4) -> [t_rel, x, y, yaw]
    """
    reader = open_reader(bag_path)

    topics_and_types = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in topics_and_types}

    gnss_records = []
    odom_records = []

    base_time_sec = None

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t_sec = stamp * 1e-9
        if base_time_sec is None:
            base_time_sec = t_sec

        t_rel = t_sec - base_time_sec
        if start_sec is not None and t_rel < start_sec:
            continue
        if end_sec is not None and t_rel > end_sec:
            break

        type_name = topic_type_map.get(topic)
        if not type_name:
            continue

        if type_name == "ublox_msgs/msg/NavPVT":
            msg = deserialize_message(data, NavPVT)
            lat = msg.lat * 1e-7
            lon = msg.lon * 1e-7
            easting, northing = lat_lon_to_utm(lat, lon, zone=utm_zone)
            carrier = (msg.flags >> 6) & 0x03
            if carrier == 2:
                status = 2  # RTK Fix
            elif carrier == 1:
                status = 1  # RTK Float
            elif msg.fix_type == 3:
                status = 0  # 3D Fix
            else:
                status = -1  # No Fix / 2D

            h_acc = msg.h_acc * 1e-3  # mm -> m
            gnss_records.append(
                (t_rel, easting, northing, status, h_acc, lat, lon))

        elif type_name == "sensor_msgs/msg/NavSatFix" and "/navpvt" not in topic_type_map:
            # NavPVT がない場合のバックアップ
            msg = deserialize_message(data, NavSatFix)
            if abs(msg.latitude) > 0.1 and abs(msg.longitude) > 0.1:
                easting, northing = lat_lon_to_utm(
                    msg.latitude, msg.longitude, zone=utm_zone)
                raw_st = msg.status.status
                if raw_st == 2:
                    status = 2  # GBAS / RTK Fix
                elif raw_st == 1:
                    status = 1  # SBAS / Float
                elif raw_st == 0:
                    status = 0  # Fix
                else:
                    status = -1  # No Fix

                h_acc = 0.0
                if msg.position_covariance_type > 0:
                    var_e = msg.position_covariance[0]
                    var_n = msg.position_covariance[4]
                    if var_e >= 0 and var_n >= 0:
                        h_acc = math.sqrt(var_e + var_n)

                gnss_records.append(
                    (t_rel, easting, northing, status,
                     h_acc, msg.latitude, msg.longitude)
                )

        elif type_name == "nav_msgs/msg/Odometry" and topic == "/odom":
            msg = deserialize_message(data, Odometry)
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            ori = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            odom_records.append((t_rel, px, py, yaw))

    gnss_arr = (
        np.array(gnss_records, dtype=np.float64)
        if gnss_records
        else np.empty((0, 7), dtype=np.float64)
    )
    odom_arr = (
        np.array(odom_records, dtype=np.float64)
        if odom_records
        else np.empty((0, 4), dtype=np.float64)
    )
    return gnss_arr, odom_arr


def print_status_periods(gnss: np.ndarray):
    """GNSSステータス（Fix, Float, Single）の推移区間サマリーを表示する。"""
    if len(gnss) == 0:
        print("GNSSデータが存在しません。")
        return

    status_labels = {-1: "No Fix", 0: "Single/3D",
                     1: "RTK Float", 2: "RTK Fix"}
    print("\n" + "=" * 70)
    print("GNSS FIX STATUS TIMELINE PERIODS")
    print("=" * 70)
    print(
        f"{'Period [sec]':<18} {'Status':<12} {'Count':>6} {'Mean hAcc [m]':>15}")
    print("-" * 70)

    prev_st = int(gnss[0, 3])
    start_idx = 0
    for i in range(1, len(gnss)):
        st = int(gnss[i, 3])
        if st != prev_st:
            t_start = gnss[start_idx, 0]
            t_end = gnss[i - 1, 0]
            count = i - start_idx
            mean_acc = float(np.mean(gnss[start_idx:i, 4]))
            lbl = status_labels.get(prev_st, f"Status({prev_st})")
            print(
                f"{t_start:6.1f}s - {t_end:6.1f}s    {lbl:<12} {count:>6d} {mean_acc:>14.3f}m")
            prev_st = st
            start_idx = i

    t_start = gnss[start_idx, 0]
    t_end = gnss[-1, 0]
    count = len(gnss) - start_idx
    mean_acc = float(np.mean(gnss[start_idx:, 4]))
    lbl = status_labels.get(prev_st, f"Status({prev_st})")
    print(f"{t_start:6.1f}s - {t_end:6.1f}s    {lbl:<12} {count:>6d} {mean_acc:>14.3f}m")
    print("=" * 70 + "\n")


def add_accuracy_circles(
    ax,
    x: np.ndarray,
    y: np.ndarray,
    accuracies: np.ndarray,
    statuses: np.ndarray,
    step: Optional[int] = None,
    scale: float = 1.0,
    max_radius: float = 20.0,
    alpha: float = 0.25,
):
    """GNSS精度円 (NavPVT hAcc) を軸に描画する共通ヘルパー関数。"""
    n_points = len(x)
    if n_points == 0:
        return

    if step is None:
        step = max(1, n_points // 40)
    step = max(1, step)

    label_text = (
        f"Accuracy Circle (hAcc ×{scale:g})"
        if scale != 1.0
        else "Accuracy Circle (hAcc)"
    )

    legend_added = False
    for i in range(0, n_points, step):
        acc = accuracies[i]
        if not (0.0 < acc <= max_radius):
            continue

        draw_radius = acc * scale

        status = statuses[i]
        if status == 2:
            color = "green"
        elif status == 1:
            color = "orange"
        elif status == 0:
            color = "gray"
        else:
            color = "red"

        circle = patches.Circle(
            (x[i], y[i]),
            draw_radius,
            fill=False,
            edgecolor=color,
            linestyle="--",
            linewidth=1.0,
            alpha=alpha,
            label=label_text if not legend_added else None,
        )
        ax.add_patch(circle)
        legend_added = True


def plot_gnss_only(
    gnss: np.ndarray,
    output_path: str,
    show_accuracy_circles: bool = False,
    circle_step: Optional[int] = None,
    circle_scale: float = 1.0,
    max_circle_radius: float = 20.0,
    circle_alpha: float = 0.25,
    point_size: float = 1.0,
    transparent: bool = False,
    title: str = "GNSS UTM Trajectory",
):
    """GNSS単体のUTM軌跡をプロットする。"""
    fig, ax = plt.subplots(figsize=(10, 10))

    fix_mask = gnss[:, 3] == 2
    float_mask = gnss[:, 3] == 1
    single_mask = gnss[:, 3] == 0
    nofix_mask = gnss[:, 3] == -1

    if np.any(nofix_mask):
        ax.scatter(
            gnss[nofix_mask, 1],
            gnss[nofix_mask, 2],
            c="red",
            s=1.5 * point_size,
            label=f"No Fix ({np.sum(nofix_mask)})",
            alpha=0.6,
        )
    if np.any(single_mask):
        ax.scatter(
            gnss[single_mask, 1],
            gnss[single_mask, 2],
            c="gray",
            s=1.5 * point_size,
            label=f"Single/3D ({np.sum(single_mask)})",
            alpha=0.6,
        )
    if np.any(float_mask):
        ax.scatter(
            gnss[float_mask, 1],
            gnss[float_mask, 2],
            c="orange",
            s=1.5 * point_size,
            label=f"RTK Float ({np.sum(float_mask)})",
            alpha=0.8,
        )
    if np.any(fix_mask):
        ax.scatter(
            gnss[fix_mask, 1],
            gnss[fix_mask, 2],
            c="green",
            s=1.5 * point_size,
            label=f"RTK Fix ({np.sum(fix_mask)})",
            alpha=1.0,
            zorder=5,
        )

    # 誤差円（精度円）
    if show_accuracy_circles and len(gnss) > 0:
        add_accuracy_circles(
            ax,
            gnss[:, 1],
            gnss[:, 2],
            gnss[:, 4],
            gnss[:, 3],
            step=circle_step,
            scale=circle_scale,
            max_radius=max_circle_radius,
            alpha=circle_alpha,
        )

    # 開始点・終了点
    if len(gnss) > 0:
        ax.plot(gnss[0, 1], gnss[0, 2], "bo",
                markersize=3 * point_size, label="Start")
        ax.plot(gnss[-1, 1], gnss[-1, 2], "ks",
                markersize=3 * point_size, label="End")

    ax.set_title(title, fontsize=14)
    ax.set_xlabel("UTM Easting [m]", fontsize=12)
    ax.set_ylabel("UTM Northing [m]", fontsize=12)
    ax.legend(loc="best")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    fig.savefig(output_path, dpi=150, transparent=transparent)
    plt.close(fig)
    print(f"画像を保存しました: {output_path}")


def plot_gnss_and_odom(
    gnss: np.ndarray,
    odom: np.ndarray,
    output_path: str,
    overlay: bool = False,
    initial_yaw_deg: Optional[float] = None,
    anchor_e: Optional[float] = None,
    anchor_n: Optional[float] = None,
    show_accuracy_circles: bool = False,
    circle_step: Optional[int] = None,
    circle_scale: float = 1.0,
    max_circle_radius: float = 20.0,
    circle_alpha: float = 0.25,
    point_size: float = 1.0,
    title: str = "GNSS vs Odometry Comparison",
):
    """GNSSとオドメトリを比較プロットする（並列形状比較、またはUTM重ね合わせ）。"""
    if len(odom) == 0:
        print("オドメトリデータが空のため、GNSS単体で描画します。")
        plot_gnss_only(
            gnss,
            output_path,
            show_accuracy_circles=show_accuracy_circles,
            circle_step=circle_step,
            circle_scale=circle_scale,
            max_circle_radius=max_circle_radius,
            circle_alpha=circle_alpha,
            point_size=point_size,
        )
        return

    if not overlay:
        # 並列表示 (Side-by-side 形状比較)
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))

        # GNSS 相対座標
        ax1 = axes[0]
        gnss_rel = gnss[:, 1:3] - \
            gnss[0, 1:3] if len(gnss) > 0 else np.zeros((1, 2))
        ax1.plot(
            gnss_rel[:, 0],
            gnss_rel[:, 1],
            "g.-",
            linewidth=0.8 * point_size,
            markersize=3 * point_size,
            label="GNSS Relative (dE, dN)",
        )
        ax1.set_title("GNSS Relative Trajectory (East=X, North=Y)")
        ax1.set_xlabel("dEasting [m]")
        ax1.set_ylabel("dNorthing [m]")
        ax1.axis("equal")
        ax1.grid(True, linestyle="--", alpha=0.5)
        ax1.legend()

        # Odom 相対座標
        ax2 = axes[1]
        odom_rel = odom[:, 1:3] - odom[0, 1:3]
        ax2.plot(
            odom_rel[:, 0],
            odom_rel[:, 1],
            "b.-",
            linewidth=0.8 * point_size,
            markersize=3 * point_size,
            label="Raw Odom (Forward=X, Left=Y)",
        )
        ax2.set_title("Raw Odometry Trajectory")
        ax2.set_xlabel("Odom X [m]")
        ax2.set_ylabel("Odom Y [m]")
        ax2.axis("equal")
        ax2.grid(True, linestyle="--", alpha=0.5)
        ax2.legend()

        plt.suptitle(title, fontsize=14)
        plt.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        print(f"画像を保存しました: {output_path}")

    else:
        # UTM座標系上での回転重ね合わせ (Overlay)
        # 初期アンカー
        ae = anchor_e if anchor_e is not None else (
            gnss[0, 1] if len(gnss) > 0 else 0.0)
        an = anchor_n if anchor_n is not None else (
            gnss[0, 2] if len(gnss) > 0 else 0.0)

        # 初期角度
        yaw_rad = math.radians(
            initial_yaw_deg) if initial_yaw_deg is not None else 0.0
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)

        # Odomの原点基準化と回転
        ox = odom[:, 1] - odom[0, 1]
        oy = odom[:, 2] - odom[0, 2]
        odom_utm_e = c * ox - s * oy + ae
        odom_utm_n = s * ox + c * oy + an

        fig, ax = plt.subplots(figsize=(12, 12))

        # GNSSプロット
        if len(gnss) > 0:
            fix_mask = gnss[:, 3] == 2
            float_mask = gnss[:, 3] == 1
            other_mask = gnss[:, 3] <= 0

            ax.plot(gnss[:, 1], gnss[:, 2], "k.",
                    markersize=1, alpha=0.2, label="GNSS All")
            if np.any(other_mask):
                ax.scatter(
                    gnss[other_mask, 1],
                    gnss[other_mask, 2],
                    c="gray",
                    s=1.5 * point_size,
                    label="GNSS Single/NoFix",
                    alpha=0.5,
                )
            if np.any(float_mask):
                ax.scatter(
                    gnss[float_mask, 1],
                    gnss[float_mask, 2],
                    c="orange",
                    s=1.5 * point_size,
                    label="GNSS Float",
                    alpha=0.7,
                )
            if np.any(fix_mask):
                ax.scatter(
                    gnss[fix_mask, 1],
                    gnss[fix_mask, 2],
                    c="green",
                    s=1.5 * point_size,
                    label="GNSS RTK Fix",
                    zorder=5,
                )

            # 誤差円（精度円）
            if show_accuracy_circles:
                add_accuracy_circles(
                    ax,
                    gnss[:, 1],
                    gnss[:, 2],
                    gnss[:, 4],
                    gnss[:, 3],
                    step=circle_step,
                    scale=circle_scale,
                    max_radius=max_circle_radius,
                    alpha=circle_alpha,
                )

        # Odomプロット
        ax.plot(
            odom_utm_e,
            odom_utm_n,
            "b-",
            linewidth=0.9 * point_size,
            label=f"Rotated Odom (yaw={initial_yaw_deg or 0:.1f}°)",
            alpha=0.8,
        )
        ax.plot(ae, an, "ro", markersize=5 * point_size, label="Anchor Point")

        ax.set_title(
            f"{title} (UTM Frame, Odom Rotated {initial_yaw_deg or 0:.1f}°)", fontsize=14
        )
        ax.set_xlabel("UTM Easting [m]", fontsize=12)
        ax.set_ylabel("UTM Northing [m]", fontsize=12)
        ax.legend(loc="best")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.5)

        plt.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        print(f"画像を保存しました: {output_path}")


def plot_on_map(
    gnss: np.ndarray,
    odom: np.ndarray,
    map_yaml_path: str,
    output_path: str,
    transform: Optional[Tuple[float, float, float]] = None,
    initial_yaw_deg: Optional[float] = None,
    map_alpha: float = 0.6,
    show_accuracy_circles: bool = False,
    circle_step: Optional[int] = None,
    circle_scale: float = 1.0,
    max_circle_radius: float = 20.0,
    circle_alpha: float = 0.25,
    point_size: float = 1.0,
    title: str = "Trajectory Overlaid on Map",
):
    """
    OccupancyGrid 地図画像の上に GNSS 軌跡（およびオドメトリ）を重ね合わせてプロットする。
    アライメント:
      - transform: (tx, ty, yaw) [UTM -> map]
      - 未設定の場合は GNSS 開始点アンカー + initial_yaw_deg、または自動バウンディングボックス合わせ
    """
    map_img, origin, resolution, _ = load_map_info(map_yaml_path)
    h_px, w_px = map_img.shape[:2]

    # map座標系の範囲 [m]
    map_x0 = origin[0]
    map_y0 = origin[1]
    map_x1 = map_x0 + w_px * resolution
    map_y1 = map_y0 + h_px * resolution

    # UTM -> map 座標変換の決定
    if transform is not None:
        tx, ty, tyaw = transform
    elif initial_yaw_deg is not None and len(gnss) > 0:
        tyaw = math.radians(initial_yaw_deg)
        # GNSS開始点をマップ原点 (または重心) に合わせる
        ae, an = gnss[0, 1], gnss[0, 2]
        c = math.cos(tyaw)
        s = math.sin(tyaw)
        tx = - (c * ae - s * an)
        ty = - (s * ae + c * an)
    elif len(gnss) > 0:
        # 自動フィッティング: GNSSの重心を地図画像の中心に配置
        center_map_x = (map_x0 + map_x1) / 2.0
        center_map_y = (map_y0 + map_y1) / 2.0
        ae = float(np.mean(gnss[:, 1]))
        an = float(np.mean(gnss[:, 2]))
        tyaw = 0.0
        tx = center_map_x - ae
        ty = center_map_y - an
    else:
        tx, ty, tyaw = 0.0, 0.0, 0.0

    # GNSS 点の map 座標系への投影
    c = math.cos(tyaw)
    s = math.sin(tyaw)

    gnss_map_x = tx + c * gnss[:, 1] - s * gnss[:, 2]
    gnss_map_y = ty + s * gnss[:, 1] + c * gnss[:, 2]

    fig, ax = plt.subplots(figsize=(12, 12))

    # 地図画像の描画 (extent: [left, right, bottom, top])
    ax.imshow(
        map_img,
        origin="lower",
        extent=[map_x0, map_x1, map_y0, map_y1],
        alpha=map_alpha,
        cmap="gray",
    )

    # GNSS軌跡のプロット
    fix_mask = gnss[:, 3] == 2
    float_mask = gnss[:, 3] == 1
    other_mask = gnss[:, 3] <= 0

    if np.any(other_mask):
        ax.scatter(
            gnss_map_x[other_mask],
            gnss_map_y[other_mask],
            c="gray",
            s=1.5 * point_size,
            label="GNSS Single/NoFix",
            alpha=0.5,
        )
    if np.any(float_mask):
        ax.scatter(
            gnss_map_x[float_mask],
            gnss_map_y[float_mask],
            c="orange",
            s=1.5 * point_size,
            label="GNSS RTK Float",
            alpha=0.8,
        )
    if np.any(fix_mask):
        ax.scatter(
            gnss_map_x[fix_mask],
            gnss_map_y[fix_mask],
            c="green",
            s=1.5 * point_size,
            label="GNSS RTK Fix",
            zorder=5,
        )

    # 誤差円（精度円）
    if show_accuracy_circles and len(gnss) > 0:
        add_accuracy_circles(
            ax,
            gnss_map_x,
            gnss_map_y,
            gnss[:, 4],
            gnss[:, 3],
            step=circle_step,
            scale=circle_scale,
            max_radius=max_circle_radius,
            alpha=circle_alpha,
        )

    # オドメトリがある場合は map 座標系でそのままプロット
    if len(odom) > 0:
        ax.plot(
            odom[:, 1],
            odom[:, 2],
            "b-",
            linewidth=0.8 * point_size,
            label="Odometry",
            alpha=0.7,
        )

    # 描画範囲の調整（地図または軌跡全体が含まれるように）
    all_x = np.concatenate([[map_x0, map_x1], gnss_map_x])
    all_y = np.concatenate([[map_y0, map_y1], gnss_map_y])
    margin = 5.0
    ax.set_xlim(np.min(all_x) - margin, np.max(all_x) + margin)
    ax.set_ylim(np.min(all_y) - margin, np.max(all_y) + margin)

    ax.set_title(title, fontsize=14)
    ax.set_xlabel("Map X [m]", fontsize=12)
    ax.set_ylabel("Map Y [m]", fontsize=12)
    ax.legend(loc="best")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"地図オーバーレイ画像を保存しました: {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="rosbag の GNSS 軌跡・品質を可視化し、オドメトリや地図と比較するツール"
    )
    parser.add_argument("bag_path", type=str, help="rosbagディレクトリまたはファイルパス")
    add_output_args(parser, default_filename="gnss_trajectory.png")
    parser.add_argument(
        "--mode",
        choices=["gnss", "odom", "map"],
        default="gnss",
        help="描画モード: gnss (単体), odom (オドメトリ比較), map (地図オーバーレイ)",
    )
    parser.add_argument(
        "--compare-odom", action="store_true", help="オドメトリ軌跡と比較描画する (mode=odom と同義)"
    )
    parser.add_argument(
        "--overlay",
        action="store_true",
        help="オドメトリ比較時に2画面並列ではなくUTM座標系上で重ね合わせてプロット",
    )
    parser.add_argument(
        "--initial-yaw",
        type=float,
        default=None,
        help="オドメトリ重ね合わせや地図投影時の初期方位角 [deg]",
    )
    parser.add_argument(
        "--map", type=str, default=None, help="オーバーレイ対象の地図YAMLパス (mode=map 用)"
    )
    parser.add_argument(
        "--static-transforms",
        type=str,
        default=None,
        help="generate_static_transforms.py が出力した static_transforms YAML パス",
    )
    parser.add_argument(
        "--label", type=str, default=None, help="static_transforms 内の対象 label 名"
    )
    parser.add_argument(
        "--map-alpha", type=float, default=0.6, help="地図オーバーレイ時の背景地図透過度 (0.0 - 1.0)"
    )
    parser.add_argument(
        "--periods", action="store_true", help="Fix/Float/Single の区間推移サマリーを端末に表示"
    )
    parser.add_argument(
        "--utm-zone", type=int, default=54, help="WGS84->UTM 変換ゾーン (デフォルト: 54)"
    )
    parser.add_argument(
        "--transparent", action="store_true", help="背景を透明にして画像保存"
    )
    parser.add_argument(
        "--accuracy-circles",
        action="store_true",
        default=False,
        help="GNSS精度円 (NavPVT hAcc) を描画",
    )
    parser.add_argument(
        "--circle-scale",
        type=float,
        default=1.0,
        help="精度円の半径スケール倍率 (例: 5.0 で5倍表示、デフォルト: 1.0)",
    )
    parser.add_argument(
        "--no-accuracy-circles",
        action="store_true",
        help="GNSS精度円の描画を明示的に無効化",
    )
    parser.add_argument(
        "--circle-step",
        type=int,
        default=None,
        help="精度円を描画するサンプリング間隔 (N点ごと、1で全点描画、未指定時は約40点に自動調整)",
    )
    parser.add_argument(
        "--max-circle-radius",
        type=float,
        default=20.0,
        help="描画する精度円の最大半径 [m] (デフォルト: 20.0)",
    )
    parser.add_argument(
        "--circle-alpha",
        type=float,
        default=0.25,
        help="精度円の透過度 (デフォルト: 0.25)",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=1.0,
        help="軌跡パス（マーカー・線幅）の太さ倍率 (デフォルト: 1.0)",
    )
    parser.add_argument(
        "--start-time", type=float, default=None, help="抽出開始秒 (rosbag開始からの経過秒)"
    )
    parser.add_argument(
        "--end-time", type=float, default=None, help="抽出終了秒 (rosbag開始からの経過秒)"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    bag_path = os.path.abspath(args.bag_path)

    if not os.path.exists(bag_path):
        print(f"エラー: 指定された rosbag パスが存在しません: {bag_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading bag: {bag_path}")
    gnss, odom = extract_bag_data(
        bag_path,
        utm_zone=args.utm_zone,
        start_sec=args.start_time,
        end_sec=args.end_time,
    )
    print(f"Extracted: {len(gnss)} GNSS points, {len(odom)} Odometry points")

    if len(gnss) == 0:
        print("警告: 有効な GNSS データが見つかりませんでした。", file=sys.stderr)
        if len(odom) == 0:
            print("エラー: GNSS・オドメトリともにデータがありません。", file=sys.stderr)
            sys.exit(1)

    if args.periods:
        print_status_periods(gnss)

    # 描画モードの決定
    mode = args.mode
    if args.compare_odom or args.overlay:
        mode = "odom"
    elif args.map:
        mode = "map"

    output_path = resolve_output_path(
        bag_path,
        default_filename="gnss_trajectory.png",
        output=args.output,
        output_to_bag_dir=args.output_to_bag_dir,
        output_dir=args.output_dir,
    )

    show_circles = False
    if args.no_accuracy_circles:
        show_circles = False
    elif args.accuracy_circles or (args.circle_step is not None) or (args.circle_scale != 1.0):
        show_circles = True

    if mode == "map":
        if not args.map:
            print("エラー: mode=map には --map <map.yaml> の指定が必要です。", file=sys.stderr)
            sys.exit(1)

        transform = None
        if args.static_transforms:
            transform = load_transform_from_yaml(
                args.static_transforms, args.label)
            if transform:
                print(f"Loaded transform (label={args.label}): {transform}")

        plot_on_map(
            gnss,
            odom,
            args.map,
            output_path,
            transform=transform,
            initial_yaw_deg=args.initial_yaw,
            map_alpha=args.map_alpha,
            show_accuracy_circles=show_circles,
            circle_step=args.circle_step,
            circle_scale=args.circle_scale,
            max_circle_radius=args.max_circle_radius,
            circle_alpha=args.circle_alpha,
            point_size=args.point_size,
        )

    elif mode == "odom":
        plot_gnss_and_odom(
            gnss,
            odom,
            output_path,
            overlay=args.overlay,
            initial_yaw_deg=args.initial_yaw,
            show_accuracy_circles=show_circles,
            circle_step=args.circle_step,
            circle_scale=args.circle_scale,
            max_circle_radius=args.max_circle_radius,
            circle_alpha=args.circle_alpha,
            point_size=args.point_size,
        )

    else:
        # GNSS 単体
        plot_gnss_only(
            gnss,
            output_path,
            show_accuracy_circles=show_circles,
            circle_step=args.circle_step,
            circle_scale=args.circle_scale,
            max_circle_radius=args.max_circle_radius,
            circle_alpha=args.circle_alpha,
            point_size=args.point_size,
            transparent=args.transparent,
        )


if __name__ == "__main__":
    main()
