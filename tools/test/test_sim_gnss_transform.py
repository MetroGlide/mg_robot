import math
import os

import pyproj
import pytest

from tools.scripts.make_sim_gnss_transform import build_transform, read_spherical_coordinates

WAREHOUSE_SDF = os.path.join(
    os.path.dirname(__file__), "..", "..", "mg_simulation", "worlds", "warehouse.sdf")


def test_read_spherical_coordinates_from_world():
    lat, lon = read_spherical_coordinates(WAREHOUSE_SDF)
    assert lat == pytest.approx(-22.986687)
    assert lon == pytest.approx(-43.202501)


def test_southern_hemisphere_zone_and_convergence():
    transform = build_transform(-22.986687, -43.202501)
    assert transform["anchor_utm"]["zone"] == 23
    assert transform["anchor_utm"]["hemisphere"] == "south"
    # 中央子午線 (-45 度) の東 1.8 度、南緯 23 度: 収束角は約 -0.70 度 (経度差 x sin 緯度)。
    # rotation_rad は、UTM の座標を東・北に揃える回転なので、その逆の +0.70 度
    assert math.degrees(transform["rotation_rad"]) == pytest.approx(
        -1.798 * math.sin(math.radians(-22.99)), abs=0.02)


def test_rotation_maps_utm_offsets_to_east_north():
    lat, lon = 36.0, 140.0
    transform = build_transform(lat, lon)
    proj = pyproj.Proj(proj="utm", zone=transform["anchor_utm"]["zone"], ellps="WGS84")
    e0, n0 = transform["anchor_utm"]["easting"], transform["anchor_utm"]["northing"]
    # 原点から真東へ 200 m の点
    east_lon, east_lat, _ = pyproj.Geod(ellps="WGS84").fwd(lon, lat, 90.0, 200.0)
    e, n = proj(east_lon, east_lat)
    theta = transform["rotation_rad"]
    x = math.cos(theta) * (e - e0) - math.sin(theta) * (n - n0)
    y = math.sin(theta) * (e - e0) + math.cos(theta) * (n - n0)
    # 長さは UTM の縮尺係数 (約 0.9996) の分だけ変わるので、向きで確かめる
    assert x == pytest.approx(200.0, abs=0.15)
    assert math.atan2(y, x) == pytest.approx(0.0, abs=1e-4)
