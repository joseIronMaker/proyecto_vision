"""Pruebas puras de percepcion_nav.geometry (sin ROS): SC-006, FR-031, FR-032, FR-034."""

import math

import numpy as np
from percepcion_nav import geometry
from percepcion_nav.geometry import Intrinsics
import pytest


def test_geometry_does_not_import_ros():
    import ast
    with open(geometry.__file__, encoding='utf-8') as source:
        tree = ast.parse(source.read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    assert imported <= {'math', 'typing', 'numpy'}, imported


def test_deproject_known_point():
    x, y, z = geometry.deproject(420, 140, 2.0, Intrinsics(500.0, 500.0, 320.0, 240.0))
    assert (x, y, z) == pytest.approx((0.4, -0.4, 2.0), abs=1e-9)


def test_deproject_roundtrip_error_below_1mm():
    intr = Intrinsics(fx=615.3, fy=612.8, cx=318.4, cy=241.7)
    rng = np.random.default_rng(0)
    points = rng.uniform([-2.0, -1.5, 0.4], [2.0, 1.5, 6.0], size=(500, 3))
    for px, py, pz in points:
        u = intr.fx * px / pz + intr.cx
        v = intr.fy * py / pz + intr.cy
        x, y, z = geometry.deproject(u, v, pz, intr)
        assert math.dist((x, y, z), (px, py, pz)) < 1e-3


@pytest.mark.parametrize('encoding, scale', [
    ('32FC1', 1.0), ('16UC1', 0.001), ('mono16', 0.001), ('bgr8', None), ('rgb8', None)])
def test_depth_scale_for_encoding(encoding, scale):
    assert geometry.depth_scale_for_encoding(encoding) == scale


def test_central_window_centered_and_sized():
    window = geometry.central_window(100, 80, 40, 20, 0.5, 640, 480)
    # lados = max(1, round(w·f)) x max(1, round(h·f)) = 20 x 10, centrados en (100, 80)
    assert window == (90, 75, 110, 85)


def test_central_window_minimum_one_pixel():
    x0, y0, x1, y1 = geometry.central_window(10, 10, 2, 2, 0.1, 640, 480)
    assert (x1 - x0, y1 - y0) == (1, 1)


def test_central_window_clipped_at_border():
    x0, y0, x1, y1 = geometry.central_window(2, 478, 40, 40, 0.5, 640, 480)
    assert x0 == 0 and y1 == 480
    assert x1 > x0 and y1 > y0


def test_central_window_outside_image_is_none():
    assert geometry.central_window(-50, -50, 10, 10, 0.5, 640, 480) is None


def test_robust_depth_median_ignores_invalid_values():
    depth = np.full((10, 10), 2.0, dtype=np.float32)
    depth[0, :] = 0.0
    depth[1, :] = np.nan
    depth[2, :] = np.inf
    depth[3, :] = -np.inf
    depth[4, :5] = 10.0
    z = geometry.robust_depth(depth, (0, 0, 10, 10), 1.0, min_valid=5)
    assert z == pytest.approx(2.0)


def test_robust_depth_all_invalid_returns_none():
    depth = np.zeros((10, 10), dtype=np.float32)
    depth[5:, :] = np.nan
    assert geometry.robust_depth(depth, (0, 0, 10, 10), 1.0, min_valid=1) is None


def test_robust_depth_requires_min_valid_pixels():
    depth = np.zeros((10, 10), dtype=np.float32)
    depth[0, :3] = 1.5
    assert geometry.robust_depth(depth, (0, 0, 10, 10), 1.0, min_valid=5) is None
    assert geometry.robust_depth(depth, (0, 0, 10, 10), 1.0, min_valid=3) == pytest.approx(1.5)


def test_robust_depth_millimeters_to_meters():
    depth = np.full((4, 4), 1500, dtype=np.uint16)
    depth[0, 0] = 0
    assert geometry.robust_depth(depth, (0, 0, 4, 4), 0.001, min_valid=5) == pytest.approx(1.5)


def test_intrinsics_from_k():
    k = [600.0, 0.0, 320.0, 0.0, 610.0, 240.0, 0.0, 0.0, 1.0]
    assert geometry.intrinsics_from_k(k) == Intrinsics(600.0, 610.0, 320.0, 240.0)


def test_intrinsics_from_hfov():
    intr = geometry.intrinsics_from_hfov(640, 480, 1.2)
    focal = 320.0 / math.tan(0.6)
    assert intr == pytest.approx(Intrinsics(focal, focal, 320.0, 240.0))


def test_metric_size():
    intr = Intrinsics(500.0, 400.0, 0.0, 0.0)
    assert geometry.metric_size(100, 80, 2.0, intr) == pytest.approx((0.4, 0.4))


def test_center_along_ray_moves_point_half_extent_further():
    intr = Intrinsics(500.0, 500.0, 320.0, 240.0)
    surface = geometry.deproject(420, 140, 2.0, intr)
    center = geometry.center_along_ray(surface, 0.6)
    assert center[2] == pytest.approx(2.3)
    # Sigue sobre el mismo rayo: se proyecta en el mismo píxel.
    assert intr.fx * center[0] / center[2] + intr.cx == pytest.approx(420)
    assert intr.fy * center[1] / center[2] + intr.cy == pytest.approx(240 - 100)


def test_center_along_ray_zero_extent_is_identity():
    assert geometry.center_along_ray((0.1, -0.2, 1.5), 0.0) == pytest.approx((0.1, -0.2, 1.5))
