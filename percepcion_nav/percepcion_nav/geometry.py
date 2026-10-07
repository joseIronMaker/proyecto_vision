"""
Geometría de la cámara sin dependencias de ROS (constitución, principio V).

Modelo pinhole: un píxel (u, v) con profundidad Z en el marco óptico (REP 103: Z hacia
adelante, X a la derecha, Y hacia abajo) corresponde al punto

    X = (u − cx)·Z/fx,    Y = (v − cy)·Z/fy,    Z.
"""

import math
from typing import NamedTuple

import numpy as np


class Intrinsics(NamedTuple):
    """Parámetros intrínsecos de la cámara, en píxeles."""

    fx: float
    fy: float
    cx: float
    cy: float


# Escala de cada codificación de profundidad a metros (FR-032).
_DEPTH_SCALES = {
    '32FC1': 1.0,     # metros en float
    '16UC1': 0.001,   # milímetros en entero
    'mono16': 0.001,  # milímetros en entero
}


def intrinsics_from_k(k):
    """Lee fx, fy, cx, cy de la matriz K (fila mayor) de sensor_msgs/CameraInfo."""
    return Intrinsics(float(k[0]), float(k[4]), float(k[2]), float(k[5]))


def intrinsics_from_hfov(width, height, hfov_rad):
    """Intrínsecos de una cámara ideal: fx = fy = (w/2)/tan(hfov/2), centro en la imagen."""
    focal = (width / 2.0) / math.tan(hfov_rad / 2.0)
    return Intrinsics(focal, focal, width / 2.0, height / 2.0)


def deproject(u, v, z, intrinsics):
    """Devuelve (X, Y, Z) en metros del píxel (u, v) a profundidad z en el marco óptico."""
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    return x, y, z


def depth_scale_for_encoding(encoding):
    """Devuelve la escala a metros de una codificación de profundidad, o None si no se soporta."""
    return _DEPTH_SCALES.get(encoding)


def central_window(cx, cy, width, height, fraction, image_width, image_height):
    """
    Ventana central del bbox recortada a la imagen, como (x0, y0, x1, y1) semiabierta.

    Los lados son max(1, round(w·f)) x max(1, round(h·f)) píxeles y se centran en el centro del
    bbox. Devuelve None si la ventana queda vacía tras recortarla a la imagen.
    """
    side_x = max(1, int(round(width * fraction)))
    side_y = max(1, int(round(height * fraction)))
    x0 = int(math.floor(cx - side_x / 2.0))
    y0 = int(math.floor(cy - side_y / 2.0))
    x1 = min(x0 + side_x, int(image_width))
    y1 = min(y0 + side_y, int(image_height))
    x0 = max(x0, 0)
    y0 = max(y0, 0)
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def robust_depth(depth, window, scale, min_valid):
    """
    Mediana en metros de los píxeles válidos de la ventana, o None si hay menos de min_valid.

    Un píxel es válido si es finito y mayor que 0 (se descartan 0, NaN e infinitos).
    """
    x0, y0, x1, y1 = window
    values = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64).ravel()
    valid = values[np.isfinite(values) & (values > 0.0)]
    if valid.size < max(1, int(min_valid)):
        return None
    return float(np.median(valid)) * float(scale)


def metric_size(width_px, height_px, z, intrinsics):
    """Extensión métrica aproximada (ancho, alto) en metros de un bbox a profundidad z."""
    return width_px * z / intrinsics.fx, height_px * z / intrinsics.fy


def center_along_ray(surface_xyz, depth_extent):
    """
    Desplaza un punto de la superficie visible hasta el centro estimado del objeto.

    La cámara mide la cara visible. Si el objeto tiene una profundidad depth_extent (en metros),
    su centro está depth_extent/2 más lejos sobre el mismo rayo: Z' = Z + e/2 y X, Y escalan
    con Z'/Z, de modo que el punto sigue proyectándose en el mismo píxel.
    """
    x, y, z = surface_xyz
    center_z = z + depth_extent / 2.0
    scale = center_z / z
    return x * scale, y * scale, center_z
