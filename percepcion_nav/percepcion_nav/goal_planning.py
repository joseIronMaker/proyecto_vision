"""
Cálculo de la meta de navegación sin dependencias de ROS (constitución, principio V).

Todas las posiciones están en el plano del marco del mapa, en metros. La meta g se coloca a la
distancia de separación d del objeto o, sobre la línea robot r -> objeto, orientada hacia él:

    g = o − d·(o − r)/‖o − r‖,    yaw = atan2(o_y − g_y, o_x − g_x).
"""

import math


def select_best(candidates, target_class):
    """
    Elige, entre tuplas (clase, score, dato), el dato de la clase objetivo con mayor score.

    Devuelve None si no hay candidatos de esa clase (FR-040).
    """
    best = None
    best_score = -math.inf
    for class_name, score, payload in candidates:
        if class_name == target_class and score > best_score:
            best, best_score = payload, score
    return best


def compute_goal(robot_xy, object_xy, standoff, tolerance):
    """
    Devuelve (x, y, yaw) de la meta, o None si el robot ya está suficientemente cerca.

    No hay meta si ‖o − r‖ ≤ d + t: el robot ya está a la distancia de separación más la
    tolerancia de llegada, o más cerca (FR-042).
    """
    dx = object_xy[0] - robot_xy[0]
    dy = object_xy[1] - robot_xy[1]
    distance = math.hypot(dx, dy)
    if distance <= standoff + tolerance:
        return None
    goal_x = object_xy[0] - standoff * dx / distance
    goal_y = object_xy[1] - standoff * dy / distance
    yaw = math.atan2(object_xy[1] - goal_y, object_xy[0] - goal_x)
    return goal_x, goal_y, yaw


def yaw_to_quaternion(yaw):
    """Cuaternión (x, y, z, w) de un giro yaw alrededor del eje z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def plausible_jump(previous_xy, new_xy, max_jump):
    """
    Indica si un objetivo nuevo es compatible con el anterior para un objeto estático.

    Un salto mayor que max_jump delata un error de estimación (por ejemplo, la profundidad vio
    el fondo a través del objeto) u otro objeto. Sin objetivo previo, todo salto es válido.
    """
    if previous_xy is None:
        return True
    return math.dist(previous_xy, new_xy) <= max_jump


def target_moved(previous_xy, new_xy, threshold):
    """Indica si el objetivo se movió más que el umbral; sin objetivo previo, cuenta como sí."""
    if previous_xy is None:
        return True
    return math.dist(previous_xy, new_xy) > threshold
