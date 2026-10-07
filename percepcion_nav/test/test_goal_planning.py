"""Pruebas puras de percepcion_nav.goal_planning (sin ROS): SC-007, FR-040, FR-042, FR-044."""

import math

from percepcion_nav import goal_planning
import pytest


def test_goal_planning_does_not_import_ros():
    import ast
    with open(goal_planning.__file__, encoding='utf-8') as source:
        tree = ast.parse(source.read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    assert imported <= {'math'}, imported


def test_goal_on_axis():
    assert goal_planning.compute_goal((0.0, 0.0), (2.5, 0.0), 1.0, 0.30) == pytest.approx(
        (1.5, 0.0, 0.0))


def test_goal_diagonal_meets_sc007():
    robot, target = (1.0, 1.0), (4.0, 5.0)
    gx, gy, yaw = goal_planning.compute_goal(robot, target, 1.0, 0.30)
    assert (gx, gy) == pytest.approx((3.4, 4.2))
    # SC-007: a la distancia de separación con tolerancia de 1 cm y orientada con error < 1°.
    assert math.dist((gx, gy), target) == pytest.approx(1.0, abs=0.01)
    expected_yaw = math.atan2(target[1] - gy, target[0] - gx)
    assert abs(math.degrees(yaw - expected_yaw)) < 1.0
    assert yaw == pytest.approx(math.atan2(4.0, 3.0))


@pytest.mark.parametrize('distance, expect_goal', [(1.29, False), (1.30, False), (1.31, True)])
def test_no_goal_within_standoff_plus_tolerance(distance, expect_goal):
    goal = goal_planning.compute_goal((0.0, 0.0), (distance, 0.0), 1.0, 0.30)
    assert (goal is not None) == expect_goal


def test_yaw_to_quaternion():
    for yaw in (0.0, 0.7, -2.1, math.pi):
        assert goal_planning.yaw_to_quaternion(yaw) == pytest.approx(
            (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))


def test_target_moved_threshold():
    assert goal_planning.target_moved((0.0, 0.0), (0.3, 0.4), 0.5) is False
    assert goal_planning.target_moved((0.0, 0.0), (0.3, 0.41), 0.5) is True
    assert goal_planning.target_moved(None, (0.0, 0.0), 0.5) is True


def test_select_best_highest_score_of_target_class():
    candidates = [('person', 0.9, 'a'), ('chair', 0.6, 'b'), ('chair', 0.8, 'c')]
    assert goal_planning.select_best(candidates, 'chair') == 'c'
    assert goal_planning.select_best(candidates, 'dog') is None
    assert goal_planning.select_best([], 'chair') is None


def test_plausible_jump():
    assert goal_planning.plausible_jump(None, (9.0, 1.0), 1.5) is True
    assert goal_planning.plausible_jump((3.0, 0.0), (3.4, 0.3), 1.5) is True
    assert goal_planning.plausible_jump((3.0, 0.0), (9.87, 0.93), 1.5) is False
