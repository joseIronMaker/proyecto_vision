"""Pruebas de construcción de mensajes vision_msgs (FR-013, FR-014, FR-015, FR-033)."""

from builtin_interfaces.msg import Time
import pytest
from std_msgs.msg import Header


def make_header(sec=12, nanosec=345, frame='camera_color_optical_frame'):
    """Crea un header de prueba con estampa y marco no vacíos."""
    return Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=frame)


# --- Detector (US1) ------------------------------------------------------------------------

def test_parse_class_filter_empty_means_all():
    from percepcion_nav.detector_node import parse_class_filter
    assert parse_class_filter('') == set()
    assert parse_class_filter('  ') == set()


def test_parse_class_filter_trims_and_splits():
    from percepcion_nav.detector_node import parse_class_filter
    assert parse_class_filter(' chair, person ') == {'chair', 'person'}
    assert parse_class_filter('chair,,') == {'chair'}


def test_detection2d_array_fields_and_header():
    from percepcion_nav.detector_node import build_detection2d_array
    header = make_header()
    boxes = [(100.0, 50.0, 40.0, 20.0, 'chair', 0.87), (10.5, 20.5, 5.0, 6.0, 'person', 0.5)]
    msg = build_detection2d_array(header, boxes)

    assert msg.header == header
    assert len(msg.detections) == 2
    det = msg.detections[0]
    assert det.header == header
    assert det.bbox.center.position.x == pytest.approx(100.0)
    assert det.bbox.center.position.y == pytest.approx(50.0)
    assert det.bbox.center.theta == 0.0
    assert det.bbox.size_x == pytest.approx(40.0)
    assert det.bbox.size_y == pytest.approx(20.0)
    assert det.results[0].hypothesis.class_id == 'chair'
    assert det.results[0].hypothesis.score == pytest.approx(0.87)
    assert msg.detections[1].results[0].hypothesis.class_id == 'person'


def test_detection2d_array_skips_non_positive_sizes():
    from percepcion_nav.detector_node import build_detection2d_array
    boxes = [(1.0, 1.0, 0.0, 5.0, 'chair', 0.9),
             (1.0, 1.0, 5.0, -1.0, 'chair', 0.9),
             (1.0, 1.0, 5.0, 5.0, 'chair', 0.9)]
    msg = build_detection2d_array(make_header(), boxes)
    assert len(msg.detections) == 1
    assert msg.detections[0].bbox.size_x > 0 and msg.detections[0].bbox.size_y > 0


def test_detection2d_array_empty_keeps_header():
    from percepcion_nav.detector_node import build_detection2d_array
    header = make_header(sec=99)
    msg = build_detection2d_array(header, [])
    assert msg.header == header
    assert len(msg.detections) == 0


# --- Localizador RGB-D (US3) -----------------------------------------------------------------

def _detection2d(header, class_name='chair', score=0.8):
    from percepcion_nav.detector_node import build_detection2d_array
    return build_detection2d_array(header, [(50.0, 40.0, 20.0, 10.0, class_name, score)])


def test_detection3d_copies_hypothesis_and_position():
    from percepcion_nav.rgbd_localizer_node import build_detection3d
    det2d = _detection2d(make_header()).detections[0]
    det3d = build_detection3d(det2d, (0.1, -0.2, 2.5), (0.3, 0.4))
    assert det3d.results[0].hypothesis.class_id == 'chair'
    assert det3d.results[0].hypothesis.score == pytest.approx(0.8)
    position = det3d.results[0].pose.pose.position
    assert (position.x, position.y, position.z) == pytest.approx((0.1, -0.2, 2.5))
    center = det3d.bbox.center.position
    assert (center.x, center.y, center.z) == pytest.approx((0.1, -0.2, 2.5))
    assert det3d.results[0].pose.pose.orientation.w == 1.0
    assert det3d.bbox.center.orientation.w == 1.0
    size = det3d.bbox.size
    assert (size.x, size.y, size.z) == pytest.approx((0.3, 0.4, 0.0))
    assert det3d.header == det2d.header


def test_detection3d_array_keeps_source_header():
    from percepcion_nav.rgbd_localizer_node import build_detection3d, build_detection3d_array
    header = make_header(sec=77)
    det2d = _detection2d(header).detections[0]
    array = build_detection3d_array(header, [build_detection3d(det2d, (0, 0, 1), (0, 0))])
    assert array.header == header
    assert len(array.detections) == 1
    assert build_detection3d_array(header, []).header == header


def test_detection3d_size_z_when_depth_extent_known():
    from percepcion_nav.rgbd_localizer_node import build_detection3d
    det2d = _detection2d(make_header()).detections[0]
    det3d = build_detection3d(det2d, (0.0, 0.0, 3.0), (0.5, 0.9), 0.5)
    assert det3d.bbox.size.z == pytest.approx(0.5)
