"""Road calibration must not inherit the dim-junction centroid offset."""
import unittest
from unittest.mock import patch
import numpy as np
import test_lane_follower as fixtures
from test_camera_gateway import load_gateway


class RoadWhiteReferenceTests(unittest.TestCase):
    setUp = fixtures.LaneTests.setUp
    subscribe = fixtures.LaneTests.subscribe
    message = fixtures.LaneTests.message

    def scene(self):
        image = np.zeros((480, 640, 3), np.uint8)
        image[240:456, 40:60] = (0, 255, 255)
        image[240:456, 440:460] = (210, 210, 210)
        image[240:456, 460:610] = (160, 160, 160)
        return image

    def calibrated(self):
        n = self.node
        n.white_lower = np.array([0, 0, 150], np.uint8)
        n.road_white_reference_value = 170
        n.lane_target_fraction = .441
        n.drive_enabled = True
        n.manual_stop = False
        n.navigation_state = 'following'
        return n

    def test_bright_reference_restores_left_request_and_keeps_junction_geometry(self):
        n = self.calibrated()
        n.road_white_reference_value = 150
        old, _, _ = n.detect_lane_bgr(self.scene())
        geometry = n._junction_lane_geometry
        n.road_white_reference_value = 170
        new, _, _ = n.detect_lane_bgr(self.scene())
        self.assertGreater(old, 0)
        self.assertLess(new, 0)
        self.assertTrue(n._road_white_reference_used)
        self.assertEqual(geometry, n._junction_lane_geometry)
        for _ in range(40):
            self.now += .05
            wheels = n.compute_wheel_speeds(new)
        self.assertGreater(wheels[1], wheels[0])

    def test_dim_only_border_remains_available(self):
        n = self.calibrated()
        image = self.scene()
        image[240:456, 440:460] = (160, 160, 160)
        error, _, _ = n.detect_lane_bgr(image)
        self.assertIsNotNone(error)
        self.assertTrue(n._white_boundary_visible)
        self.assertFalse(n._road_white_reference_used)

    def test_incomplete_lane_is_not_invented(self):
        n = self.calibrated()
        n.temporal_lane_width_fallback = True
        image = self.scene()
        image[:, :100] = 0
        error, _, _ = n.detect_lane_bgr(image)
        self.assertIsNone(error)
        self.assertFalse(n._road_white_reference_used)
        self.assertEqual(n.compute_wheel_speeds(error)[:2], (0, 0))

    def test_existing_junction_and_corner_phases_keep_broad_calibration(self):
        n = self.calibrated()
        n.road_white_reference_value = 150
        baseline, _, _ = n.detect_lane_bgr(self.scene())
        n.road_white_reference_value = 170
        for name, value in [('navigation_state', 'crossing'),
                            ('navigation_state', 'reacquiring'),
                            ('_junction_phase', 'settling'),
                            ('_red_line_visible', True),
                            ('_junction_approach_active', True),
                            ('_sharp_corner_state', 'turning')]:
            with self.subTest(field=name, value=value):
                old = getattr(n, name)
                setattr(n, name, value)
                error, _, _ = n.detect_lane_bgr(self.scene())
                self.assertAlmostEqual(error, baseline)
                self.assertFalse(n._road_white_reference_used)
                setattr(n, name, old)

    def test_stopped_start_preview_has_same_road_reference(self):
        n = self.calibrated()
        expected = n.detect_lane_bgr(self.scene())[0]
        n.navigation_state = 'route_complete'
        self.assertEqual(n.detect_lane_bgr(self.scene())[0], expected)
        gateway = load_gateway(self.mod)
        preview = gateway.LanePreview()
        preview.white_lower = n.white_lower.copy()
        preview.road_white_reference_value = 170
        preview.lane_target_fraction = n.lane_target_fraction
        self.assertAlmostEqual(preview.detect_lane_bgr(self.scene())[0], expected)

    def test_invalid_reference_rejected(self):
        for value in (149, 256, float('nan'), True):
            self.params.update({'~white_lower': [0, 0, 150],
                                '~road_white_reference_value': value})
            with self.subTest(value=value), patch.dict(self.mod.os.environ, VEHICLE_NAME='duck2'):
                with self.assertRaises(ValueError):
                    self.mod.LaneFollowerNode('invalid')


if __name__ == '__main__':
    unittest.main()
