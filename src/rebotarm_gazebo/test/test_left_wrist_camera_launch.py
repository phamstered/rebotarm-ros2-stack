"""Check the camera launch contract without requiring a physical camera."""

import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch/left_wrist_camera.launch.py"
URDF_FILE = LAUNCH_FILE.parents[1] / "urdf/rebotarm.urdf"


class LeftWristCameraLaunchTests(unittest.TestCase):
    def camera_mount(self):
        joint = ET.parse(URDF_FILE).find("joint[@name='camera_mount_joint']")
        self.assertEqual(joint.find("parent").get("link"), "link6")
        self.assertEqual(joint.find("child").get("link"), "camera_link")
        origin = joint.find("origin")
        roll, pitch, yaw = map(float, origin.get("rpy").split())
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation = (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
        return origin, rotation

    def test_mount_matches_exported_hand_eye_quaternion(self):
        # Active arguments of the 2026-09-12 MoveIt calibration export.
        # Its commented-out Euler values are NOT equivalent to this quaternion.
        quaternion = (0.598366, 0.00397963, 0.801167, 0.00856162)
        norm = math.sqrt(sum(value * value for value in quaternion))
        x, y, z, w = (value / norm for value in quaternion)
        expected = (
            (1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
            (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
            (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)),
        )
        origin, actual = self.camera_mount()
        for actual_row, expected_row in zip(actual, expected):
            for actual_value, expected_value in zip(actual_row, expected_row):
                self.assertAlmostEqual(actual_value, expected_value, places=10)
        for actual_value, expected_value in zip(
            map(float, origin.get("xyz").split()),
            (0.0520795, -0.0139823, 0.0672182),
        ):
            self.assertAlmostEqual(actual_value, expected_value, places=10)

    def test_positive_optical_depth_points_ahead_of_wrist(self):
        _, rotation = self.camera_mount()
        # RealSense optical +Z is camera body +X. The gripper extends along
        # link6 +Z; a one-metre central depth ray must point toward that side.
        forward = tuple(row[0] for row in rotation)
        self.assertGreater(forward[2], 0.95)
        self.assertAlmostEqual(sum(value * value for value in forward), 1.0)

    def test_camera_identity_alignment_and_mount_contract(self):
        source = LAUNCH_FILE.read_text()
        ast.parse(source)
        for value in (
            '"camera_namespace": "cameras/left_arm"',
            '"camera_name": "wrist"',
            '"tf_prefix": "left_arm/"',
            '"align_depth.enable": "true"',
            '"enable_sync": "true"',
            '"enable_color": "true"',
            '"enable_depth": "true"',
            '"--frame-id", "camera_link"',
            '"--child-frame-id", "left_arm/wrist_link"',
        ):
            self.assertIn(value, source)
        self.assertIn('"serial_no": serial_no', source)


if __name__ == "__main__":
    unittest.main()
