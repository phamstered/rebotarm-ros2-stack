"""Check the camera launch contract without requiring a physical camera."""

import ast
from pathlib import Path
import unittest


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch/left_wrist_camera.launch.py"


class LeftWristCameraLaunchTests(unittest.TestCase):
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
