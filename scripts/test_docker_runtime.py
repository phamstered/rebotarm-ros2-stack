"""Exercise runtime command construction without Docker or ROS installed."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).with_name('docker-runtime.sh')

class RuntimeTests(unittest.TestCase):
    def invoke(self, args, **settings):
        with tempfile.TemporaryDirectory() as directory:
            docker = Path(directory) / 'docker'
            docker.write_text('#!/usr/bin/env python3\nimport sys,json\nprint(json.dumps(sys.argv[1:]))\n')
            docker.chmod(0o755)
            env = {k: v for k, v in os.environ.items() if not k.startswith(('STACK_DOCKER_', 'ROBOT_EXTERNAL_', 'ROS_DOMAIN_ID'))}
            env.update(settings)
            env['PATH'] = directory + os.pathsep + env['PATH']
            return subprocess.run(['bash', str(SCRIPT), *args], env=env, text=True, capture_output=True)

    def test_command_and_domain(self):
        result = self.invoke(['run', 'bash', '-c', 'echo "$HOME"'], ROS_DOMAIN_ID='221')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = json.loads(result.stdout)
        self.assertEqual(args[-3:], ['bash', '-c', 'echo "$HOME"'])
        self.assertIn('ROS_DOMAIN_ID=221', args)
        self.assertEqual(args[args.index('--network') + 1], 'host')
        self.assertNotIn('--privileged', args)
        self.assertNotIn('--pid', args)

    def test_privileged_opt_in(self):
        result = self.invoke(['run', 'true'], STACK_DOCKER_PRIVILEGED='1')
        self.assertIn('--privileged', json.loads(result.stdout))

    def test_invalid_config_and_missing_command(self):
        for settings in ({'ROS_DOMAIN_ID': '233'}, {'ROS_DOMAIN_ID': '-1'}, {'ROS_DOMAIN_ID': 'bad'}, {'STACK_DOCKER_PRIVILEGED': 'yes'}, {'ROBOT_EXTERNAL_SETUP': '/missing'}):
            with self.subTest(settings=settings):
                self.assertNotEqual(self.invoke(['run', 'true'], **settings).returncode, 0)
        self.assertNotEqual(self.invoke(['run']).returncode, 0)

if __name__ == '__main__':
    unittest.main()
