"""Offline build/inspection checks and inspected ROS message compatibility."""
import base64
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tools"))
import build_local
import inspect_robot_setup


class SetupTests(unittest.TestCase):
    def test_pinned_runtime_references(self):
        config=json.loads((ROOT/"config/duck2.json").read_text())
        for ref in config["base_images"].values():
            self.assertRegex(ref,r"^duckietown/dt-ros-commons:v4\.3\.0-(amd64|arm64v8)@sha256:[0-9a-f]{64}$")

    def test_remote_docker_host_is_rejected_before_any_call(self):
        with patch.object(subprocess,"run",side_effect=AssertionError("No remote Docker call")):
            with self.assertRaises(ValueError):
                build_local.local_docker(["docker"],{"DOCKER_HOST":"tcp://192.168.137.222:2375"})

    def test_remote_context_is_rejected(self):
        with patch.object(subprocess,"run",return_value=NS(stdout="robot ssh://duckie@duck2.local")):
            with self.assertRaises(ValueError):
                build_local.local_docker(["docker"],{})

    def test_local_context_is_accepted(self):
        with patch.object(subprocess,"run",return_value=NS(stdout="desktop-linux npipe:////./pipe/dockerDesktopLinuxEngine")):
            self.assertEqual(build_local.local_docker(["docker"],{}),"desktop-linux")

    def test_inspection_default_is_offline(self):
        with patch.object(sys,"argv",["inspect_robot_setup.py"]), contextlib.redirect_stdout(io.StringIO()) as output:
            with patch.object(subprocess,"run",side_effect=AssertionError("No SSH without --connect")):
                inspect_robot_setup.main()
        self.assertEqual(json.loads(output.getvalue())["vehicle_name"],"duck2")

    def test_ssh_argument_injection_rejected(self):
        for host in ("-oProxyCommand=x","duck2;touch /tmp/x","a b","$(id)"):
            with self.assertRaises(ValueError):
                inspect_robot_setup.ssh_arguments(host,"duckie")

    def test_inspection_payload_is_fixed_metadata_only(self):
        args=inspect_robot_setup.ssh_arguments("192.168.137.222","duckie")
        payload=args[-1].split()[1]
        source=base64.b64decode(payload).decode()
        self.assertEqual(source,inspect_robot_setup.REMOTE)
        for forbidden in ("rostopic echo","rostopic pub","rostopic hz","rosrun",
                          "roslaunch","docker run","docker stop","Subscriber","Publisher"):
            self.assertNotIn(forbidden,source)

    @unittest.skipUnless(os.environ.get("ROS_DISTRO"),"Run inside sourced Noetic image")
    def test_message_definitions_match_inspected_robot(self):
        config=json.loads((ROOT/"config/duck2.json").read_text())
        self.assertEqual(os.environ["ROS_DISTRO"],config["ros_distro"])
        for key in ("camera","wheels"):
            result=subprocess.check_output(["rosmsg","md5",config[key]["type"]],text=True).strip()
            self.assertEqual(result,config[key]["md5"])
        import cv2
        import numpy
        from cv_bridge import CvBridge
        from duckietown.dtros import DTROS
        self.assertEqual(cv2.__version__,config["opencv_version"])
        self.assertEqual(numpy.__version__,config["numpy_version"])
        self.assertTrue(callable(CvBridge))
        self.assertTrue(callable(DTROS))


if __name__=="__main__":
    unittest.main()
