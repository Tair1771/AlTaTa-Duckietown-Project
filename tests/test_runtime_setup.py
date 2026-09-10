"""Offline build/inspection checks and inspected ROS message compatibility."""
import base64
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
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
                build_local.local_docker(["docker"],{"DOCKER_HOST":"tcp://192.0.2.1:2375"})

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

    def test_quick_inspection_payload_is_fixed_and_read_only(self):
        args=inspect_robot_setup.ssh_arguments("192.0.2.1","duckie","quick")
        payload=args[-1].split()[1]
        source=base64.b64decode(payload).decode()
        self.assertEqual(source,inspect_robot_setup.QUICK_REMOTE)
        for forbidden in ("rostopic echo","rostopic pub","rostopic hz","rosrun",
                          "roslaunch","docker run","docker stop","Subscriber","Publisher"):
            self.assertNotIn(forbidden,source)
        self.assertIn("wheels_cmd",source)
        self.assertIn("universal_newlines=True", source)
        self.assertNotIn("text=True", source)

    def test_full_inspection_payload_contains_compatibility_metadata(self):
        args=inspect_robot_setup.ssh_arguments("duck2.local","duckie","full")
        source=base64.b64decode(args[-1].split()[1]).decode()
        self.assertEqual(source,inspect_robot_setup.FULL_REMOTE)
        self.assertIn("rosmsg md5",source)

    def test_ssh_check_is_batch_mode_with_no_password_prompt(self):
        args=inspect_robot_setup.ssh_arguments("duck2.local","duckie")
        self.assertIn("BatchMode=yes",args)
        self.assertIn("NumberOfPasswordPrompts=0",args)

    def test_failure_messages_are_actionable(self):
        self.assertEqual(inspect_robot_setup.classify_failure("Host key verification failed")[0],"host identity")
        self.assertEqual(inspect_robot_setup.classify_failure("Permission denied (publickey)")[0],"login")
        self.assertEqual(inspect_robot_setup.classify_failure("Connection timed out")[0],"network")
        self.assertEqual(inspect_robot_setup.classify_failure("Could not resolve hostname")[0],"name resolution")

    def test_name_resolution_failure_does_not_start_ssh(self):
        with patch.object(inspect_robot_setup,"resolve",return_value=([], "not found")):
            with patch.object(subprocess,"run",side_effect=AssertionError("SSH should not start")):
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    code=inspect_robot_setup.run_check("duck2.local","duckie","quick",None,False)
        self.assertEqual(code,2)
        self.assertIn("name resolution failed",output.getvalue())

    def test_login_failure_does_not_retry(self):
        failed=NS(returncode=255,stdout="duckie@duck2: Permission denied (publickey).")
        with patch.object(inspect_robot_setup,"resolve",return_value=(["192.0.2.1"],None)):
            with patch.object(subprocess,"run",return_value=failed) as run:
                with contextlib.redirect_stdout(io.StringIO()) as output:
                    code=inspect_robot_setup.run_check("duck2.local","duckie","quick","duck2",False)
        self.assertEqual(code,255)
        self.assertEqual(run.call_count,1)
        self.assertIn("Unlock the duck2 SSH key",output.getvalue())

    def test_changed_runtime_identity_is_detected_in_local_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            location=Path(directory)
            first="ros image-a Up\nduckiebot-interface image-a Up\ncar-interface image-a Up"
            second="ros image-b Up\nduckiebot-interface image-a Up\ncar-interface image-a Up"
            with patch.object(inspect_robot_setup,"cache_directory",return_value=location):
                _,changed=inspect_robot_setup.save_summary("duck2.local",["192.0.2.1"],"quick",first)
                _,changed_after=inspect_robot_setup.save_summary("duck2.local",["192.0.2.1"],"quick",second)
        self.assertFalse(changed)
        self.assertTrue(changed_after)

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
