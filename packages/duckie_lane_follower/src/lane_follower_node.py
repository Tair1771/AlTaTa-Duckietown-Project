#!/usr/bin/env python3

"""
Lane follower node adapted from the local Gym-Duckietown simulation script.

This version is for the real Duckiebot:
- subscribes to the Duckiebot camera topic
- runs the OpenCV lane detector
- computes left/right wheel speeds
- publishes WheelsCmdStamped commands

It deliberately does not import gym_duckietown, pyglet, Simulator, or manual_update().
"""

import os
import copy
import json
import math
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


import sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parent))
from duck2_core import AutonomyCore, MAP_PORTS, HEADINGS, validate_route, junction_turn, route_via_turn

class LaneFollowerNode(AutonomyCore, DTROS):
    def __init__(self, node_name):
        DTROS.__init__(self, node_name=node_name, node_type=NodeType.PERCEPTION)
        runtime = SimpleNamespace(**{name: getattr(rospy, name) for name in
            ('Publisher', 'Subscriber', 'get_param', 'on_shutdown', 'Timer',
             'Duration', 'Time', 'loginfo', 'logwarn')})
        runtime.CvBridge = CvBridge
        runtime.CvBridgeError = CvBridgeError
        runtime.WheelsCmdStamped = WheelsCmdStamped
        runtime.CompressedImage = CompressedImage
        runtime.String = String
        AutonomyCore.__init__(self, node_name, runtime, time)

if __name__ == '__main__':
    node = LaneFollowerNode(node_name='lane_follower_node')
    rospy.spin()
