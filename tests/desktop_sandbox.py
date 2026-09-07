"""Short-lived local ROS sandbox for Windows transport checks. Driving is disabled."""
import os
os.environ.update(ROS_MASTER_URI="http://localhost:11311", ROS_HOSTNAME="localhost",
                  VEHICLE_NAME="duck2")
import subprocess
import sys
import time
import xmlrpc.client
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

master=subprocess.Popen(["roscore"],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
children=[]
log=open("/tmp/desktop-sandbox.log","w")
try:
    deadline=time.monotonic()+15
    while True:
        try:
            xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getUri("/desktop_sandbox")
            break
        except OSError:
            if time.monotonic()>deadline:
                raise RuntimeError("ROS master did not start")
            time.sleep(.1)
    rospy.init_node("desktop_sandbox")
    camera=rospy.Publisher("/duck2/camera_node/image/compressed",CompressedImage,queue_size=1)
    base="/code/catkin_ws/src/duckiebot-ros/packages/duckie_lane_follower/src/"
    children.append(subprocess.Popen([sys.executable,base+"lane_follower_node.py",
        "_drive_enabled:=false","_show_debug:=false"],stdout=log,stderr=subprocess.STDOUT))
    children.append(subprocess.Popen([sys.executable,base+"command_gateway.py",
        "_listen_host:=0.0.0.0"],stdout=log,stderr=subprocess.STDOUT))
    image=np.zeros((480,640,3),np.uint8)
    cv2.rectangle(image,(160,240),(180,455),(0,255,255),-1)
    cv2.rectangle(image,(470,240),(490,455),(255,255,255),-1)
    ok,data=cv2.imencode(".jpg",image)
    assert ok
    deadline=time.monotonic()+90
    print("Local desktop sandbox started; driving is disabled",flush=True)
    while not rospy.is_shutdown() and time.monotonic()<deadline:
        if any(p.poll() is not None for p in children):
            raise RuntimeError("A sandbox node stopped unexpectedly")
        message=CompressedImage()
        message.header.stamp=rospy.Time.now()
        message.format,message.data="jpeg",data.tobytes()
        camera.publish(message)
        time.sleep(.05)
finally:
    for child in children:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=10)
    if master.poll() is None:
        master.terminate()
        master.wait(timeout=10)
    log.close()
