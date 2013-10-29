#!/usr/bin/env python

import rospy
import roslib

import os
import subprocess
roslib.load_manifest("rwt_image_view")

from rwt_image_view.srv import *
from std_srvs.srv import *
import signal

g_pobj = None

def rosbagRecordRequestCB(req):
    global g_pobj
    rospy.loginfo("topics: " + str(req.topics))
    record_path = roslib.packages.find_node("rosbag", "record")
    args = [record_path[0], "-O", os.path.join(os.path.dirname(__file__), "..", "www", "tmp.bag")] + req.topics
    g_pobj = subprocess.Popen(args)
    return RosbagRecordRequestResponse()

def rosbagRecordStopCB(req):
    global g_pobj
    if g_pobj:
        rospy.loginfo("sending SIGINT")
        g_pobj.send_signal(signal.SIGINT)
        #os.kill(g_pobj.pid, signal.SIGINT)
        g_pobj = None
    return EmptyResponse()

def main():
    rospy.init_node("rosbag_record_server")
    s1 = rospy.Service('rosbag_record', RosbagRecordRequest, rosbagRecordRequestCB)
    s2 = rospy.Service('rosbag_record_stop', Empty, rosbagRecordStopCB)
    rospy.spin()

if __name__ == "__main__":
   main()
