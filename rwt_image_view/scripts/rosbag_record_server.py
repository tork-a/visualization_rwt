#!/usr/bin/env python

import rospy
import roslib

roslib.load_manifest("rwt_image_view")

from rwt_image_view.srv import *

def rosbagRecordRequestCB(req):
    rospy.loginfo("topics: " + str(req.topics))
    return RosbagRecordRequestResponse("foo")
    

def main():
    rospy.init_node("rosbag_record_server")
    s = rospy.Service('rosbag_record', RosbagRecordRequest, rosbagRecordRequestCB)
    rospy.spin()

if __name__ == "__main__":
   main()
