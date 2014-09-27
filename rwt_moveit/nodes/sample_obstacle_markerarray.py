#!/usr/bin/env python

# Taken from:
# http://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/?answer=16587#post-id-16587

import math

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

topic = '/obst/markerarray'

publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('sample_obstacle')

count = 0
MARKERS_MAX = 2  # Up to two boxes.

marker_box_1 = Marker()
marker_box_1.header.frame_id = "/BASE"
marker_box_1.type = marker_box_1.CUBE
marker_box_1.action = marker_box_1.ADD
marker_box_1.scale.x = 0.2
marker_box_1.scale.y = 0.2
marker_box_1.scale.z = 0.2
marker_box_1.color.a = 1.0
marker_box_1.color.r = 0.0
marker_box_1.color.g = 0.3
marker_box_1.color.b = 0.5
marker_box_1.pose.orientation.w = 1.0
marker_box_1.pose.position.x = 0.15
marker_box_1.pose.position.y = 0.2
marker_box_1.pose.position.z = 0.5
#    publisher.publish(marker_box_1)
    
marker_box_2 = Marker()
marker_box_2.header.frame_id = "/BASE"
marker_box_2.type = marker_box_2.CUBE
marker_box_2.action = marker_box_2.ADD
marker_box_2.scale.x = 0.2
marker_box_2.scale.y = 0.2
marker_box_2.scale.z = 0.2
marker_box_2.color.a = 1.0
marker_box_2.color.r = 0.0
marker_box_2.color.g = 0.3
marker_box_2.color.b = 0.5
marker_box_2.pose.orientation.w = 1.0
marker_box_2.pose.position.x = 0.15
marker_box_2.pose.position.y = 0.2
marker_box_2.pose.position.z = 0.15
#    publisher.publish(marker_box_2)

    # We add the new marker to the MarkerArray, removing the oldest
    # marker from it when necessary
#    if(count > MARKERS_MAX):
#        markerArray.markers.pop(0)

markerArray = MarkerArray()
markerArray.markers.append(marker_box_1)
markerArray.markers.append(marker_box_2)

while not rospy.is_shutdown():
    publisher.publish(markerArray)
    rospy.sleep(0.01)

# markerArray = MarkerArray()
# 
# count = 0
# MARKERS_MAX = 100
# 
# while not rospy.is_shutdown():
# 
#    marker = Marker()
#    marker.header.frame_id = "/BASE"
#    marker.type = marker.SPHERE
#    marker.action = marker.ADD
#    marker.scale.x = 0.2
#    marker.scale.y = 0.2
#    marker.scale.z = 0.2
#    marker.color.a = 1.0
#    marker.color.r = 1.0
#    marker.color.g = 1.0
#    marker.color.b = 0.0
#    marker.pose.orientation.w = 1.0
#    marker.pose.position.x = math.cos(count / 50.0)
#    marker.pose.position.y = math.cos(count / 40.0) 
#    marker.pose.position.z = math.cos(count / 30.0) 
# 
#    # We add the new marker to the MarkerArray, removing the oldest
#    # marker from it when necessary
#    if(count > MARKERS_MAX):
#        markerArray.markers.pop(0)
# 
#    markerArray.markers.append(marker)
# 
#    # Renumber the marker IDs
#    id = 0
#    for m in markerArray.markers:
#        m.id = id
#        id += 1
# 
#    # Publish the MarkerArray
#    publisher.publish(markerArray)
# 
#    count += 1
# 
#    rospy.sleep(0.01)
# 
