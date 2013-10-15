#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import PointStamped
from random import random
def main():
    rospy.init_node("random_point")
    pub = rospy.Publisher("/random_point", PointStamped)
    while not rospy.is_shutdown():
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.point.x = random() * 100;
        msg.point.y = random() * 100;
        msg.point.z = random() * 100;
        pub.publish(msg)
        rospy.sleep(0.1 * random())


if __name__ == "__main__":
    main()
    
