#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64
from random import random
def main():
    rospy.init_node("random_float")
    pub = rospy.Publisher("/random_float", Float64)
    while not rospy.is_shutdown():
        val = random() * 100
        pub.publish(Float64(val))
        rospy.sleep(0.05)


if __name__ == "__main__":
    main()
