#!/usr/bin/env python

import roslib
import rospy
import math
from std_msgs.msg import Float64
from random import random
def main():
    rospy.init_node("sin")
    pub = rospy.Publisher("/sin", Float64, queue_size=1)
    counter = 0
    RESOLUTION = 100
    while not rospy.is_shutdown():
        if counter == RESOLUTION:
            counter = 0
        val = math.sin(2 * math.pi / RESOLUTION * counter)
        pub.publish(Float64(val))
        rospy.sleep(0.05)
        counter = counter + 1


if __name__ == "__main__":
    main()
