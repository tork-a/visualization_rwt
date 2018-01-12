#!/usr/bin/env python

import roslib
import rospy
import math
from std_msgs.msg import Float64MultiArray
from random import random
def main():
    rospy.init_node("array_sin_cos")
    pub = rospy.Publisher("/array_sin_cos", Float64MultiArray, queue_size=1)
    counter = 0
    RESOLUTION = 100
    while not rospy.is_shutdown():
        if counter == RESOLUTION:
            counter = 0
        arr = Float64MultiArray()
        arr.data = [math.sin(2 * math.pi / RESOLUTION * counter),
                    math.cos(2 * math.pi / RESOLUTION * counter),
                    -math.sin(2 * math.pi / RESOLUTION * counter),
                    -math.cos(2 * math.pi / RESOLUTION * counter)]
        pub.publish(arr)
        rospy.sleep(0.05)
        counter = counter + 1


if __name__ == "__main__":
    main()

