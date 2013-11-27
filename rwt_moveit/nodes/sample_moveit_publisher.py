#!/usr/bin/python

import roslib
import rospy
import time
import actionlib
from moveit_msgs.srv import *
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import WorkspaceParameters
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import MoveGroupAction
from moveit_msgs.msg import MoveGroupGoal
from moveit_msgs.msg import MoveGroupActionGoal
from moveit_msgs.msg import MotionPlanRequest
from geometry_msgs.msg import Vector3


if __name__ == '__main__':
    rospy.init_node('moveit_publisher')
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    client.wait_for_server()
    move_group_goal = MoveGroupGoal()

    try:
        msg = MotionPlanRequest()

        workspace_parameters = WorkspaceParameters()
        workspace_parameters.header.stamp = rospy.Time.now()
        workspace_parameters.header.frame_id = "/BASE"
        workspace_parameters.min_corner = Vector3(-1.0, -1.0, -1.0)
        workspace_parameters.max_corner = Vector3(1.0, 1.0, 1.0)

        start_state = RobotState()
#        start_state.joint_state.header.stamp = rospy.Time.now()
        start_state.joint_state.header.frame_id = "/BASE"
        start_state.joint_state.name =  ["j1", "j2", "j3", "j4", "j5", "flange"]
        start_state.joint_state.position = [-0.2569046038066598, -0.8442722962923348, 1.849082034218144, 0.26825374068443164, -0.04090683809444329, 5.745512865657193]
        start_state.joint_state.velocity =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        jc2 = JointConstraint()
        jc2.joint_name = "j1"
        jc2.position = -1.37353344093
        jc2.tolerance_above = 0.0001
        jc2.tolerance_below = 0.0001
        jc2.weight = 1.0

        jc3 = JointConstraint()
        jc3.joint_name = "j2"
        jc3.position = -1.45013378543
        jc3.tolerance_above = 0.0001
        jc3.tolerance_below = 0.0001
        jc3.weight = 1.0

        jc4 = JointConstraint()
        jc4.joint_name = "j3"
        jc4.position = 2.18173030842
        jc4.tolerance_above = 0.0001
        jc4.tolerance_below = 0.0001
        jc4.weight = 1.0

        jc5 = JointConstraint()
        jc5.joint_name = "j4"
        jc5.position = 1.46043924358
        jc5.tolerance_above = 0.0001
        jc5.tolerance_below = 0.0001
        jc5.weight = 1.0

        jc1 = JointConstraint()
        jc1.joint_name = "j5"
        jc1.position = 1.25752837976
        jc1.tolerance_above = 0.0001
        jc1.tolerance_below = 0.0001
        jc1.weight = 1.0

        jc0 = JointConstraint()
        jc0.joint_name = "flange"
        jc0.position = -4.43859335239
        jc0.tolerance_above = 0.0001
        jc0.tolerance_below = 0.0001
        jc0.weight = 1.0

        cons = Constraints()
        cons.name = ""
        cons.joint_constraints.append(jc2)
        cons.joint_constraints.append(jc3)
        cons.joint_constraints.append(jc4)
        cons.joint_constraints.append(jc5)
        cons.joint_constraints.append(jc1)
        cons.joint_constraints.append(jc0)

        msg.workspace_parameters = workspace_parameters
        msg.start_state = start_state
        msg.goal_constraints.append(cons)

        msg.num_planning_attempts = 1
        msg.allowed_planning_time = 5.0

        msg.group_name = "manipulator"

        move_group_goal.request = msg

        client.send_goal(move_group_goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))


    except rospy.ROSInterruptException, e:
        print "failed: %s"%e

