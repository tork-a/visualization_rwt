#!/usr/bin/python

import roslib
import rospy
import time
import actionlib
roslib.load_manifest("rwt_moveit")

from rwt_moveit.msg import MoveGroupPlan
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
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveGroupActionResult

def joint_position_callback(joints):

    global plan_only
    fixed_frame = rospy.get_param("/fixed_frame")

    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    client.wait_for_server()
    move_group_goal = MoveGroupGoal()

    try:

        msg = MotionPlanRequest()
        workspace_parameters = WorkspaceParameters()
        workspace_parameters.header.stamp = rospy.Time.now()
        workspace_parameters.header.frame_id = fixed_frame
        workspace_parameters.min_corner = Vector3(-1.0, -1.0, -1.0)
        workspace_parameters.max_corner = Vector3(1.0, 1.0, 1.0)

        start_state = RobotState()
#        start_state.joint_state.header.stamp = rospy.Time.now()
        start_state.joint_state.header.frame_id = fixed_frame
        start_state.joint_state.name =  []
        start_state.joint_state.position = []

        cons = Constraints()
        cons.name = ""
        i = 0
        for dim in joints.start_joint.layout.dim:
            start_state.joint_state.name.append(dim.label)
            start_state.joint_state.position.append(joints.start_joint.data[i])

            jc = JointConstraint()
            jc.joint_name = dim.label
            jc.position = joints.goal_joint.data[i]
            jc.tolerance_above = 0.0001
            jc.tolerance_below = 0.0001
            jc.weight = 1.0
            i = i + 1
            cons.joint_constraints.append(jc)


        msg.workspace_parameters = workspace_parameters
        msg.start_state = start_state
        msg.goal_constraints.append(cons)

        msg.num_planning_attempts = 1
        msg.allowed_planning_time = 5.0

        msg.group_name = joints.group_name
        move_group_goal.request = msg


        if joints.plan_only:
            plan_only = True
            move_group_goal.planning_options.plan_only = True
        else:
            plan_only = False

        client.send_goal(move_group_goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))


    except rospy.ROSInterruptException, e:
        print "failed: %s"%e


def moveit_callback(msg):
    pub = rospy.Publisher('/update_joint_position', Float64MultiArray)
    stock_pub = rospy.Publisher('/stock_joint_position', Float64MultiArray)

    global plan_only

    r = rospy.Rate(10)
    names = msg.result.planned_trajectory.joint_trajectory.joint_names
    points = msg.result.planned_trajectory.joint_trajectory.points

    for point in points:
        pos_msg = Float64MultiArray()
        i = 0
        for name in names:
            dim = MultiArrayDimension()
            dim.label = name
            pos_msg.layout.dim.append(dim)
        pos_msg.data = point.positions
        if rospy.get_param('~sim_mode') and plan_only == False:
            pub.publish(pos_msg)
        elif plan_only:
            stock_pub.publish(pos_msg)
        r.sleep()

    if plan_only:
        global robot_trajectory
        robot_trajectory = msg.result.planned_trajectory

def execute_callback(msg):
    global robot_trajectory
    rospy.wait_for_service('execute_kinematic_path')
    try:
        execute_service = rospy.ServiceProxy('execute_kinematic_path', ExecuteKnownTrajectory)
        srv = ExecuteKnownTrajectoryRequest()
        srv.trajectory = robot_trajectory
        response = execute_service(srv)
        print response.error_code
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('moveit_publisher')
    rospy.Subscriber("/execute_trajectory", Empty, execute_callback)
    rospy.Subscriber("moveit_joint", MoveGroupPlan, joint_position_callback)
    rospy.Subscriber("/move_group/result", MoveGroupActionResult, moveit_callback)
    rospy.spin()
