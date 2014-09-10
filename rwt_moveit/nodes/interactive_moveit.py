#!/usr/bin/python

import roslib
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import *
from moveit_msgs.msg import RobotState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import JointState
from visualization_msgs.msg import (
    Marker,
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
server = None


def makeInteractiveMarker(name, description):
    global fixed_frame
    interactive_marker = InteractiveMarker()
    interactive_marker.header.frame_id = fixed_frame
    interactive_marker.name = name
    interactive_marker.description = description
    return interactive_marker


def makeInteractiveMarkerControl(interactive_marker, mode):
    interactive_marker_control = InteractiveMarkerControl()
    interactive_marker_control.always_visible = True
    interactive_marker_control.interaction_mode = mode
    interactive_marker.controls.append(interactive_marker_control)
    return interactive_marker_control

def setOrientation(w, x, y, z, marker):
    marker.orientation.w = w
    marker.orientation.x = x
    marker.orientation.y = y
    marker.orientation.z = z

def setColor(red, green, blue, alpha, marker):
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = alpha


def callback(req):
    print req
    return GetPositionIKResponse

def initial_callback(msg):
    global joint_states,joint_names, initial_joint_position, group_name
    group_name = msg.data
    joint_names = rospy.get_param("/link_group/" + group_name)
    initial_joint_position = [0] * len(joint_names)

def im_size_callback(msg):
    global interactive_marker
    interactive_marker.scale = msg.data
    server.clear()
    server.insert(interactive_marker, feedback)
    server.applyChanges()


def joint_state_callback(msg):
    global joint_states
    joint_states = msg

def feedback(feedback):
    global pub, initial_joint_position, fixed_frame, joint_names, group_name

    server.setPose(feedback.marker_name, feedback.pose)
    server.applyChanges()

    if feedback.event_type == 0:
        return

    rospy.wait_for_service('compute_ik')
    try:
        service = rospy.ServiceProxy('compute_ik', GetPositionIK)
        request = GetPositionIKRequest()
        request.ik_request.group_name = group_name
        request.ik_request.timeout = rospy.Duration.from_sec(0.0001)

        # initial robot state
        robot_state = RobotState()
        robot_state.joint_state.header.frame_id = fixed_frame
        robot_state.joint_state.name =  joint_names
        robot_state.joint_state.position = initial_joint_position
        robot_state.joint_state.velocity =  []
        request.ik_request.robot_state = robot_state

        # goal end pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = fixed_frame
        pose_stamped.pose.position.x = feedback.pose.position.x
        pose_stamped.pose.position.y = feedback.pose.position.y
        pose_stamped.pose.position.z = feedback.pose.position.z
        pose_stamped.pose.orientation.x = feedback.pose.orientation.x
        pose_stamped.pose.orientation.y = feedback.pose.orientation.y
        pose_stamped.pose.orientation.z = feedback.pose.orientation.z
        pose_stamped.pose.orientation.w = feedback.pose.orientation.w

        request.ik_request.pose_stamped = pose_stamped
        response = service(request)
        print response
        if len(response.solution.joint_state.position) != 0:
            print "success"
            msg = Float64MultiArray()
            for i,joint_name in enumerate(response.solution.joint_state.name):
                for j, name in enumerate(joint_names):
                    if joint_name == name:
                        initial_joint_position[j] = response.solution.joint_state.position[i]
                        dim = MultiArrayDimension()
                        dim.label = name
                        msg.layout.dim.append(dim)
                        msg.data.append(response.solution.joint_state.position[i])
            pub.publish(msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    rospy.init_node("moveit_interactive_marker")
    global pub, fixed_frame, interactive_marker
    fixed_frame = rospy.get_param("/fixed_frame")
    prefix = rospy.get_param("~prefix")

    pub = rospy.Publisher('update_' + prefix + '_joint_position',Float64MultiArray)
    rospy.Subscriber('/' + prefix + '/initial_marker', String, initial_callback)
    rospy.Subscriber('/' + prefix + '_joint_states', JointState, joint_state_callback)
    rospy.Subscriber('/im_size/update', Float32, im_size_callback)

    server = InteractiveMarkerServer("/" + prefix + "/marker")

    interactive_marker = makeInteractiveMarker(prefix, "")
    interactive_marker.scale = 0.3
    interactive_marker.pose.position.x = 0
    interactive_marker.pose.position.y = 0
    interactive_marker.pose.position.z = 1.0

    control_slide_x = makeInteractiveMarkerControl(
        interactive_marker, InteractiveMarkerControl.MOVE_AXIS)
    control_slide_y = makeInteractiveMarkerControl(
        interactive_marker, InteractiveMarkerControl.MOVE_AXIS)
    control_slide_z = makeInteractiveMarkerControl(
        interactive_marker, InteractiveMarkerControl.MOVE_AXIS)
    control_rotate_x = makeInteractiveMarkerControl(
        interactive_marker, InteractiveMarkerControl.ROTATE_AXIS)
    control_rotate_y = makeInteractiveMarkerControl(
        interactive_marker, InteractiveMarkerControl.ROTATE_AXIS)
    control_rotate_z = makeInteractiveMarkerControl(
        interactive_marker, InteractiveMarkerControl.ROTATE_AXIS)
    control_sphere = makeInteractiveMarkerControl(
        interactive_marker, InteractiveMarkerControl.MOVE_ROTATE_3D)
    marker = Marker()
    marker.color.r = 0.2
    marker.color.g = 0.3
    marker.color.b = 0.7
    marker.color.a = 0.5

    marker.type = Marker.SPHERE
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    control_sphere.markers.append(marker)

    setOrientation(1,1,0,0, control_slide_x)
    setOrientation(1,0,1,0, control_slide_y)
    setOrientation(1,0,0,1, control_slide_z)
    setOrientation(1,1,0,0, control_rotate_x)
    setOrientation(1,0,1,0, control_rotate_y)
    setOrientation(1,0,0,1, control_rotate_z)
    setOrientation(1,1,0,0, control_sphere)


    server.insert(interactive_marker, feedback)
    server.applyChanges()
    rospy.spin()
