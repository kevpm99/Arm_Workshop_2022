#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
# import geometry_msgs.msg
import aruco_msgs.msg
# from math import pi
from math import fabs
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
import control_msgs.msg
import numpy as np
import quaternion

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
rel_pos = aruco_msgs.msg.MarkerArray()

###########################################################

def pose_callback(data):
    
    global rel_pos
    if data.markers:
        rel_pos = data.markers[0].pose.pose

###########################################################

def status_callback (data):

    if not any(data.desired.velocities):
        
        global rel_pos
        curr_pos = move_group.get_current_pose().pose
        new_pos = curr_pos
        rel_or = rel_pos.orientation
        rel_or = np.quaternion(rel_or.w,rel_or.x,rel_or.y,rel_or.z)
        transform = quaternion.as_rotation_matrix(rel_or)

        rel_pos.position.z -= 0.30
        
        change_in_coords = np.matmul(np.linalg.inv(transform), np.array([[rel_pos.position.x,rel_pos.position.y,rel_pos.position.z]]).T)
        change_in_coords = change_in_coords * 0.01

        # import pdb; pdb.set_trace()
        print(rel_pos.position)
        checkx = fabs(rel_pos.position.x)>0.01
        checky = fabs(rel_pos.position.y)>0.01
        checkz = rel_pos.position.z>0.30 or rel_pos.position.z<0.29

        if checky or checkx or checkz:
                if checkz:
                    new_pos.position.x -= change_in_coords[0][0]
                if checky:
                    new_pos.position.y += change_in_coords[1][0]
                if checkx:
                    new_pos.position.z += change_in_coords[2][0]

                # import pdb; pdb.set_trace()

                # move_group.set_pose_target(new_pos)
                # plan = move_group.go(wait=True)
                waypoints = []
                waypoints.append(copy.deepcopy(new_pos))

                (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
                move_group.execute(plan)

###########################################################


def listener():
    rospy.init_node('aruco_pose_listener', anonymous=True)

    rospy.Subscriber("/aruco_marker_publisher/markers", aruco_msgs.msg.MarkerArray, pose_callback)
    rospy.Subscriber("/arm_controller/state", control_msgs.msg.JointTrajectoryControllerState, status_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
