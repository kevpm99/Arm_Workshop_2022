#!/usr/bin/env python
import sys
import rospy
import geometry_msgs.msg
import moveit_commander
from std_msgs.msg import String
from aruco_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
import copy


def approach(msg, scale=1):
	
	gz=msg.pose.pose.position.z
	
	waypoints = []
	wpose = move_group.get_current_pose().pose
	print gz, wpose.position.z
    	wpose.position.z -= scale * (gz)  # Add coefficint depending on type of grasp		
    	waypoints.append(copy.deepcopy(wpose))
	
    	(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)# waypoints to follow # eef_step # jump_threshold
	move_group.execute(plan, wait=True)
	rospy.sleep(1)
	sys.exit(0)                        	

def gripper(msg):
	print("OPERATING_GRIPPER")
	pub = rospy.Publisher('/gripper_command',String,queue_size=10)
	for i in range(5):
		pub.publish(msg)
		rate.sleep()
	return

def align(groll):
	print("Aligning")
	joint_goal = move_group.get_current_joint_values()
	joint_goal[5] += groll/3
	
	move_group.go(joint_goal, wait=True)
	move_group.stop()
	

def get_rotation(msgl):

    global roll, pitch, yaw , groll, gpitch , gyaw

    orientation_curr=move_group.get_current_pose().pose.orientation
    orientation_currlist = [orientation_curr.x, orientation_curr.y, orientation_curr.z, orientation_curr.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_currlist)
    found=False
    for m in msgl.markers:
	if m.id==10:
		msg=m
		found=True
		break
	if(not found):
		return

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (groll, gpitch, gyaw) = euler_from_quaternion (orientation_list)
    print gpitch, groll, gyaw
    if abs(groll) > 0.05:
    	align(groll)
    else:
	print("!!ALIGNED!!")
	approach(msg)
	gripper("semi_open")
	print("GRIPPED")
	sub.unregister()
	return
    
    

moveit_commander.roscpp_initialize(sys.argv)    
rospy.init_node('quaternion_to_euler')
rate = rospy.Rate(1)
gripper("open")
move_group = moveit_commander.MoveGroupCommander("manipulator")	
sub = rospy.Subscriber ('/aruco_marker_publisher/markers', MarkerArray, get_rotation,queue_size=1)


while not rospy.is_shutdown():
    rate.sleep()
