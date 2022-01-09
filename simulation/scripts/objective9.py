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

def gripper(msg):
	print("OPERATING_GRIPPER")
	pub = rospy.Publisher('/gripper_command',String,queue_size=10)
	for i in range(5):
		pub.publish(msg)
		rate.sleep()
	rospy.sleep(5)
	return
	
f3=0
f=0
flag=0
f2=0
moveit_commander.roscpp_initialize(sys.argv)    
rospy.init_node('objective6')
rate = rospy.Rate(1)
joint_state=[0,-120*pi/180,100*pi/180, 20*pi/180, 90*pi/180, -90*pi/180]
move_group = moveit_commander.MoveGroupCommander("manipulator")	

joint_goal=move_group.get_current_joint_values()
j=0
for item in joint_state:
	joint_goal[j]=item
	j+=1

move_group.go(joint_goal,wait=True)
move_group.stop()
rospy.sleep(0.5)
gripper('open')
rospy.sleep(0.5)
