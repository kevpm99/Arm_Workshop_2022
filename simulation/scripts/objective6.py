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

def clbk_marker(msgl):
	global f
	m=msgl.markers[0]
	for msg in msgl.markers:
		if msg.id<=9:
			print(msg.id)
			global flag
			flag=1
f3=0
f=0
flag=0
f2=0
moveit_commander.roscpp_initialize(sys.argv)    
rospy.init_node('objective6')
rate = rospy.Rate(1)
joint_state=[5.2761862991120525,-1.4004615095497184,1.032841262495966,5.097690548362802,-1.6036868206255521,0.9566206928376335]
move_group = moveit_commander.MoveGroupCommander("manipulator")	

joint_goal=move_group.get_current_joint_values()
j=0
for item in joint_state:
	joint_goal[j]=item
	j+=1

move_group.go(joint_goal,wait=True)
move_group.stop()
rospy.sleep(0.5)

sub = rospy.Subscriber ('/aruco_marker_publisher/markers', MarkerArray, clbk_marker,queue_size=1)

if(f==0):
	gripper('close')
	rospy.sleep(5)
if(f==0):
	gripper('open')
	rospy.sleep(5)
while(f==0):
	pose_goal=move_group.get_current_pose()
	p_goal=pose_goal.pose.position
	p_goal.z+=0.05		
	move_group.set_pose_target(pose_goal)
	plan=move_group.go(wait=True)
	move_group.stop()
	move_group.clear_pose_targets()
	rospy.sleep(0.5)



while not rospy.is_shutdown():
	if flag==1:
		break
	rate.sleep()
