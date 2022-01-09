#! /usr/bin/env python

from __future__ import print_function
from six.moves import input
from math import pi
import sys
import copy
import tf
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#Callback function for marker topic
def clbk_marker(msg):
	global f2#f2 is 1 if the marker has been reached and a maneouvre is being performed
	if(f2==1):return
	x=msg.pose.position.x-0.003
	y=msg.pose.position.y-0.0025
	z=msg.pose.position.z
	o=msg.pose.orientation
	mark_id=msg.id
	if abs(x)<0.0007:
		x=0.0
	if abs(y)<0.001:
		y=0.0
	if abs(z)<0.012:
		z=0.0
	if(x==0 and y==0 and z==0):
		f2=1
		print('Reached')
		return
	print('Callback to',x,',',y,',',z,' with id ',mark_id)
	
	pose_goal=move_group.get_current_pose()
	#FIXME
	#Doesn't align exactly to the middle of the marker
	pose_goal.pose.position.x += z/15#Order of coordinates is a bit messed up
	pose_goal.pose.position.y -= y/3
	pose_goal.pose.position.z += x/3
	
	"""pose_goal.pose.orientation.x=o.x
	pose_goal.pose.orientation.y=o.y
	pose_goal.pose.orientation.z=o.z
	pose_goal.pose.orientation.w=o.w
	"""
	
	move_group.set_pose_target(pose_goal)
	plan = move_group.go(wait=True)
	move_group.stop()
	move_group.clear_pose_targets()
	global f#f>1 if a marker has been detected
	f+=1
	"""
	o=msg.pose.orientation
	print('Goal: ',o)
	ol = [o.x,o.y,o.z,o.w]
	rpy=euler_from_quaternion(ol)
	
	print('RPY: ',rpy)
	p=move_group.get_current_pose()
	po=p.pose.orientation
	pose_list=[po.x,po.y,po.z,po.w]
	(roll,pitch,yaw)=euler_from_quaternion(pose_list)
	yaw=(rpy[1]+yaw)/2
	pose_goal=quaternion_from_euler(roll,pitch,yaw)
	p.pose.orientation.x=pose_goal[0]
	p.pose.orientation.y=pose_goal[1]
	p.pose.orientation.z=pose_goal[2]
	p.pose.orientation.w=pose_goal[3]
	
	
	move_group.set_pose_target(p)
	plan=move_group.go(wait=True)
	move_group.stop()
	move_group.clear_pose_targets()
	"""
	
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = move_group.get_current_joint_values()
print(len(joint_goal))
joint_goal[0] = -294*pi/180#0
joint_goal[1] = -70*pi/180#-pi/90
joint_goal[2] = 114*pi/180#-pi/100
joint_goal[3] = -224*pi/180#-pi/100
joint_goal[4] = -149*pi/180#0
joint_goal[5] = 90*pi/180#pi/60

#TODO
#Explore the workspace here, maybe store marker poses in a list,or joint values where markers were detected

#move_group.go(joint_goal, wait=True)

move_group.stop()

rate = rospy.Rate(20)

pub=rospy.Publisher('/gripper_command',String,queue_size=10)
gripper_msg='close'

for i in range(10):
	pub.publish(gripper_msg)
	rate.sleep()
time.sleep(5)

print('First done')
f=0
f1=0


sub = rospy.Subscriber('/aruco_marker_publisher/marker',Marker,clbk_marker)

#Marker id: /aruco_single/marker
f2=0
while not rospy.is_shutdown():
	rate.sleep()
	"""
	if(f>0):
		f1=f
		time.sleep(0.4)#Start a maneouvre if no callbacks for 0.4 seconds
		if(f1==f and f2==0):
		#TODO
		#Replace with code for specific markers 
			f2=1
			print('Starting switching manouevre')
			pose_goal=move_group.get_current_pose()
			pose_goal.pose.position.z-=0.05
			move_group.set_pose_target(pose_goal)
			plan=move_group.go(wait=True)
			move_group.stop()
			move_group.clear_pose_targets()
			pose_goal=move_group.get_current_pose()
			pose_goal.pose.position.z-=0.05
			move_group.set_pose_target(pose_goal)
			pan=move_group.go(wait=True)
			move_group.stop()
			move_group.clear_pose_targets()
			pose_goal=move_group.get_current_pose()
			pose_goal.pose.position.y+=0.06
			move_group.set_pose_target(pose_goal)
			plan=move_group.go(wait=True)
			move_group.stop()
			move_group.clear_pose_targets()
			pose_goal=move_group.get_current_pose()
			pose_goal.pose.position.y-=0.06
			move_group.set_pose_target(pose_goal)
			plan=move_group.go(wait=True)
			move_group.stop()
			move_group.clear_pose_targets()
	rate.sleep()
"""
