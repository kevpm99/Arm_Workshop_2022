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
from aruco_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def button_press():
	print('Button press')
	pose_goal=move_group.get_current_pose().pose
	pose_goal.position.x+=0.05
	waypoints=[copy.deepcopy(pose_goal)]
	(plan,frac)=move_group.compute_cartesian_path(waypoints,0.001,0.0)
	move_group.execute(plan,wait=True)
	print('Done')
	sys.exit(0)
	
	
#Callback function for marker topic
def clbk_marker(msgl):
	found=False
	for m in msgl.markers:
		if m.id==9:
			msg=m
			found=True
			break
	if(not found):
		sys.exit(1)
	global f2#f2 is 1 if the marker has been reached and a maneouvre is being performed
	global current_marker#current marker that's being planned to, avoid any other detected markers from interfering
	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y
	z=msg.pose.pose.position.z-0.08
	o=msg.pose.pose.orientation
	mark_id=msg.id
	if(msg.id!=current_marker and current_marker!=0):
		return
	current_marker=mark_id
	pose_goal=move_group.get_current_pose()
	p_goal=pose_goal.pose.position
	#if abs(x-p_goal.x)<0.02:
	#	x=p_goal.x
	#if abs(y-p_goal.y)<0.01:
	#	y=p_goal.y
	#if abs(z-p_goal.z)<0.03:
	#	z=p_goal.z
	if(f2==1):
		print('Reached')
		if(mark_id>=1 and mark_id<=9):
			button_press()
		#elif(mark_id==10):
		#	imu_pickup()
		#elif(mark_id==11):
		#	imu_place()
		#elif(mark_id==12):
		#	cover_pick()
		#elif(mark_id==14):
		#	cover_place()
		f2=0
		return
	print('Callback to',x-p_goal.x,',',y-p_goal.y,',',z-p_goal.z,' with id ',mark_id)
	ratio=2#ratio:1
	#FIXME
	#Doesn't align exactly to the middle of the marker
	plan=False
	p_temp=[p_goal.x,p_goal.y,p_goal.z]
	while(not plan and ratio<129):
		p_goal.x=p_temp[0]
		p_goal.y=p_temp[1]
		p_goal.z=p_temp[2]
		p_goal.x=(5*ratio*p_goal.x+x)/(5*ratio+1)
		p_goal.y=(ratio*p_goal.y+y)/(ratio+1)
		p_goal.z=(ratio*p_goal.z+z)/(ratio+1)
		move_group.set_pose_target(pose_goal)
		plan=move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()
		ratio*=4
	if(not plan):
		f2=1
	
	#pose_goal.pose.position.x += min(0.001,z/2) if z>0 else max(-0.001,z/2) #Order of coordinates is a bit messed up
	#pose_goal.pose.position.y += min(0.001,y/2) if y>0 else max(-0.001,y/2)
	#pose_goal.pose.position.z += min(0.001,x/2) if x>0 else max(-0.001,x/2)
	
	#pose_goal.pose.orientation.x=o.x
	#pose_goal.pose.orientation.y=o.y
	#pose_goal.pose.orientation.z=o.z
	#pose_goal.pose.orientation.w=o.w
	
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
	"""
	
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = move_group.get_current_joint_values()
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
print('Closing the gripper')
for i in range(10):
	pub.publish(gripper_msg)
	rate.sleep()
time.sleep(5)

print('First done')
f=0
f1=0

current_marker=0

sub = rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,clbk_marker,queue_size=1,buff_size=2**24)

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
