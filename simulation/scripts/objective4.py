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
import csv   	

def gripper(msg):
	print("OPERATING_GRIPPER")
	pub = rospy.Publisher('/gripper_command',String,queue_size=10)
	for i in range(5):
		pub.publish(msg)
		rate.sleep()
	rospy.sleep(5)
	return

def aligner(groll):
	print("Aligning")
	joint_goal = move_group.get_current_joint_values()
	joint_goal[5] -= groll/3
	
	move_group.go(joint_goal, wait=True)
	move_group.stop()

def align():
	global angle
	orientation_curr=move_group.get_current_pose().pose.orientation
	orientation_currlist = [orientation_curr.x, orientation_curr.y, orientation_curr.z, orientation_curr.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_currlist)
	if(roll<0):
		roll=roll+2*pi
	roll_begin=roll
	
	
	
	while abs(roll-roll_begin+angle) > 0.05:
		print roll
		if(roll<-2.5):
			roll=roll+2*pi
		print angle
    		aligner(roll-roll_begin+angle)
    		orientation_curr=move_group.get_current_pose().pose.orientation
		orientation_currlist = [orientation_curr.x, orientation_curr.y, orientation_curr.z, orientation_curr.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_currlist)
	else:
		print("!!ALIGNED!!")
		return

def stick():
	x=0.25
	with open('markers.csv') as csvfile:
		mreader=csv.reader(csvfile,delimiter=' ')
		j=1
		for pose in mreader:
			if j==11:
				x=float(pose[0])
				break
			j+=1
	pose_goal=move_group.get_current_pose().pose
	print(x-pose_goal.position.x)
	pose_goal.position.x=x
	waypoints = []		
    	waypoints.append(copy.deepcopy(pose_goal))
	
    	(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)# waypoints to follow # eef_step # jump_threshold
	move_group.execute(plan, wait=True)
	rospy.sleep(0.5)
	print('IMU placed')
	return                 
	
def remove():
	pose_goal=move_group.get_current_pose().pose
	pose_goal.position.x-=0.1
	waypoints = []		
    	waypoints.append(copy.deepcopy(pose_goal))
	
    	(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)# waypoints to follow # eef_step # jump_threshold
	move_group.execute(plan, wait=True)
	rospy.sleep(0.5)
	print('Objective 4 done!')
	return                 
	
flag=0
f2=0
moveit_commander.roscpp_initialize(sys.argv)    
rospy.init_node('objective4')
rate = rospy.Rate(1)

angle=float(rospy.get_param('ang'))
angle=angle*pi/180

joint_state=[-1.4180705323790317,-2.1441651636780725,-2.4701723783449694,1.5715432504855764,0.20426343178001982,1.4695344626567026]

move_group = moveit_commander.MoveGroupCommander("manipulator")	

joint_goal=move_group.get_current_joint_values()
j=0
for item in joint_state:
	joint_goal[j]=item
	j+=1

move_group.go(joint_goal,wait=True)
move_group.stop()
rospy.sleep(0.5)

align()
stick()
gripper('open')
remove()

