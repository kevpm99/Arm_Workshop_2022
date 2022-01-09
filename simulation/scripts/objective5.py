#!/usr/bin/env python
import sys
import rospy
import geometry_msgs.msg
import moveit_commander
from std_msgs.msg import String
from aruco_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi,cos,sin
import copy
import csv

def approach(msg, scale=1):
	
	gz=msg.pose.pose.position.z
	
	waypoints = []
	wpose = move_group.get_current_pose().pose
	print gz, wpose.position.z
    	wpose.position.z = scale * (gz) + 0.04 # Add coefficint depending on type of grasp		
    	waypoints.append(copy.deepcopy(wpose))
	print('Planning')
    	(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)# waypoints to follow # eef_step # jump_threshold
	move_group.execute(plan, wait=True)
	rospy.sleep(0.5)
	gripper("semi_close")
	print("GRIPPED")
	global flag
	flag=2 	
	return                    	

def gripper(msg):
	print("OPERATING_GRIPPER")
	pub = rospy.Publisher('/gripper_command',String,queue_size=10)
	for i in range(5):
		pub.publish(msg)
		rate.sleep()
	rospy.sleep(5)
	return

	 
def moveto(msg):
	global flag,f2
	x=msg.pose.pose.position.x+0.008261
	y=msg.pose.pose.position.y-0.07883
	z=msg.pose.pose.position.z+0.2
	
	pose_goal=move_group.get_current_pose()
	p_goal=pose_goal.pose.position
	if(abs(p_goal.x-x)<0.005 and abs(p_goal.y-y)<0.005 and abs(p_goal.z-z)<0.2):
		f2=1
	if(f2==1):
		print('Reached')
		flag=1
		return
	print('Callback to',x-p_goal.x,',',y-p_goal.y,',',z-p_goal.z)
	ratio=2#ratio:1
	#FIXME
	#Doesn't align exactly to the middle of the marker
	plan=False
	p_temp=[p_goal.x,p_goal.y,p_goal.z]
	while(not plan and ratio<129):
		p_goal.x=p_temp[0]
		p_goal.y=p_temp[1]
		p_goal.z=p_temp[2]
		p_goal.x=(ratio*p_goal.x+x)/(ratio+1)
		p_goal.y=(ratio*p_goal.y+y)/(ratio+1)
		p_goal.z=(5*ratio*p_goal.z+z)/(5*ratio+1)
		move_group.set_pose_target(pose_goal)
		plan=move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()
		ratio*=4
	if(not plan):
		print('Unable to plan :(')
		p_goal.x=p_temp[0]+0.01
		p_goal.y=p_temp[1]
		p_goal.z=p_temp[2]+0.02
		move_group.set_pose_target(pose_goal)
		plan=move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()
		flag=1
		return

def place():

	pose_goal0=move_group.get_current_pose().pose
	pose_goal0.position.z+=0.2

	pose_goal1=move_group.get_current_pose().pose
	pose_goal1.position.x-=0.2
	pose_goal1.position.z+=0.2

	with open('markers.csv') as csvfile:
		mreader=csv.reader(csvfile,delimiter=' ')
		j=1
		for pose in mreader:
			if j==14:
				x=float(pose[0])
				y=float(pose[1])
				z=float(pose[2])
				break
			j+=1
	pose_goal=move_group.get_current_pose().pose
	pose_goal.position.x=x-0.1
	pose_goal.position.y=y-0.1
	pose_goal.position.z=z+0.02
	
	waypoints = []		
    	waypoints.append(copy.deepcopy(pose_goal0))
    	waypoints.append(copy.deepcopy(pose_goal1))
    	waypoints.append(copy.deepcopy(pose_goal))
	
    	(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)# waypoints to follow # eef_step # jump_threshold
	move_group.execute(plan, wait=True)
	rospy.sleep(0.5)
	gripper('open')
	print('Cover placed')
	return                 


def clbk_marker(msgl):
	global f
	m=msgl.markers[0]
	for msg in msgl.markers:
		if msg.id==13:
			m=msg
			f=1
	if f==0:
		return
	global flag
	if flag==0:
		moveto(m)
	elif flag==1:
		approach(m)
	 	return
	elif flag==2:
		place()
	elif flag==3:
		return
f3=0
f=0
flag=0
f2=0
moveit_commander.roscpp_initialize(sys.argv)    
rospy.init_node('objective5')
rate = rospy.Rate(1)

gripper("open")
move_group = moveit_commander.MoveGroupCommander("manipulator")	
joint_state=[0,-120*pi/180,100*pi/180, 20*pi/180, 90*pi/180, -90*pi/180]
joint_goal=move_group.get_current_joint_values()
j=0
for item in joint_state:
	joint_goal[j]=item
	j+=1

move_group.go(joint_goal,wait=True)
move_group.stop()
rospy.sleep(0.5)


joint_state=[5.2761862991120525,-1.4004615095497184,1.032841262495966,5.097690548362802,-1.6036868206255521,0.9566206928376335]

joint_goal=move_group.get_current_joint_values()
j=0
for item in joint_state:
	joint_goal[j]=item
	j+=1

move_group.go(joint_goal,wait=True)
move_group.stop()
rospy.sleep(0.5)

sub = rospy.Subscriber ('/aruco_marker_publisher/markers', MarkerArray, clbk_marker,queue_size=1)

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
	if flag==3:
		break
	rate.sleep()
