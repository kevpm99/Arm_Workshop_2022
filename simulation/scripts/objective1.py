#!/usr/bin/env python

import sys
import copy
import rospy 
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import *
from aruco_msgs.msg import MarkerArray
import numpy as np
import csv

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('please_work', anonymous=True)

group_name = "manipulator"



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander(group_name) 

pos=[[[0 for k in range(3)] for i in range(100)] for j in range(14)]
avgpose=[[0 for i in range(3)] for j in range(14)]
nread=[0 for j in range(14)]
#marker_states=[[0 for k in range(6)] for i in range(14)]      

#################################################################
def execute_plan(plan):
    group.execute(plan, wait=True)

##################################################################
def go_to_state(joints): 
    group.go(joints, wait=True)
    group.stop()


#################################################################

def clbk_marker(msg):
	for mark in msg.markers:
		if mark.id<1 or mark.id>14:
			continue
		if nread[mark.id-1]<100:
			position=mark.pose.pose.position
			pos[mark.id-1][nread[mark.id-1]][0]=position.x
			pos[mark.id-1][nread[mark.id-1]][1]=position.y
			pos[mark.id-1][nread[mark.id-1]][2]=position.z
			nread[mark.id-1]+=1
#		if f==1:
#			curr_state=group.get_current_joint_values()
#			for i in range(6):
#				marker_states[mark.id-1][i]=curr_state[i]
		

#################################################################

sub = rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,clbk_marker,queue_size=1,buff_size=2**24)

path=rospy.get_param('file_path')
states=[]
with open(path+'/states.csv') as csvfile:
	jreader=csv.reader(csvfile,delimiter=' ')
	for row in jreader:
		states.append(row)

#f=0
j=0

for values in states:
	joint_state=[float(value) for value in values]
	explore_next=go_to_state(joint_state)
	rospy.sleep(0.5)
	print('state ',j)
	if j==2 or j==4:
		with open(path+'/markers.csv','w') as csvfile:
			pose_writer=csv.writer(csvfile,delimiter=' ',quotechar='|', quoting=csv.QUOTE_MINIMAL)
			for r in pos:
				pose_writer.writerow(r[0])
	j+=1
#	f=1
#	rospy.sleep(0.5)
#	f=0

j=0

for poses in pos:
	sum=[0,0,0]
	for pose in poses:
		for i in range(3):
			sum[i]+=pose[i]
	if(nread[j]>0):
		for i in range(3):
			sum[i]=sum[i]/nread[j]
			avgpose[j][i]=sum[i]
		j+=1

with open(path+'/markers.csv','w') as csvfile:
	pose_writer=csv.writer(csvfile,delimiter=' ',quotechar='|', quoting=csv.QUOTE_MINIMAL)
	for j in avgpose:
		pose_writer.writerow(j)

#with open(path+'/marker_states.csv','w') as csvfile:
	#state_writer=csv.writer(csvfile,delimiter=' ',quotechar='|', quotingcsv.QUOTE_MINIMAL)
	#for j in marker_states:
	#	state_writer.writerow(j)

moveit_commander.roscpp_shutdown()








