#!/usr/bin/env python

import sys
import copy
import rospy 
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import *
import numpy as np


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('please_work', anonymous=True)

group_name = "manipulator"

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander(group_name) 


def converter(yaw,pitch,roll):
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	
	return [qx,qy,qz,qw]

      
#################################################################
def execute_plan(plan):
    group.execute(plan, wait=True)

##################################################################
def go_to_state(joints): 
        group.go(joints, wait=True)
        group.stop()


#################################################################

#EXPLRE LEFT UP
joints_left_up = [-0.9403913208670707, -1.3837870024801755, -1.2329784584129797, 2.510066354359614, 0.3397657544817614, -1.4813071928867956]


explore_left_up = go_to_state(joints_left_up)
rospy.sleep(2) 
########
left_down = [-1.4180705323790317, -2.1441651636780725, -2.4701723783449694, 1.5715432504855764, 0.20426343178001982, 1.4695344626567026]

explore_left_down = go_to_state(left_down)
rospy.sleep(2)
#####
left_ground = [-1.2838236391777116, -1.5037936477317486, -1.8758328274404068, 4.968148038350566, 1.5949311152127343, -2.698502569868803]
left_ground = go_to_state(left_ground)
rospy.sleep(2)
##################################################################
#Explore middle


j4 = [4.051814833446692, -2.1730123192508626, -2.3295244188920474, 4.491443628211006, 5.643459624855952, 4.7238155847586905]
state4 = go_to_state(j4)
rospy.sleep(5) 

j5 = [3.3181323594927914, -2.1728085529950634, -2.3387280315316, 4.504935187858785, 4.910021415368389, 4.716030992320193]
state5 = go_to_state(j5)
rospy.sleep(5)

j6 = [4.1120293563649035, -1.5827427963507215, -2.0217664477687816, 3.592497790844627, 5.703801592995461, 4.724937574543288]
state6 = go_to_state(j6)
rospy.sleep(5)

j7= [3.3922937411298806, -1.511553472979012, -2.09279630270197, 3.5975329155966564, 4.983977014042375, 4.71662944406521]
state5 = go_to_state(j7)
rospy.sleep(5)

j8 = [4.071839429787873, -1.4688125324745802, -1.7429368074886176, 3.200454802171433, 5.663647577926657, 4.7241128756208415]
state5 = go_to_state(j8)
rospy.sleep(5)

###################################################################
#EXPLRE Right UP

right_up = [2.1210043307394493, -1.5703061546991344, -1.612126151802281, 0.04080696423582353, 2.6520810067761236, 1.5706413556227776]

explore_right_up = go_to_state(right_up)

joints_right_up = [5.2761862991120525, -1.4004615095497184, 1.0328412624959666, 5.097690548362802, -1.6036868206255521, 0.9566206928376335]

right_ground = [2.1539352315996485, -1.3244309133876757, -2.3162091749566747, -1.0903267280190274, 1.5382580931915317, 0.5225391755034936]

explore_right_ground = go_to_state(right_ground)

explore_right_up = go_to_state(joints_right_up)
rospy.sleep(2) 
########
joints_right_front =[-3.437092422683218, -1.476542951139685, -1.8029316362360879, 2.5986121578237302, 4.798127192177189, -1.5229542515373335]

explore_right_front = go_to_state(joints_right_front)
rospy.sleep(2) 
##################################################################

moveit_commander.roscpp_shutdown()




j1 = [4.646693279334001, -0.5849602016529469, -2.5415386603922556, -0.16345835800508368, 0.04513134224806592, 1.7215059267098436]
j2 = [4.283490107557041, -1.3565642569010063, -2.2806157017198263, 0.4792376149783655, 0.40814520710802693, 1.588341295218087]
j3 = [3.6892018306529932, -1.2518820755304776, -2.3513047355932883, 0.4538608742788748, 1.0022364794153926, 1.5773103058668916]

##########







