import vrep
import time
import numpy as np
import math
import copy
import sys

from math import cos, sin
from scipy.linalg import expm,logm

PI = np.pi

# lab4 - invers kinematic
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	theta = [None, None, None, None, None, None]
	# Using the extra 0 to make the indices line up with the paper calculations
	link_len = [0, 0.152, 0.120, 0.244, 0.093, 0.213, 0.083, 0.083, 0.082, 0.0535, 0.059]

	# TODO: offset the "Wgrip" positions to the base frame (frame 0)
	# We are given coordinates in the world frame, but our IK is done in frame 0
	x_grip = xWgrip + 0.150
	y_grip = yWgrip - 0.150
	z_grip = zWgrip - 0.010


	r_grip = 0.0535 # meters
	yaw_rad = PI/180*yaw_WgripDegree
	x_cen = x_grip - r_grip*math.cos(yaw_rad)
	y_cen = y_grip - r_grip*math.sin(yaw_rad)
	z_cen = z_grip

	# Calculation of theta_1
	link_offset_1 = link_len[2] - link_len[4] + link_len[6]
	theta[0] = math.atan2(y_cen, x_cen) - math.asin(link_offset_1/math.sqrt(x_cen**2 + y_cen**2))

	# Calculation of theta_6
	theta[5] = PI/2 - yaw_rad + theta[0]

	# Calculation of x_3end, y_3end, z_3end
	alpha = math.atan2(link_len[7], 0.027+link_len[6])
	beta = PI/2 - alpha
	gamma = PI - theta[0] - beta


	# use coordinate transformation to go between c frame fixed at p_cen
	# and base frame.

	transform = np.eye(4)
	transform[0,0] = math.cos(theta[0])
	transform[0,1] = -math.sin(theta[0])
	transform[1,0] = math.sin(theta[0])
	transform[1,1] = math.cos(theta[0])
	transform[0,3] = x_cen
	transform[1,3] = y_cen
	transform[2,3] = z_cen

	# multiply T by x_3end, y_3end, z_3end relative to c frame
	# 0.11 is L2 - L4 + L6 and 0.083 is L7
	p_3end = np.matmul(transform, np.array([-0.083, -0.11, link_len[8] + link_len[10], 1]).T)

	x_3end = p_3end[0]
	y_3end = p_3end[1]
	z_3end = p_3end[2]

	z_3end_l1 = z_3end - link_len[1]
	r_end_l1 = math.sqrt(x_3end**2 + y_3end**2 + z_3end_l1**2)

	alpha = math.atan2(z_3end_l1, math.sqrt(r_end_l1**2 - z_3end_l1**2))

	# apply law of sines and law of cosines to get intermediate angles
	gamma = math.acos((link_len[5]**2 + r_end_l1**2 - link_len[3]**2)/(2*link_len[5]*r_end_l1))
	beta = math.asin(link_len[5]*math.sin(gamma)/link_len[3])

	# determine theta_2 and theta_3
	theta[1] = -(alpha + beta)
	theta[2] = beta + gamma

	# determine theta_4
	theta[3] = -(theta[1] + theta[2])

	# theta_5 is locked at -90 degrees
	theta[4] = -PI/2

	return (theta[0], theta[1], theta[2], theta[3], theta[4], theta[5])

# set joints
def set_joint(theta):
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta[2], vrep.simx_opmode_oneshot)
	time.sleep(0.1)
	vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta[3], vrep.simx_opmode_oneshot)
	time.sleep(0.1)    
	vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta[4], vrep.simx_opmode_oneshot)
	time.sleep(0.1)
	vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta[5], vrep.simx_opmode_oneshot)
	time.sleep(0.1)
	vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta[0], vrep.simx_opmode_oneshot)
	time.sleep(0.5)
	vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta[1], vrep.simx_opmode_oneshot)
	time.sleep(0.5)



# Function that used to read joint angles
def get_joint():
	r3, theta3 = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
	r4, theta4 = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
	r5, theta5 = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
	r6, theta6 = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
	r1, theta1 = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
	r2, theta2 = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)

	if r1 != vrep.simx_return_ok or r2 != vrep.simx_return_ok or r3 != vrep.simx_return_ok or r4 != vrep.simx_return_ok or r5 != vrep.simx_return_ok or r6 != vrep.simx_return_ok:
		raise Exception('get joint failed')

	theta = np.array([[theta1],[theta2],[theta3],[theta4],[theta5],[theta6]])
	return theta


def move(start, pick, loc1, loc2, base_pos):
	# go to picking pos
	set_joint(pick)
	time.sleep(0.5)
	# turn suction on
	suction(1)
	time.sleep(0.5)
	# first move
	set_joint(loc1)
	time.sleep(0.5)
	# second move
	set_joint(loc2)
	time.sleep(0.5)
	set_joint(base_pos) 
	time.sleep(0.5)
	# turn suction off
	suction(0)
	time.sleep(0.5)
    # return back to start
	set_joint(loc2)
	set_joint(loc1)
	set_joint(start)


def suction(flag):
    result, base_handle = vrep.simxGetObjectHandle(clientID, 'active', vrep.simx_opmode_blocking)
    # turn suction on
    if (flag==1):
        vrep.simxClearIntegerSignal(clientID, 'active', vrep.simx_opmode_oneshot)
        vrep.simxSetIntegerSignal(clientID, 'active', 1, vrep.simx_opmode_blocking)
	# turn suction off
    else:
        vrep.simxSetIntegerSignal(clientID, 'active', 0, vrep.simx_opmode_blocking)



# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  =========================================== #

# Get "handle" to the base of robot
result, base_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for base frame')
    
# Get "handle" to the all joints of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for first joint')
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for second joint')
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for third joint')
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fourth joint')
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for fifth joint')
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for sixth joint')

# Get "handle" to the end-effector of robot
result, end_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link7_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object handle for end effector')
# ==================================================================================================== #

# Start simulation

vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #
xWgrip = 0.21
yWgrip = 0
zWgrip1 = 0.23
zWgrip2 = 0.085
zWgrip3 = 0.3
yaw_WgripDegree = 90
offset = 0.15

# base position
init_position = [-0.1, -0.25, 0.055]

# initialize destination thetas
start = lab_invk(xWgrip, yWgrip, zWgrip1, yaw_WgripDegree)
pick = lab_invk(xWgrip, yWgrip, zWgrip2, yaw_WgripDegree)
loc1 = lab_invk(xWgrip, yWgrip, zWgrip3, yaw_WgripDegree)
# initialize proximity sensor
result, Proximity_sensor = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_blocking)

init_x = init_position[0]
init_y = init_position[1]
init_z = init_position[2]

# go to starting position
set_joint(start)
cur_block_cnt = 0
cur_layer_cnt = 1
prev_init_x = init_x

# variables used for pyramid building
num_layer = 3
num_block = 3

brick_size = 0.07
end_test=0
while (end_test==0):
	# read the state of proximity sensor
    result, distance_to_detected_point, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID, Proximity_sensor, vrep.simx_opmode_blocking)
    if (distance_to_detected_point > 0):
        if (cur_layer_cnt <= num_layer):
            if (cur_block_cnt < num_block):
				# turn suction off
                suction(0)
				# get base position and loc2
                loc2 = lab_invk(init_x, init_y, init_z + offset, yaw_WgripDegree)
                base_pos = lab_invk(init_x, init_y, init_z, yaw_WgripDegree)
				# motion planning
                move(start, pick, loc1, loc2, base_pos)
                cur_block_cnt += 1
				# update current x position
                init_x = init_x - brick_size
			# update pyramid block position
            else:
				# shift by half the brick size
                prev_init_x = prev_init_x - (brick_size / 2)
                init_x = prev_init_x  
				# increment height
                init_z = init_z + init_position[2]
                num_block = num_block + num_layer - cur_layer_cnt
                cur_layer_cnt += 1
        else:
            break 
           

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #
