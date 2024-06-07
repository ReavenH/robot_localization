#!/usr/bin/env python

'''
Based on FinalLocalization script, using proximity sensors for horizontal offset correction instead.
'''
from __future__ import print_function
from argparse import ArgumentParser
import numpy as np
import os
import cv2
import math

import time
import json
import serial
import threading
import re
import sys
import matplotlib.pyplot as plt

import sm_bus
# sys.path.append("/home/pi/WAVEGO/RPi/small_function_group/AprilTag/scripts")
'''
import apriltag_video_picamera2 as avp
'''
import calibrationfunc as cf
from enum import Enum
##kill the agetty serial
os.system("sudo systemctl stop serial-getty@ttyS0.service")

def connect_serial(port, baudrate):
    while True:
        try:
            ser = serial.Serial(port, baudrate)
            print("Serial connected")
            return ser
        except serial.SerialException as e:
            print("Serial Disconnected:", e)
            print("wait for 5 second for reconnecting...")
            time.sleep(5)


port = "/dev/ttyS0"
baudrate = 115200

ser = connect_serial(port, baudrate)
ser.timeout = None
#ser.timeout = 1  


dataCMD = json.dumps({'var':"", 'val':0, 'ip':""})
#YAO, 3.16.2024
#if isfront=5: move forward, if isfront=-5: move backward
isfront=5
iswalk = 1

################ Proximity Sensor Start ########################################
print("\nSparkFun Proximity Sensor VCN4040 Example 1\n")
oProx_0 = sm_bus.QwiicProximity0()
oProx_1 = sm_bus.QwiicProximity1()
oProx_0.begin()
oProx_1.begin()
################ Proximity Sensor End ########################################

class WaveStatus(Enum):  
	FREE_WALKING = 99
	FREE_CLIMBING = 98
	
	STANDING_DETECT_START = 97
	STANDING_DETECTING = 96
	PLAN_NEXT_STEP = 95 
	
	ADJUST_ROTATION_LEFT = 94
	ADJUST_ROTATION_RIGHT = 93

	SHIFT_LEFT = 92
	ROTATING = 91
	BACKWARD = 90
	
	ADJUST_BEFORE_CLIMB = 89
	ADJUST_BEFORE_ROTATE = 88
	
	RESET = 87
	
	START = 1
	END = 0

'''
def apriltaggetpositionThreadFunc():
	global tag_info
	avp.apriltag_video(output_stream=False,display_stream=False,print_log=False,cameraMatrix=cf.cameraMatrix,distCoeffs=cf.distCoeffs)
'''
'''
#YAO, 3.17.2024, mean value filter
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = []

    def add_data_point(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)

    def get_filtered_value(self):
        if not self.data:
            return None
        return sum(self.data) / len(self.data)

window_size = 10  
z_distance_filter = MovingAverageFilter(window_size)
'''
'''
def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False
'''
waveStatus = WaveStatus.STANDING_DETECT_START

def get_prox_dis():
	proxValue_0 = oProx_0.get_proximity()
	proxValue_1 = oProx_1.get_proximity()
	prox_diff = (proxValue_0 - proxValue_1)/15
	return prox_diff
	
def prox_check_cross():
	cross_threshold =  250
	
	proxValue_0 = oProx_0.get_proximity()
	proxValue_1 = oProx_1.get_proximity()
	
	prox_sum = proxValue_0 + proxValue_1
	# print("prox_sum",prox_sum)
	if prox_sum <= cross_threshold:
		return True
	else:
		return False
			
	
	
	return prox_diff
		
#YAO, 3.16.2024, add threading of walking on the board
def WalkOnBoard():
	global tag_info
	global isfront
	global iswalk
	global waveStatus
	global robotPose
	last_time = time.time()
	while iswalk:
		################################### GET APRIL TAG DATA #################################
		# print(waveStatus)
		current_time = time.time()
		#tag_info = avp.tag_info
		# if len(tag_info)==0:		#skip if the apriltag is not detected
		# 	continue
		'''
		yaw = round(float(tag_info[0][1]),3)
		z_distance = round(float(tag_info[0][2]),3)
		x_distance = round(float(tag_info[0][4]),3)
        '''

		

		## mean value filter the z_distance
		#z_bias = 1.5
		# z_bias = 0
		# z_distance_filter.add_data_point(z_distance)
		# z_distance_filtered = z_distance_filter.get_filtered_value()+z_bias
		#print("yaw is: ",yaw, " x distance is: ",x_distance,"z distance is: ",z_distance)
		
		#write out data as file
		#with open('output.txt', 'a') as file:
		#	file.write(str(tag_info) + ' ' + value_str+ ' '+ '\n')
		'''
		####################################### GET DATA END ########################################
		'''
					

		
		if waveStatus == WaveStatus.STANDING_DETECT_START:
			print('Standing Detect Start')
			#print("robotPose {}".format(robotPose))
			yaw = - round(robotPose[0].copy(), 3)
			x_distance = 0
			z_distance = get_prox_dis()
			#z_distance = round(robotPose[2].copy(), 3)
			#x_distance = round(robotPose[1].copy(), 3)
			
			
			z_distance_acc = z_distance		#accumulate the z_distance
			yaw_acc = yaw
			x_distance_acc = x_distance
			z_dis_count = 1					#count till 20 times	
			
			waveStatus = WaveStatus.STANDING_DETECTING
			continue
			
		if waveStatus == WaveStatus.STANDING_DETECTING:
			
			#print("robotPose {}".format(robotPose))
			yaw = - round(robotPose[0].copy(), 3)
			z_distance = round(robotPose[2].copy(), 3)
			x_distance = round(robotPose[1].copy(), 3)
			z_distance_acc += z_distance		#accumulate the z_distance
			x_distance_acc += x_distance		#accumulate the z_distance
			yaw_acc += yaw
			z_dis_count += 1					#count till 20 times
			num_cnt = 40	
			if z_dis_count == num_cnt:				#already collect 20 z_distance data
				steady_z_distance = z_distance_acc / num_cnt + 1 		#calculate the mean z_distance
				steady_yaw = yaw_acc / num_cnt
				steady_x_distance = x_distance_acc / num_cnt
				waveStatus = WaveStatus.PLAN_NEXT_STEP
				print("steady_yaw is: ",steady_yaw," steady_x distance is: ",steady_x_distance," steady_z distance is: ",steady_z_distance)
				print(' ')
				print(' ')
			continue

############################################### PLANNING ######################################
		if waveStatus == WaveStatus.PLAN_NEXT_STEP:
			print('Start Planning')
			distance_climb_prepare = 64
			distance_climb_start = 61
			distance_climb_end = 44
			
			distance_rotate_prepare = 7
			distance_rotate_start = 5.5
			isfront = 5
			
			

			############################  WALK  ################################
			
			if steady_yaw>5.0:
				#left5()
				freeturn(min(max(steady_yaw*1.5,7.5),15))
				waveStatus = WaveStatus.ADJUST_ROTATION_LEFT
				continue
			elif steady_yaw<-5.0:
				#right5()
				freeturn(max(min(steady_yaw*1.5,-7.5),-15))
				waveStatus = WaveStatus.ADJUST_ROTATION_RIGHT
				continue
			else:
				direction_deg = math.degrees(math.atan2(steady_z_distance,isfront))
				#freewalk(max(min(direction_deg+steady_yaw/2,15),-15),dis_diff)
				freewalk(max(min(direction_deg+steady_yaw/2,15),-15))
					
				last_time = current_time
				waveStatus = WaveStatus.FREE_WALKING
				continue
			
#####################################  PLAN END  ##########################				
				
		if waveStatus == WaveStatus.FREE_WALKING:

			global_step = -1
			value_str = ser.readline().decode().strip()
			global_step_boolean = re.search(r'Global_Step: (-?\d+\.\d+)', value_str)
			if global_step_boolean:
				# get the time in second
				global_step = global_step_boolean.group(1)
			else:
				global_step = -1
			
			#print(global_step)
			
			if prox_check_cross() == True:
				waveStatus = WaveStatus.END
			
			if float(global_step) >= 1.00:
				waveStatus = WaveStatus.STANDING_DETECT_START
			continue
			
		if waveStatus == WaveStatus.FREE_CLIMBING:
			global_step = -1
			value_str = ser.readline().decode().strip()
			global_step_boolean = re.search(r'Global_Step: (-?\d+\.\d+)', value_str)
			if global_step_boolean:
				global_step = global_step_boolean.group(1)
			else:
				global_step = -1

			if float(global_step) >= 1.00:
				waveStatus = WaveStatus.STANDING_DETECT_START
			continue
		
		if waveStatus == WaveStatus.ADJUST_ROTATION_LEFT:
			global_step = -1
			value_str = ser.readline().decode().strip()
			global_step_boolean = re.search(r'Global_Step: (-?\d+\.\d+)', value_str)
			if global_step_boolean:
				global_step = global_step_boolean.group(1)
			else:
				global_step = -1

			if float(global_step) >= 1.00:
				waveStatus = WaveStatus.STANDING_DETECT_START
			continue
			
		if waveStatus == WaveStatus.ADJUST_ROTATION_RIGHT:
			global_step = -1
			value_str = ser.readline().decode().strip()
			global_step_boolean = re.search(r'Global_Step: (-?\d+\.\d+)', value_str)
			if global_step_boolean:
				global_step = global_step_boolean.group(1)
			else:
				global_step = -1

			if float(global_step) >= 1.00:
				waveStatus = WaveStatus.STANDING_DETECT_START
			continue
			
		if waveStatus == WaveStatus.END:
			reset()
			continue

			

		
		
walkThread = threading.Thread(target=WalkOnBoard)			
#apriltaggetpositionThread = threading.Thread(target=apriltaggetpositionThreadFunc)
'''
def setUpperIP(ipInput):
	global upperGlobalIP
	upperGlobalIP = ipInput
'''


def startwalk():
	dataCMD = json.dumps({'var':"move", 'val':1})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	freewalk(0)
	time.sleep(0.1)
	dataCMD = json.dumps({'var':"funcMode", 'val':13})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	freewalk(0)
	time.sleep(0.1)
	print('start testing')


def freewalk(degree,distance=30):
	print('freewalk direction: ',degree,' distance: ',distance)
	dataCMD = json.dumps({'var':"freewalk", 'val':degree, 'dis':distance})
	#dataCMD = json.dumps({'var':"freetrot", 'val':degree*2, 'dis':distance})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	

def freeturn(degree):
	print('freeturn direction: ',degree)
	dataCMD = json.dumps({'var':"freeturn", 'val':degree})
	ser.write(dataCMD.encode())
	time.sleep(0.1)

def freerotate():
	print('freerotate direction: ',15)
	dataCMD = json.dumps({'var':"freerotate", 'val':0})
	ser.write(dataCMD.encode())
	time.sleep(0.1)

def right5():
	dataCMD = json.dumps({'var': "crabwalk", 'val': 0})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	dataCMD = json.dumps({'var': "funcMode", 'val': 17})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	print('right 5 degree')
	
def left5():
	dataCMD = json.dumps({'var': "crabwalk", 'val': 1})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	dataCMD = json.dumps({'var': "funcMode", 'val': 17})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	print('left 5 degree')

def left5():
	dataCMD = json.dumps({'var': "crabwalk", 'val': 1})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	dataCMD = json.dumps({'var': "funcMode", 'val': 17})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	print('left 5 degree')

def reset():
	dataCMD = json.dumps({'var': "reset"})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	print('reset')

def lightCtrl(colorName, cmdInput):
	colorNum = 0
	if colorName == 'off':
		colorNum = 0
	elif colorName == 'blue':
		colorNum = 1
	elif colorName == 'red':
		colorNum = 2
	elif colorName == 'green':
		colorNum = 3
	elif colorName == 'yellow':
		colorNum = 4
	elif colorName == 'cyan':
		colorNum = 5
	elif colorName == 'magenta':
		colorNum = 6
	elif colorName == 'cyber':
		colorNum = 7
	dataCMD = json.dumps({'var':"light", 'val':colorNum})
	ser.write(dataCMD.encode())


def buzzerCtrl(buzzerCtrl, cmdInput):
	dataCMD = json.dumps({'var':"buzzer", 'val':buzzerCtrl})
	ser.write(dataCMD.encode())

print("Imported final_localization.")
robotPose = []

if __name__ == '__main__':
	try:
		
		#with open('output.txt', 'w') as file:
			#file.write(  ' '+ '\n')
		#apriltaggetpositionThread.start()
		#print('apriltag Thread start')
		time.sleep(1)
		walkThread.start()
		print('walk Thread start')
		time.sleep(2)
		while 1:
			time.sleep(2)
			continue
		
	except KeyboardInterrupt:
		print('STOOOOOOOOOOOOOOOOOOOOOP')
		iswalk=0
		walkThread.join()
		#avp.aptIsRunning = False
		#apriltaggetpositionThread.join()
		ser.close()
		
		
		
		
		
		
		
		
		
		
		
		


