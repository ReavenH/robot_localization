#!/usr/bin/env python

from argparse import ArgumentParser
import numpy as np
import os
import cv2
import math

import time
import json
import serial
import threading

import sys
sys.path.append("/home/pi/WAVEGO/RPi/small_function_group/AprilTag/scripts")
import apriltag_video_picamera2 as avp
import calibrationfunc as cf
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

dataCMD = json.dumps({'var':"", 'val':0, 'ip':""})
upperGlobalIP = 'UPPER IP'
tag_info = np.empty((0, 5))  # empty array, 0 row, 5 columns: tag_id, distance, Euler angles (zyx)
pitch, roll = 0, 0
#YAO, 3.16.2024
#if isfront=5: move forward, if isfront=-5: move backward
isfront=5
iswalk = 1

def apriltaggetpositionThreadFunc():
	global tag_info
	avp.apriltag_video(output_stream=False,display_stream=False,print_log=False,cameraMatrix=cf.cameraMatrix,distCoeffs=cf.distCoeffs)

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

def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


#YAO, 3.16.2024, add threading of walking on the board
def WalkOnBoard():
	global tag_info
	global isfront
	global iswalk
	last_time = time.time()
	while iswalk:
		value_str = ser.readline().decode().strip()
		global_step = -1
		if is_float(value_str):
			global_step = float(value_str)
        
		current_time = time.time()
		tag_info = avp.tag_info
		#skip if the apriltag is not detected
		if len(tag_info)==0:
			continue
			
		yaw = round(float(tag_info[0][1]),3)
		z_distance = round(float(tag_info[0][2]),3)
		x_distance = round(float(tag_info[0][4]),3)
		## mean value filter the z_distance
		z_bias = 1.5
		z_distance_filter.add_data_point(z_distance)
		z_distance = z_distance_filter.get_filtered_value()+z_bias
		#print("yaw is: ",yaw, " x distance is: ",x_distance,"z distance is: ",z_distance)
		print(z_distance)
		if yaw>10.0:
			left15()
		if yaw<-10.0:
			right15()
		
		#inverse the direction
		if x_distance<2:
			isfront = -5
		if x_distance>20:
			isfront = 5
		
		#adjust the direction every 5 second
		#measure the z_distance when in the initial stance phase, and leave 0.1s to make the robot adjust the 
		if global_step == 0.1:
			if z_distance>-1 and z_distance<1:
				if isfront >0:
					freewalk(0.0)
				else:
					freewalk(-180.0)
			else:
				direction_deg = math.degrees(math.atan2(z_distance,isfront))
				print("yaw is: ",yaw," x distance is: ",x_distance," z distance is: ",z_distance," direction is: ",direction_deg)
				freewalk(direction_deg+yaw/2)
			last_time = current_time
		
walkThread = threading.Thread(target=WalkOnBoard)			
apriltaggetpositionThread = threading.Thread(target=apriltaggetpositionThreadFunc)

def setUpperIP(ipInput):
	global upperGlobalIP
	upperGlobalIP = ipInput

def forward(speed=50):
	dataCMD = json.dumps({'var':"move", 'val':1})
	ser.write(dataCMD.encode())
	print('robot-forward')

def backward(speed=100):
	dataCMD = json.dumps({'var':"move", 'val':5})
	ser.write(dataCMD.encode())
	print('robot-backward')

def left(speed=50):
	dataCMD = json.dumps({'var':"move", 'val':2})
	ser.write(dataCMD.encode())
	print('robot-left')

def right(speed=100):
	dataCMD = json.dumps({'var':"move", 'val':4})
	ser.write(dataCMD.encode())
	print('robot-right')

def stopLR():
	dataCMD = json.dumps({'var':"move", 'val':6})
	ser.write(dataCMD.encode())
	print('robot-stop')

def stopFB():
	dataCMD = json.dumps({'var':"move", 'val':3})
	ser.write(dataCMD.encode())
	print('robot-stop')


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


def freewalk(degree):
	print('freewalk direction: ',degree)
	dataCMD = json.dumps({'var':"freewalk", 'val':degree})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	
	

def right15():
	dataCMD = json.dumps({'var': "crabwalk", 'val': 0})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	dataCMD = json.dumps({'var': "funcMode", 'val': 17})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	
def left15():
	dataCMD = json.dumps({'var': "crabwalk", 'val': 1})
	ser.write(dataCMD.encode())
	time.sleep(0.1)
	dataCMD = json.dumps({'var': "funcMode", 'val': 17})
	ser.write(dataCMD.encode())
	time.sleep(0.1)

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




if __name__ == '__main__':
	try:
		apriltaggetpositionThread.start()
		print('apriltag Thread start')
		time.sleep(1)
		walkThread.start()
		print('walk Thread start')
		#time.sleep(5)
		startwalk()
		while 1:
			continue
			
	except KeyboardInterrupt:
		print('STOOOOOOOOOOOOOOOOOOOOOP')
		stopLR()
		stopFB()
		iswalk=0
		walkThread.join()
		avp.aptIsRunning = False
		apriltaggetpositionThread.join()
		ser.close()
