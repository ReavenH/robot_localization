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
import re

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



def readimu():
    
    try:
        value_str = ser.readline().decode().strip()
    except Exception as e:
        print(f"Error reading from serial port: {e}")
        return
    print(value_str)    
    robot_time = re.search(r'Time: (\d+)', value_str)  # int
    
    yaw = re.search(r'Yaw: (-?\d+\.\d+)', value_str)  # float
    dx = re.search(r'dx: (-?(\d+\.\d+|inf))', value_str)  # float
    if robot_time:
        # get the time in second
        print(int(robot_time.group(1)))
    else:
        print("No match found")
        
    if yaw:
        print(yaw.group(1))
    else:
        print("No match found")
        
    if dx:
        print(dx.group(1))
    else:
        print("No match found")        
        
def forward(speed=50):
	dataCMD = json.dumps({'var':"move", 'val':1})
	ser.write(dataCMD.encode())
	print('robot-forward')
	
def stopLR():
	dataCMD = json.dumps({'var':"move", 'val':6})
	ser.write(dataCMD.encode())
	print('robot-stop')

def stopFB():
	dataCMD = json.dumps({'var':"move", 'val':3})
	ser.write(dataCMD.encode())
	print('robot-stop')	
	

if __name__ == '__main__':
	
	
    duration = 5
    start_time = time.time()  
    end_time = start_time + duration  
    
    forward()
    while time.time() < end_time:
        readimu()
        
    stopFB()
    stopLR()	
    
			
	
		
