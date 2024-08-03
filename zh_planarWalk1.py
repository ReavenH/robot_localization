from zh_Utilities import hmRPYG, robot, ser
import cv2
import time
import numpy as np
import os
import serial

'''
TODO:
1. Separate the system into schedular and the implementation part.
2. Use FBLR to represent the direction to go, use S to represent stop.
3. input of the schedular: the number of passed crossings -> to search the action to take at the current crossing.
4. input of the motionController: the alphabet of the motion to take.
'''