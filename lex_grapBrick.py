from zh_Utilities import robot, hmRPYG, ser, pi
import subprocess
import time

subprocess.run('sudo pigpiod', shell=True, check=True)
import pigpio
pi = pigpio.pi()

myRobot = robot(hmRPYG, None, None, ser, vidsrc = 0, servoConfig="dog3ServoConfig.json") 
myRobot.resetPose()

myRobot.buzzer(True)
time.sleep(2.5)
myRobot.buzzer(False)

myRobot.singleServoCtrl(0, 1005, 0.1)
myRobot.singleServoCtrl(1, 590, 0.1)
myRobot.closeGripper()
time.sleep(1)
myRobot.openGripper()


time.sleep(20)
myRobot.closeGripper()

myRobot.buzzer(True)
time.sleep(5)
myRobot.buzzer(False)

myRobot.ser.close()
pi.stop()
subprocess.Popen(["sudo", "killall", "pigpiod"])
