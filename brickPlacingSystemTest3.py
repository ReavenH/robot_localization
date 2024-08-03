# on dog 2

import subprocess
subprocess.run('sudo pigpiod', shell=True, check=True)
import pigpio
import time
import json

# GPIO INIT
pi = pigpio.pi()

min_pulsewidth = 500 
max_pulsewidth = 2500  

servoNo = [17, 27, 22]

# read the servo config files.
with open("dog2ServoConfig.json", 'r', encoding='utf-8') as json_file:
    servoCriticalAngles = json.load(json_file)

servoAngles = [servoCriticalAngles["linkageUp"], servoCriticalAngles["brickUp"], servoCriticalAngles["gripperLoose"]]

def servoIOInit(servoNo, frequency):
    for pin in servoNo:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin, frequency)

def singleServoCtrl(number_servo, angle, speed):
    while angle != servoAngles[number_servo]:
        servoAngles[number_servo] += min(max(angle-servoAngles[number_servo],-speed),speed)
        pi.set_servo_pulsewidth(servoNo[number_servo], servoAngles[number_servo])
        # time.sleep(0.01)

def placeBrick(progress=9):
    if progress >= 1:
        # linkage down.
        singleServoCtrl(0, servoCriticalAngles["linkageTilt"], 1/10)
        time.sleep(0.5)

    if progress >= 2:
        # align brick.
        singleServoCtrl(1, servoCriticalAngles["brickVertical"], 1/20)  # rotate brick board to vertical.
        time.sleep(0.5)
        singleServoCtrl(2, servoCriticalAngles["gripperAlign"], 1/2)  # gripper open
        time.sleep(1)
        singleServoCtrl(2, servoCriticalAngles["gripperFasten"], 1/2)  # gripper fasten
        time.sleep(0.5)

    if progress >= 3:
        # rotate brick.
        singleServoCtrl(1, servoCriticalAngles["brickDown"], 1/50)  # changed
        time.sleep(1)

    if progress >= 4:
        # linkage down.
        singleServoCtrl(0, servoCriticalAngles["linkageDown"], 1/10)  # changed
        time.sleep(1)

    if progress >= 5:
        # release brick.
        singleServoCtrl(2, servoCriticalAngles["gripperLoose"], 1/10)
        time.sleep(2)

    if progress >= 6:
        # linkage slightly up.
        singleServoCtrl(0, servoCriticalAngles["linkageSlightlyUp"], 1/2)
        time.sleep(1)

    if progress >= 7:
        # rotate brickboard.
        singleServoCtrl(1, servoCriticalAngles["brickUp"], 1/2)
        time.sleep(0.5)

    if progress >= 8:
        # fasten gripper.
        singleServoCtrl(2, servoCriticalAngles["gripperClose"], 1/2)
        time.sleep(0.5)

    if progress >= 9:
        # linkages up.
        singleServoCtrl(0, servoCriticalAngles["linkageUp"], 10)

def resetPose():
    singleServoCtrl(0, servoAngles[0], 1/2)
    time.sleep(0.5)
    singleServoCtrl(1, servoAngles[1], 1/2)
    time.sleep(0.5)
    singleServoCtrl(2, servoAngles[2], 1/2)
    time.sleep(0.5)

def openGripper():
    singleServoCtrl(2, servoCriticalAngles["gripperLoose"], 1/2)

def closeGripper():
    singleServoCtrl(2, servoCriticalAngles["gripperFasten"], 1/2)

# add some critical frames of the whole motion sequence.
def pushBrick():
    singleServoCtrl(0, servoCriticalAngles["linkageSlightlyUp"] - 200, 1/2)

try:

    # check if connected
    if not pi.connected:
        exit()

    servoIOInit(servoNo, 50)
    waitKey = True

    while(True):
        action = int(input("Action? Full=1 | Reset=2 | Open Gripper=3 | Close Gripper=4 | Choose Progress=5 | Push Brick Sightly=6\n"))
        if action == 1:
            waitKey = False
            print("Placing Brick!\n")
            placeBrick()
            print("Done")
        if action == 2:
            waitKey = False
            print("Pose Reset\n")
            resetPose()
            print("Done")
        if action == 3:
            waitKey = False
            print("Open Gripper\n")
            openGripper()
            print("Done")
        if action == 4:
            waitKey = False
            print("Close Gripper\n")
            closeGripper()
            print("Done")
        if action == 5:
            waitKey = False
            # print("Choose the Progress of Placing\n")
            progress = int(input("Input the desired process:\n"))
            placeBrick(progress = progress)
            print("Done")
        if action == 6:
            waitKey = False
            print("Push Forward Slightly\n")
            pushBrick()
            print("Done")

    

except KeyboardInterrupt:
    pi.stop()
    subprocess.Popen(["sudo", "killall", "pigpiod"])
    print("Program Exited")
