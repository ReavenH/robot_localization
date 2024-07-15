import subprocess
subprocess.run('sudo pigpiod', shell=True, check=True)
import pigpio
import time

# GPIO INIT
pi = pigpio.pi()

min_pulsewidth = 500 
max_pulsewidth = 2500  

servoAngles = [500, 500, 500]
servoNo = [17, 27, 22]

def servoIOInit(servoNo, frequency):
    for pin in servoNo:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_PWM_frequency(pin, frequency)

def singleServoCtrl(number_servo, angle, speed):
    while angle != servoAngles[number_servo]:
        servoAngles[number_servo] += min(max(angle-servoAngles[number_servo],-speed),speed)
        pi.set_servo_pulsewidth(servoNo[number_servo], servoAngles[number_servo])
        # time.sleep(0.01)

def placeBrick():
    # linkage down.
    singleServoCtrl(0, 1200, 1/10)
    time.sleep(1)

    # align brick.
    singleServoCtrl(1, 750, 1/50)  # rotate brick board to vertical.
    time.sleep(1)
    singleServoCtrl(2, 1800, 1/2)  # gripper open
    time.sleep(1)
    singleServoCtrl(2, 1700, 1/2)  # gripper fasten
    time.sleep(1)

    # rotate brick.
    singleServoCtrl(1, 1800, 1/50)  # changed
    time.sleep(1)

    # linkage down.
    singleServoCtrl(0, 1000, 1/10)  # changed
    time.sleep(1)

    # release brick.
    singleServoCtrl(2, 2500, 1/2)
    time.sleep(2)

    # rotate brickboard.
    singleServoCtrl(1, 500, 1/50)
    time.sleep(1)

    # fasten gripper.
    singleServoCtrl(2, 500, 1/2)
    time.sleep(1)

    # linkages up.
    singleServoCtrl(0, 500, 10)

def resetPose():
    singleServoCtrl(0, 500, 1/2)
    singleServoCtrl(1, 500, 1/2)
    singleServoCtrl(2, 500, 1/2)

def openGripper():
    singleServoCtrl(2, 2500, 1/2)

def closeGripper():
    singleServoCtrl(2, 1700, 1/2)

try:

    # check if connected
    if not pi.connected:
        exit()

    servoIOInit(servoNo, 50)
    waitKey = True

    while(True):
        numIn = input("Input a value: servo No. or PWM\n")
        if numIn <=3 and numIn >=1:
            desiredServo = numIn
            desiredPWM = int(input("Input desired PWM for Servo {}\n".format(desiredServo)))
            desiredServo = int(input("Input servo No (0=baseBoard, 1=brickBoard, 2=gripperGear)\n"))
            
            if desiredServo == 0:
                speed = 1/2
            else:
                speed = 1/10
        singleServoCtrl(desiredServo, desiredPWM, speed)
    

except KeyboardInterrupt:
    pi.stop()
    subprocess.Popen(["sudo", "killall", "pigpiod"])
    print("Program Exited")
