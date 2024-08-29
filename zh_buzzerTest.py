from zh_Utilities import ser, robot, hmRPYG
import time

if __name__ == "__main__":
    myRobot = robot(hmRPYG, None, None, ser)
    '''
    myRobot.buzzer(True)
    time.sleep(1)
    myRobot.buzzer(False)
    time.sleep(1)
    myRobot.buzzer(True)
    time.sleep(1)
    '''
    myRobot.buzzer(False)
    time.sleep(1)

    '''
    myRobot.buzzerPWM(400)
    time.sleep(1)
    myRobot.buzzerPWM(600)
    time.sleep(1)
    myRobot.buzzerPWM(800)
    time.sleep(1)
    myRobot.buzzerPWM(1000)
    time.sleep(1)
    myRobot.buzzerPWM(1200)
    time.sleep(1)
    '''
    myRobot.ser.close()
    print("Program exited.")
