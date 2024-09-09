from zh_Utilities import hmRPYG, robot, ser
myRobot = robot(hmRPYG, None, None, ser, config = "dog3Config.json", vidsrc=1)
myRobot.interrupt()