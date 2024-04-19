import zh_robotPose as pose

if __name__ == "__main__":
    try:
        while(True):
            print(pose.previousPose)

    except KeyboardInterrupt:
        print("exit")