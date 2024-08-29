# Construction Robot Codebase
## 1. Features
### 1.1 Visualization
- Compatible with WaveGo Inverse Kinematics, draws leg linkages in real-time;
- Compatible with Matplotlib and OpenGL for visualization (using OpenGL by default for faster speed);
- Compatible to draw the structure of bricks if their poses are known;
- Compatible with Apriltag pose estimation for the robot body;
### 1.2 Visual Pose Estimation
- Front camera -> AprilTags
- Bottom camera -> extract yaw and offset from pattern on bricks (currently using circle patterns)
### 1.3 Brick Placing System Control
- Using a '.json' file to store all the critical values of a calibrated servo.
- Functionalities integrated into the 'robot' class.
### 1.4 Modular Coding
- Modular functions / classes in the library 'zh_Utilities.py'.
- Most robot related functionalities are interated into the 'robot' class.
- Code can be executed on both Linux and Windows, the 'robot' class checks the platform when it is initialized.

## 2 File Directory
### 2.1 Brick Placing System Control
- 'dog2ServoConfig.json' and other similar files: stores the servo values.
- 'zh_servoTest.py': top level script to execute a full motion sequence of placing a brick.
- 'calibrateLinkageServos.py': a primitive version of servo calibration control.
### 2.2 OpenGL Visualization
- 'zh_DrawSceneOpenGL.py': only draws the environment, robot is not included. Displays a pseudo motion sequence of the robot.
- 'zh_drawRobotOG.py': draws the robot body and the environment, the robot body now supports inverse kinematics.
### 2.3 Bottom Camera Pose Detection
- 'zh_HBVCameraCircles1.py': the final version of the pose detection functionality, which initializes the robot class to implement pose detection from the bottom camera. Faster and up-to-date algorithm.
- 'zh_HBVCameraCircles.py': a test script that shows the camera circle detection outputs and pose result without filtering.
- 'zh_HBVCameraChessboard.py': a previous test script on chessboard patterns.
### 2.4 AprilTag Localization
- 'zh_robotPose.py': an updated code to detect pose from AprilTags. Should be used along with 'zh_Utilities.py'!
- 'zh_robotPoseProximity.py': added proximity line detection in addition to the above file.
- 'zh_PCReceiver.py': the PC visualizer using 'matplotlib'.
- 'hp_LineTracking.py': the line tracking function module based on color tapes. To be imported in other scripts.
- 'hy_LineTracking.py': the full line tracking demo using proximity sensors.
### 2.5 The Utility Libraries
- 'zh_Utilities.py': the main lib for objects including the robot, landmarks, brick's poses and other variables and functions.
- 'final_localization.py': the script that usually runs in a seperate thread to control the ESP32 with AprilTag fiducials.
- 'apriltag_video_picamera4.py': the lib for AprilTag functionalities.
- 'calibrationfunc.py': stores the front cameras's inner and outer params, and defines an undistortion function to be called in the above script. 
