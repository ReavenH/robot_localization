#!/usr/bin/env python

from argparse import ArgumentParser
import numpy as np
import os
import cv2
import apriltag
from calibrationfunc import *
import math

os.system("sudo chmod 777 /dev/ttyS0")
# new Feb 22, import scipy for calc Euler Angles
from scipy.spatial.transform import Rotation as R

# new Feb 20, import camera lib
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
# new Feb 28, test different exposure time
picam2.set_controls({"ExposureTime": 50000, "AnalogueGain": 1.5})
# new Feb 22, define global for storing tag_id, distance, Euler Angles
tag_info = np.empty((0, 5))  # empty array, 0 row, 5 columns: tag_id, distance, Euler angles (zyx)

# new Feb 22, define global flag for thread control
aptIsRunning = True

# new Feb 22, define the video writer
video_name = "waveshare_apriltag_640x480.mp4"
fps = 15.0
codec = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(video_name, codec, fps, (640, 480), isColor = True)

################################################################################

def apriltag_video(output_stream=True,
                   display_stream=True,
                   detection_window_name='AprilTag',
                   tagsize = 0.0790,  # meters
                   cameraparams = np.array([1269.9875*640/2592, 1266.71028*480/1944, 1273.97291*640/2592, 989.41452*480/1944]),
                   cameraMatrix = None,
                   distCoeffs = None,
                   print_log = False
                  ):
                      
    picam2.start()  # start camera, without showing preview
    
    # new Feb 29, define a new variable which store the number of frames and number of detection in 5 seconds
    number_of_frames=0
    number_of_detection=0
    
    # new Feb 28 get the default exposure time which is around 66000 us
    #metadata = picam2.capture_metadata()
    #print(metadata["ExposureTime"], metadata["AnalogueGain"])

    
    global tag_info, aptIsRunning
    
    parser = ArgumentParser(description='Detect AprilTags from video stream.')
    apriltag.add_arguments(parser)
    options = parser.parse_args()

    detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())
    
        
    while aptIsRunning:
            
        frame = np.array(picam2.capture_array()) # returns normal distorted image
        frame = calibrate_frame(frame, cameraMatrix, distCoeffs)  # abnormaly distorted, but can be used independently
        overlay = frame
        
        # new Feb 29, while aptIsRunning add one for each detected frames
        number_of_frames=number_of_frames+1
            
        results, overlay = apriltag.detect_tags(frame,
                                                detector,
                                                camera_params=cameraparams,
                                                tag_size=tagsize,
                                                vizualization=3,
                                                verbose=0, # 0 - silent, 1 - no of detections, 2 - detection data, 3 - detection amd pose data
                                                annotation=True
                                                )
        
            
        if results != []:
			
			# new Feb 29, while aptIsRunning add one for each frames
            number_of_detection=number_of_detection+1
                
            tag_info = np.empty((0, 5))  # dump the previously stored detections
            # print(results)  # for debug only, "results" is a list
            
            robot_absolute_pos_tag_final=[0,0,0]
            cnt_absolute=0
                
            for idx in np.arange(0, int(len(results)), 4):  # 1 tag result -> 4 elements
                result = results[idx]  # the Apriltag.detection object
                    
                # distance calculation(calculate the diagonal line)
                #diag_pixel = np.sqrt((np.sum(result.corners[2] - result.corners[0]) ** 2))
                #diag_focal = np.sqrt(np.sum(cameraparams[:2] ** 2))
                distance = np.sqrt(np.sum(results[idx + 1][:3, 3] ** 2))
                # Euler Angles calculation
                # create rotation object
                rotationMatrix = results[idx + 1][:3, :3] 
                # calculate Euler Angles. Right hand rule.
                # z-axis : perpendicular to the surface of the tag, positive if tag is rotated clockwise
                # y-axis : pointing upwards, positive if looked from the right
                # x-axis : pointing to the right, positive if looked from the above
                #eulerAngles = rotationMatrix.as_euler('zyx', degrees = True)
                
                # Position vector of the robot in the AprilTag's coordinate system, assume the distance is almost in z-axis
                #position_vector_init = np.array([0, 0, distance])
                
                # Apply the rotation matrix to the position vector
                #R_inv = np.linalg.inv(rotationMatrix)
                #position_vector_after = np.dot(R_inv, position_vector_init) 
                
                # Get the absolute position (x,y,z)
                rotationMatrix_foruse = R.from_matrix(results[idx + 1][:3, :3]) 
                eulerAngles = rotationMatrix_foruse.as_euler('zyx', degrees = True) 
                
                robot_absolute_pos_tag=[0,0,0]
                robot_absolute_pos_tag_final=[0,0,0]
                cnt_absolute=0
                yaw=0
                yaw_r=0
                degrees=0
                if(result.tag_id==14):
                    position_vector_init = np.array([-results[idx + 1][0][3] , -results[idx + 1][1][3], results[idx + 1][2][3]]) 
                    R_inv = np.linalg.inv(rotationMatrix)
                    position_vector_after = np.dot(R_inv,position_vector_init)    
                    tag18_pos = np.array([0, 0, 0.15])
                    robot_absolute_pos_tag = tag18_pos + position_vector_init
                    if not all(value == 0 for value in robot_absolute_pos_tag):
                        cnt_absolute+=1
                        robot_absolute_pos_tag_final = [sum(x) for x in zip(robot_absolute_pos_tag, robot_absolute_pos_tag_final)]

                if(result.tag_id==18):
                    position_vector_init = np.array([-results[idx + 1][0][3] , -results[idx + 1][1][3], results[idx + 1][2][3]])
                     
                    #R_inv = np.linalg.inv(rotationMatrix)
                    #position_vector_after = np.dot(-R_inv,position_vector_init)
                        
                    tag18_pos = np.array([0, 0, -0.38])
                    #get the new bias angle
                    radians = np.arcsin(-results[idx + 1][0][3] / results[idx + 1][2][3])
                    degrees = np.degrees(radians)
                    #get yaw  
                    yaw=eulerAngles[1]
                    real_yaw=yaw+degrees
                    real_yaw_r=np.radians(real_yaw)
                    
                    robot_absolute_pos_tag = tag18_pos + position_vector_init
                    #get x and zS
                    robot_absolute_pos_tag[0]=(robot_absolute_pos_tag[2]*100)*np.sin(real_yaw_r)
                    robot_absolute_pos_tag[2]=(robot_absolute_pos_tag[2]*100)*np.cos(real_yaw_r)
                    #if not all(value == 0 for value in robot_absolute_pos_tag):
                        #cnt_absolute+=1
                        #robot_absolute_pos_tag_final = [sum(x) for x in zip(robot_absolute_pos_tag, robot_absolute_pos_tag_final)]
                        
                if(result.tag_id==17):
                    position_vector_init = np.array([-results[idx + 1][0][3] , -results[idx + 1][1][3], results[idx + 1][2][3]]) 
                    R_inv = np.linalg.inv(rotationMatrix)
                    position_vector_after = np.dot(R_inv, position_vector_init) 
                    tag17_pos = np.array([0.45, 0, 0])
                    if tag17_pos[0]>0 and tag17_pos[1]==0 and tag17_pos[2]==0:
                        yaw=eulerAngles[1]
                        
                        
                    robot_absolute_pos_tag = tag17_pos + position_vector_after
                    if not all(value == 0 for value in robot_absolute_pos_tag):
                        cnt_absolute+=1
                        robot_absolute_pos_tag_final = [sum(x) for x in zip(robot_absolute_pos_tag, robot_absolute_pos_tag_final)]
                        
                        
                
                # store the values in the dict 
                tag_info = np.vstack((tag_info, np.array([yaw, degrees, robot_absolute_pos_tag[0],robot_absolute_pos_tag[1]*100,robot_absolute_pos_tag[2] ])))
                
                
                if print_log:
                    #print(tag_info[0][2]) # This is X
                    print(tag_info) 
                    #with open('output.txt', 'a') as file:
                        #file.write(str(tag_info) + '\n')
                  
            # get the average absolute position
            #if(cnt_absolute!=0):
                #robot_absolute_pos_tag_final = [x/cnt_absolute for x in robot_absolute_pos_tag_final]
                #print("Average result: ",robot_absolute_pos_tag_final)
              
        if output_stream:
            video_writer.write(overlay)
            
        if display_stream:
            cv2.imshow(detection_window_name, overlay)
            
            if cv2.waitKey(1) & 0xFF == ord(' '): # Press space bar to figure window
                cv2.destroyAllWindows()
                video_writer.release()
                picam2.stop()
                print("Detected Space, exited...")
                break
    # new Feb 29, print the detection rate           
    prop_of_detection=number_of_detection/number_of_frames
    print("The detection rate is: ", prop_of_detection)
    # new March 5, return tag_info
    return tag_info
    
                
                    

        

################################################################################

# new Feb 20: start a seperate thread to control the figure window
# prevent blocking the main thread.
# cv2.startWindowThread()

if __name__ == '__main__':
    
    try:
        
        apriltag_video(output_stream = True, cameraMatrix = cameraMatrix, distCoeffs = distCoeffs, print_log = True)
    
    except KeyboardInterrupt:
        
        cv2.destroyAllWindows()
        video_writer.release()
        picam2.stop()
        ser.close()
        print("stopped picam")
