#!/usr/bin/env python

from argparse import ArgumentParser
import numpy as np
import os
import cv2
import apriltag
from calibrationfunc import *

os.system("sudo chmod 777 /dev/ttyS0")
# new Feb 22, import scipy for calc Euler Angles
from scipy.spatial.transform import Rotation as R

# new Feb 20, import camera lib
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
# new Feb 28, test different exposure time
picam2.set_controls({"ExposureTime": 7500, "AnalogueGain": 1.5})
# new Feb 22, define global for storing tag_id, distance, Euler Angles
tag_info = np.empty((0, 5))  # empty array, 0 row, 5 columns: tag_id, distance, Euler angles (zyx)

# new Feb 22, define global flag for thread control
aptIsRunning = True

# new Feb 22, define the video writer
video_name = "waveshare_apriltag_640x480.mp4"
fps = 15.0
codec = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(video_name, codec, fps, (640, 480), isColor = True)

# new Mar 19, store the detection result as a global.
resultsGlobal = []
################################################################################

def apriltag_video(output_stream=True,
                   display_stream=True,
                   detection_window_name='AprilTag',
                   tagsize = 0.0790,  # 0.0790 meters for the smaller ones; 0.1600 meters for the larger ones.
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

    
    global tag_info, aptIsRunning, resultsGlobal
    
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

        resultsGlobal = results
            
        if results != []:
			
			# new Feb 29, while aptIsRunning add one for each frames
            number_of_detection=number_of_detection+1
                
            tag_info = np.empty((0, 5))  # dump the previously stored detections
            # print(results)  # for debug only, "results" is a list
                
            for idx in np.arange(0, int(len(results)), 4):  # 1 tag result -> 4 elements
                result = results[idx]  # the Apriltag.detection object
                    
                # distance calculation
                diag_pixel = np.sqrt((np.sum(result.corners[2] - result.corners[0]) ** 2))
                diag_focal = np.sqrt(np.sum(cameraparams[:2] ** 2))
                distance = np.sqrt(np.sum(results[idx + 1][:3, 3] ** 2))
                # Euler Angles calculation
                # create rotation object
                rotationMatrix = R.from_matrix(results[idx + 1][:3, :3])  
                # calculate Euler Angles. Right hand rule.
                # z-axis : perpendicular to the surface of the tag, positive if tag is rotated clockwise
                # y-axis : pointing upwards, positive if looked from the right
                # x-axis : pointing to the right, positive if looked from the above
                eulerAngles = rotationMatrix.as_euler('zyx', degrees = True) 
                # eulerAngles = rotationMatrix.as_euler('xyz', degrees = True)
                
                # store the values in the dict
                tag_info = np.vstack((tag_info, np.array([result.tag_id, distance, eulerAngles[0], eulerAngles[1], eulerAngles[2]])))
                    
                if print_log:
                    print(tag_info)
                    # print(results[idx + 1][:3, 3])
                    print(results[idx + 1])

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
        print("stopped picam")
