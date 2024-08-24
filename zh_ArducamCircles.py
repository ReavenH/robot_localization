import cv2
import numpy as np
from scipy.io import savemat
from scipy.spatial import distance

# function to detect the Manhattan centroid.
def getManhattanCentroid(centers):
    '''
    Only use it when there is a crossing.
    input: an np ndarray of centers of circles.
    output: an 1x2 tuple of the manhattan centroid, which is one of the circle centers.
    '''
    # compute the mutual distance.
    disMtx = distance.cdist(centers, centers, metric='euclidean')
    # find the Manhattan centroid.
    sumEucDis = np.sum(disMtx, axis=0)
    mcIdx = np.argmin(sumEucDis)
    mc = centers[mcIdx, :]
    return mc

# Function to detect and display circles
def detect_and_display_circles(frame):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply median blur to reduce noise
    gray_blurred = cv2.medianBlur(gray, 5)
    
    # Detect circles using HoughCircles
    # circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=10, param1=50, param2=21, minRadius=2, maxRadius=10)  # param1=50, param2=30 for 160x120
    circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=25, minRadius=7, maxRadius=20)  # param1=50, param2=30 for 320x240
    # circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1.5, minDist=80, param1=52, param2=25, minRadius=20, maxRadius=50)

    # Ensure at least one circle was found
    if circles is not None:
        circles = np.uint16(np.around(circles))
        count = 0
        centers = []
        for circle in circles[0, :]:

            count += 1
            center = (circle[0], circle[1])  # Circle center
            centers.append(center)
            # print("No {} circle's center: {}".format(count, center))
            radius = circle[2]  # Circle radius
            
            # Draw a small circle at the center
            cv2.circle(frame, center, 2, (0, 255, 0), 1)
            # Draw the detected circle
            cv2.circle(frame, center, radius, (0, 0, 255), 1)
            # Put text.
            cv2.putText(frame, str(count), (center[0] - 2, center[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, str(radius), (center[0] + 2, center[1] + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if count > 1:
            centers = np.array(centers).reshape(-1, 2).astype('float')
            # calculate the gradient between each other.
            x = centers[:, 0]
            y = centers[:, 1]
            # print("x:\n{}".format(x))
            # print("y:\n{}".format(y))
            xx = x[:, np.newaxis]
            yy = y[:, np.newaxis]
            # print("xx:\n{}".format(xx))
            # print("yy:\n{}".format(yy))
            dx = xx - x
            dy = yy - y
            # print("dx:\n{}".format(dx))
            # print("dy:\n{}".format(dy))
            # yaw = np.arctan2(dy, dx) * 180 / np.pi
            yaw = np.arctan(np.divide(dy, dx).copy()) * 180 / np.pi
            print("Mutual gradient of {} circles: \n{}".format(count, yaw))
            
            # save to mat file.
            # savemat('zh_CircleOutput.mat', {"yaw": yaw})

            # calculate the Manhattan centroid.
            mc = getManhattanCentroid(centers).astype('int')
            print("Mahattan Centroid: ", mc)
            cv2.putText(frame, "MC", (mc[0] + 4, mc[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 255), 1)

        else:
             print("Only {} circle detected, unable to calculate gradient.")
    else:
         print("No circle in this frame.")

    # Display the processed frame
    cv2.imshow('Detected Circles', frame)
    
    
    while not cv2.waitKey(1) & 0xFF == ord('w'): 
        continue
    

    print("-----------------------------------------------------")
	
	
# Open the camera (0 is usually the default camera)
cap = cv2.VideoCapture('/dev/video1')

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    # print("framesize: ", frame.shape)
    # print("img datatype: ", frame.dtype)
    if not ret:
        break
    
    # Detect and display circles
    detect_and_display_circles(frame)
    
    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
