import cv2
import numpy as np
from scipy.io import savemat
from scipy.spatial import distance

disThres = 45.0 # default 50 IN 160X120. the distance threshold for grouping larger circles into 2.
previousCircles = 0
nextCircles = 0

def displayFrame(frame):
    # Display the processed frame
    cv2.imshow('Detected Circles', frame)
    while not cv2.waitKey(1) & 0xFF == ord('w'): 
        continue

def groupCircles(centers, disMtx):
    global previousCircles, nextCircles
    if disMtx is not None:
        disMtxVisited = np.eye(disMtx.shape[0]).astype('bool')
        group1 = np.array([])
        group2 = np.array([])
        for row in range(disMtx.shape[0]):
            for column in range(disMtx.shape[1]):
                if disMtxVisited[row, column] == True:
                    continue
                else:
                    disMtxVisited[row, column] = True
                    disMtxVisited[column, row] = True
                    if disMtx[row, column] >= disThres:
                        if row in group1:
                            if column in group2:
                                continue
                            else:
                                group2 = np.append(group2, column)
                        elif row in group2:
                            if column in group1:
                                continue
                            else:
                                group1 = np.append(group1, column)
                        else:
                            if column in group1:
                                group2 = np.append(group2, row)
                            elif column in group2:
                                group1 = np.append(group1, row)
                            else:
                                group1 = np.append(group1, row)
                                group2 = np.append(group2, column)
                    else:
                        if row in group1:
                            if column in group1:
                                continue
                            else:
                                group1 = np.append(group1, column)
                        elif row in group2:
                            if column in group2:
                                continue
                            else:
                                group2 = np.append(group2, column)
                        else:
                            if column in group1:
                                group1 = np.append(group1, row)
                            elif column in group2:
                                group2 = np.append(group2, row)
                            else:
                                group1 = np.append(group1, row)
                                group1 = np.append(group1, column)
        '''
        if len(np.intersect1d(group1, group2)) > 0:
            group1 = np.union1d(group1, group2)
            group2 = np.array([])
        '''
        print("Group1: {} | Group2: {}\n".format(group1, group2))
        comGroup1 = np.mean(centers[group1.astype('int')], axis=0)
        comGroup2 = np.mean(centers[group2.astype('int')], axis=0)
        if comGroup1[1] >= comGroup2[1]:
            previousCircles = len(group1)
            nextCircles = len(group2)
        else:
            previousCircles = len(group2)
            nextCircles = len(group1)
    return previousCircles, nextCircles

# Function to detect and display circles
def detect_and_display_circles(frame, minDist = 10, minRadius = 15, maxRadius = 25):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply median blur to reduce noise
    gray_blurred = cv2.medianBlur(gray, 5)
    
    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=minDist, param1=48, param2=19, minRadius=minRadius, maxRadius=maxRadius)  # param1=50, param2=30
    
    # Ensure at least one circle was found
    if circles is not None:
        circles = np.uint16(np.around(circles))
        count = 0
        centers = []
        for circle in circles[0, :]:

            count += 1
            center = (circle[0], circle[1])  # Circle center
            centers.append(center)
            print("No {} circle's center: {}".format(count, center))
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
            '''
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
            '''
            # get the mutual distance matrix.
            disMtx = distance.cdist(centers, centers, metric='euclidean')
            print("disMtx: \n{}\n".format(disMtx))
            displayFrame(frame)
            return centers, disMtx

        else:
             print("Only {} circle detected, unable to calculate gradient.")
             displayFrame(frame)
             return centers, None
    else:
         print("No circle in this frame.")
         displayFrame(frame)
         return None, None

# Open the camera (0 is usually the default camera)
cap = cv2.VideoCapture("/dev/video1")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break
    
    # detect_and_display_circles(frame, minDist=10, minRadius=2, maxRadius=10)  # 160x120
    # detect_and_display_circles(frame, minDist=20, minRadius=13, maxRadius=20)  # 320x240, circle traces.
    centers, disMtx = detect_and_display_circles(frame, minDist=13, minRadius=4, maxRadius=8)
    
    # Group stigmergy circles.
    if centers is not None:
        groupCircles(centers, disMtx)
    print("Previous Cicles: {} | Next Circles: {}\n".format(previousCircles, nextCircles))
    print("-----------------------------------------------------------------------------")

    # Exit the loop if 'q' is pressed
    '''
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    '''
    
# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
