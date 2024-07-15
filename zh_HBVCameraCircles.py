import cv2
import numpy as np
from scipy.io import savemat

# Function to detect and display circles
def detect_and_display_circles(frame):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply median blur to reduce noise
    gray_blurred = cv2.medianBlur(gray, 5)
    
    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=10, param1=50, param2=21, minRadius=2, maxRadius=10)  # param1=50, param2=30
    
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
            savemat('zh_CircleOutput.mat', {"yaw": yaw})

        else:
             print("Only {} circle detected, unable to calculate gradient.")
    else:
         print("No circle in this frame.")

    # Display the processed frame
    cv2.imshow('Detected Circles', frame)
    while not cv2.waitKey(1) & 0xFF == ord('w'): 
        continue

# Open the camera (0 is usually the default camera)
cap = cv2.VideoCapture(2)


cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)


while cap.isOpened():
    # Capture frame-by-frame
    ret, frame = cap.read()
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
