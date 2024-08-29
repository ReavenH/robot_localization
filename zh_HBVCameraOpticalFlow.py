import cv2
import numpy as np

'''
ERROR: the window will stall when there is no overlap between the new frame and the previous frame. Or it is 
because there is no usable feature to track.
'''

cap = cv2.VideoCapture(0)
ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
feature_params = dict(maxCorners=10, qualityLevel=0.3, minDistance=7, blockSize=7)  # default maxCorners is 100
lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
mask = np.zeros_like(old_frame)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    if p1 is not None:
        good_new = p1[st == 1]
        good_old = p0[st == 1]
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel().astype(int)
            c, d = old.ravel().astype(int)
            mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 2)
            frame = cv2.circle(frame, (a, b), 5, (0, 255, 0), -1)
        img = cv2.add(frame, mask)
        cv2.imshow('Optical Flow', img)
        # clear the mask.
        mask.fill(0)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)

cap.release()
cv2.destroyAllWindows()
