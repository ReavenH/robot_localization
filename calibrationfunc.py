import numpy as np
import cv2

# define public calibration params
cameraMatrix = np.array([[1269.9875, 0, 1273.97291],
    [0, 1266.71028, 989.41452],
    [0, 0, 1]], dtype=np.float32) * np.array([[640/2592, 0, 480/1944],[0, 640/2592, 480/1944], [0, 0, 1]])

distCoeffs = np.array([-0.363389512, 0.124199997, -0.00250641406, 0.0000739587212, 0.0784944996], dtype=np.float32)

def calibrate_frame(img, cameraMatrix, distCoeffs):

    undistorted_img = cv2.undistort(img, cameraMatrix, distCoeffs)
    
    return undistorted_img

if __name__ == "__main__":
    im = cv2.imread("/home/pi/WAVEGO/RPi/small_function_group/CapturedImages/photo4.jpg")
    im = cv2.resize(im, (640, 480))
    im = calibrate_frame(im, cameraMatrix, distCoeffs)
    cv2.imshow("calibrated", im)
    key = cv2.waitKey(0)
    if key == 27:
        cv2.destroyAllWindows()
