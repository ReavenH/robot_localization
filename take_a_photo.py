#!usr/bin/python3

from picamera2 import Picamera2
import time
import cv2

# cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
capture_config = picam2.create_still_configuration()
picam2.start(show_preview = True)

i = 1
try:
	
	# im = picam2.capture_array()
	# cv2.imshow("Camera", im)
	input_key = input("Enter to take a photo...")
	
		# cv2.imwrite("CapturedImages/pho_l.jpg", im)
	filename = "CapturedImages/photo_right.jpg"
	picam2.switch_mode_and_capture_file(capture_config, filename)
	print("Saved photo.")

except KeyboardInterrupt:
	picam2.stop()

time.sleep(2)
