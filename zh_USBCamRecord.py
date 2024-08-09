import cv2

cap = cv2.VideoCapture('/dev/video1')

if not cap.isOpened():
    print("Camera Fails to Open!")
    exit()

frame_width = 640
frame_height = 480

cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

out = cv2.VideoWriter('zh_USBCamRecording.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 100, (frame_width, frame_height))

while True:
    # 读取一帧
    ret, frame = cap.read()

    # 如果帧读取成功
    if ret:
        # 将帧写入文件
        out.write(frame)

        # 显示帧
        cv2.imshow('Bottom Camera Frame', frame)

        # 按 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# 释放 VideoCapture 和 VideoWriter 对象
cap.release()
out.release()

# 关闭所有 OpenCV 窗口
cv2.destroyAllWindows()
