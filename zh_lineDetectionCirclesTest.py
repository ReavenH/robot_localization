import cv2
import numpy as np

# 定义视频流的路径或摄像头索引
video_path = 'output1.mp4'
cap = cv2.VideoCapture(video_path)

# 创建一个空白图像，用于存储圆心坐标
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))


while cap.isOpened():
    centers_image = np.zeros((height, width), dtype=np.uint8)

    ret, frame = cap.read()
    if not ret:
        break
    
    # 转换为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 霍夫圆检测
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=30)
    
    # 绘制圆心到 centers_image 上
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(centers_image, (x, y), 2, 255, -1)
    
    # 显示原始图像和二值化的圆心图像
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Centers Image', centers_image)
    # 使用二值化的圆心图像进行霍夫直线变换
    edges = cv2.Canny(centers_image, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges, rho=1, theta=np.pi/180, threshold=50) 
    
    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
