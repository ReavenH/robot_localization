import cv2
import numpy as np

def filter_yellow_color(image):
    # 将图像转换到HSV色彩空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 定义黄色的阈值范围
    lower_yellow = np.array([20, 100, 100])  # 可根据实际情况调整
    upper_yellow = np.array([30, 255, 255])  # 可根据实际情况调整
    # 根据阈值构建掩模
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    return mask

def find_average_position(mask):
    # 寻找所有黄色点的坐标
    points = np.column_stack(np.where(mask.transpose() > 0))
    if points.size == 0:
        return None  # 如果没有黄色点则返回None
    average_position = np.mean(points, axis=0)
    return average_position

def split_image_and_find_averages(image):
    # 分割图像为左右两部分
    height, width = image.shape[:2]
    mid_point = width // 2
    left_part = image[:, :mid_point]
    right_part = image[:, mid_point:]

    # 分别对左右两部分进行黄色过滤
    left_mask = filter_yellow_color(left_part)
    right_mask = filter_yellow_color(right_part)


    # 计算左右两部的黄色点的平均位置
    left_avg = find_average_position(left_mask)
    right_avg = find_average_position(right_mask)

    # 如果右侧有均值，调整其坐标以反映在整个图像中的位置
    if right_avg is not None:
        right_avg[0] += mid_point  # 仅调整x坐标

    return left_avg, right_avg, mid_point

# 主程序
#test
image = cv2.imread('test_middle.jpg')  # 加载图像
frame=np.array(image)

#print(frame)
def direction(frame):

    left_average, right_average,mid_point = split_image_and_find_averages(frame)

    if left_average[0]!=None and right_average[0]!=None:
        #print("左半部黄色点x平均位置:", left_average[0])
        #print("右半部黄色点x平均位置:", right_average[0])
        #print("中间的x轴位置：",mid_point)

        threshold_value = 300

        if(abs(left_average[0]-mid_point)-abs(right_average[0]-mid_point)>threshold_value):
            return "left"
        elif(abs(right_average[0]-mid_point)-abs(left_average[0]-mid_point)>threshold_value):
            return "right"
        else:
            return "no"

#test
result=direction(frame)
print(result)