import cv2
import numpy as np
import math
# change to the frame
image = cv2.imread('test_left_red.jpg')
# code start from here
class LineTracking:
    def __init__(self):
        self.line_centerx = -1
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.draw_color = self.range_rgb["red"]
        self.roi = [
            (0, 160, 0, 640, 0.1),
            (160, 320, 0, 640, 0.2),
            (320, 480, 0, 640, 0.7)
        ]
        self.roi_h_list = [
            self.roi[0][0],
            self.roi[1][0] - self.roi[0][0],
            self.roi[2][0] - self.roi[1][0]
        ]
        self.color_range_list = {
            'red': {
                'min': (0, 150, 150),
                'max': (255, 255, 255)
            }
        }
        self.target_color = ['red']
    def getAreaMaxContour(self,contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max  # 返回最大的轮廓

    def map_value(self,value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def run(self,img):
        global draw_color, line_centerx
        size = (640,480)
        # size = (640, 480)
        # img_copy = img.copy()
        img_h, img_w = img.shape[:2]


        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        centroid_x_sum = 0
        weight_sum = 0
        center_ = []
        n = 0
        # 将图像分割成上中下三个部分，这样处理速度会更快，更精确
        for r in self.roi:
            roi_h = self.roi_h_list[n]
            n += 1
            if n <= 2:
                continue
            blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
            frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

            for i in self.target_color:
                if i in self.color_range_list:
                    detect_color = i

                    frame_mask = cv2.inRange(frame_lab,
                                             (self.color_range_list[detect_color]['min'][0],
                                              self.color_range_list[detect_color]['min'][1],
                                              self.color_range_list[detect_color]['min'][2]),
                                             (self.color_range_list[detect_color]['max'][0],
                                              self.color_range_list[detect_color]['max'][1],
                                              self.color_range_list[detect_color]['max'][2]))  # 对原图像和掩模进行位运算
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
            # closed[:, 0:160] = 0
            # closed[:, 480:640] = 0
            if self.target_color == '' or self.target_color == 'None' or self.target_color == None:
                line_centerx = -1
                return img
            cnts = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找出所有轮廓
            cnt_large, area = self.getAreaMaxContour(cnts)  # 找到最大面积的轮廓
            if cnt_large is not None:  # 如果轮廓不为空
                rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形
                box = np.intp(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                for i in range(4):
                    box[i, 1] = box[i, 1] + (n - 1) * roi_h + self.roi[0][0]
                    box[i, 1] = int(self.map_value(box[i, 1], 0, size[1], 0, img_h))
                for i in range(4):
                    box[i, 0] = int(self.map_value(box[i, 0], 0, size[0], 0, img_w))

                cv2.drawContours(img, [box], -1, (0, 255, 0, 255), 2)  # 画出四个点组成的矩形

                # 获取矩形的对角点
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  # 中心点
                cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)  # 画出中心点

                center_.append([center_x, center_y])
                # 按权重不同对上中下三个中心点进行求和
                centroid_x_sum += center_x * r[4]
                weight_sum += r[4]

        if weight_sum != 0:
            # 求最终得到的中心点
            line_centerx = int(centroid_x_sum / weight_sum)
            cv2.circle(img, (line_centerx, int(center_y)), 10, (0, 0, 0), -1)  # 画出中心点
            #print('line_centerx', line_centerx)
        else:
            line_centerx = -1

        img_middle_x = img_w / 2

        # return the approximate distance to the middle of the image
        result=0
        if(line_centerx-img_middle_x<50 and line_centerx-img_middle_x>=-50):
            result=0
        elif(line_centerx-img_middle_x<100 and line_centerx-img_middle_x>=50):
            result=1
        elif(line_centerx-img_middle_x<150 and line_centerx-img_middle_x>=100):
            result=2
        elif (line_centerx - img_middle_x >= 150):
            result=3
        elif (line_centerx - img_middle_x < -50 and line_centerx - img_middle_x >= -100):
            result = -1
        elif (line_centerx - img_middle_x < -100 and line_centerx - img_middle_x >= -150):
            result = -2
        elif (line_centerx - img_middle_x < -150):
            result=-3
        return result

# the code ends here

# test the result
lt=LineTracking()
result=lt.run(image)
print(result)

'''
# 检查图像是否成功加载
if result is None:
    print("图像加载失败")
else:
    # 设置新的尺寸，例如将宽和高都缩小为原来的一半
    scale_percent = 30  # 百分比，这里设置为50%，意味着新尺寸为原尺寸的一半
    width = int(result.shape[1] * scale_percent / 100)
    height = int(result.shape[0] * scale_percent / 100)
    dim = (width, height)

    # 缩小图像
    resized = cv2.resize(result, dim, interpolation = cv2.INTER_AREA)

    # 显示缩小后的图像
    cv2.imshow('After Closing', resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

img_h, img_w = image.shape[:2]
img_middle_x=img_w/2
print('Center of the image in x directions is: ', img_middle_x)

print('The bias is: ', line_centerx-img_middle_x)
'''