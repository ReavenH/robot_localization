import numpy as np
import matplotlib.pyplot as plt

# 从 TXT 文件中读取数据到 NumPy 数组
# 假设每行代表一个数据值
data = np.loadtxt('zh_flagHistory.txt')

# 提取 x 和 y 坐标
x = np.arange(len(data))  # x 坐标是数据的索引
y = data  # y 坐标是读取的数据

# 使用 Matplotlib 画图
plt.plot(x, y, marker='o')

# 添加标题和标签
plt.title('Positive Crossing Detection Illustration')
plt.xlabel('Frame Count')
plt.ylabel('1(Detected)')

# 显示网格
plt.grid(True)

# 显示图形
plt.show()
