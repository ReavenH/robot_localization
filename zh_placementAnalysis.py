import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 读取xlsx文件
file_path = 'F:\Cornell\Summer Session\Placing Brick Distribution Preprocessed.xlsx'
df = pd.read_excel(file_path)

# 从第二行开始获取前6列的数据，并转换为numpy数组
data_array = df.iloc[4:, :6].to_numpy()
print("Data: \n{} \nShape: {}".format(data_array, data_array.shape))

# create a color table
num_lines = data_array.shape[0]
colors = plt.cm.viridis(np.linspace(0, 1, num_lines))

# draw.
for i in range(num_lines):
    plt.plot([-40, 0], [data_array[i, 2], data_array[i, 1]], color = colors[i])
# draw the reference brick.
plt.plot([0, 0], [10, -10], color = 'r', label = "Reference Brick")
plt.plot([0, 10], [10, 10], color = 'r')
plt.plot([0, 10], [-10, -10], color = 'r')
# draw the reference axis.
plt.axhline(y=0, color = 'r', linestyle='-', label = 'Ground Truth')
plt.axhline(y = 1, color = 'r', linestyle='-', label = 'Ground Truth')
plt.axhline(y = -1, color = 'r', linestyle='-', label = 'Ground Truth')
plt.grid()
plt.xlabel("The Brick Placing Direction (CM)")
plt.ylabel("Error of Brick Placement(CM)")
plt.title("Erorr of Placing and Aligning a Brick")
plt.axis('equal')
plt.show()


