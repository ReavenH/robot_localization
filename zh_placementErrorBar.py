import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
# 设置样式和颜色调色板
sns.set_style("white")
sns.set_palette("husl")

# 读取xlsx文件
file_path = 'F:\Cornell\Summer Session\Placing Brick Distribution Preprocessed.xlsx'
df = pd.read_excel(file_path)

# 从第二行开始获取前6列的数据，并转换为numpy数组
data_array = df.iloc[4:, :6].to_numpy()
print("Data: \n{} \nShape: {}".format(data_array, data_array.shape))

# create a error bar graph.
timeline = range(data_array.shape[0])
frontMean = np.mean(data_array[:, 1])
frontSD = np.std(data_array[:, 1])
backMean = np.mean(data_array[:, 2])
backSD = np.std(data_array[:, 2])
plt.scatter(0, frontMean, marker='o')
plt.errorbar(0, frontMean, yerr=frontSD, capsize=5, label="Front End Error")
plt.scatter(1, backMean, marker='o')
plt.errorbar(1, backMean, yerr=backSD, capsize=5, label="Back End Error")
plt.xlabel("Trials")
plt.ylabel("Placement Error Standard Deviation (CM)")
plt.legend()
plt.grid()
plt.show()


