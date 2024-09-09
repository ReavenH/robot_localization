import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 读取xlsx文件
file_path = 'F:\Cornell\Summer Session\Placing Brick Distribution Preprocessed.xlsx'
df = pd.read_excel(file_path)

# 从第二行开始获取前6列的数据，并转换为numpy数组
data_array = df.iloc[4:, :6].to_numpy()
print("Data: \n{} \nShape: {}".format(data_array, data_array.shape))

frontData = data_array[:, 1]
endData = data_array[:, 2]
overlapData = data_array[:, 3]
dataSize = data_array.shape[0]

# 将数据转换为 pandas DataFrame
df = pd.DataFrame({
    'Lateral and Longitudinal Offset': ['Front Lateral Offset'] * dataSize + ['Back Lateral Offset'] * dataSize + ['Longitudinal Offset'] * dataSize, 
    'Offset (CM)': np.concatenate([frontData, endData, overlapData])
})

# 绘制箱线图
plt.figure(figsize=(8, 6))
ax = sns.boxplot(x='Lateral and Longitudinal Offset', y='Offset (CM)', data=df, palette='Set2')
# ax.set_xlabel('Lateral and Longitudinal Offset', fontsize=16)
# ax.set_ylabel('Offset (CM)', fontsize=16)
ax.tick_params(axis='both', which='major', labelsize=15)
ax.set_xticklabels(['FL', 'BL', 'LT'], fontsize=15)

# 计算和标注关键数据
def annotate_boxplot(ax, data, labels):
    for i, group_data in enumerate(data):
        quartiles = np.percentile(group_data, [25, 50, 75])
        q1, median, q3 = quartiles
        iqr = q3 - q1
        whisker_low = q1 - 1.5 * iqr
        whisker_high = q3 + 1.5 * iqr

        # 中位数标注
        # ax.text(i, median, f'Median: {median:.2f}', horizontalalignment='center', color='black', fontsize=17, weight='bold')
        ax.text(i, median, f'{median:.2f}', horizontalalignment='center', color='black', fontsize=17, weight='bold')
        # 四分位数标注
        # ax.text(i, q1 - 0.3, f'Q1: {q1:.2f}', horizontalalignment='center', color='black', fontsize=15)
        # ax.text(i, q3 + 0.2, f'Q3: {q3:.2f}', horizontalalignment='center', color='black', fontsize=15)
        ax.text(i, q1 - 0.3, f'{q1:.2f}', horizontalalignment='center', color='black', fontsize=15)
        ax.text(i, q3 + 0.2, f'{q3:.2f}', horizontalalignment='center', color='black', fontsize=15)
        # 须的范围标注
        # ax.text(i, whisker_low, f'Whisker Low: {whisker_low:.2f}', horizontalalignment='center', color='black', fontsize=14, verticalalignment='top')
        # ax.text(i, whisker_high, f'Whisker High: {whisker_high:.2f}', horizontalalignment='center', color='black', fontsize=14, verticalalignment='bottom')
        ax.text(i, whisker_low, f'{whisker_low:.2f}', horizontalalignment='center', color='black', fontsize=14, verticalalignment='top')
        ax.text(i, whisker_high, f'{whisker_high:.2f}', horizontalalignment='center', color='black', fontsize=14, verticalalignment='bottom')
# 获取数据
data = [df[df['Lateral and Longitudinal Offset'] == 'Front Lateral Offset']['Offset (CM)'],
        df[df['Lateral and Longitudinal Offset'] == 'Back Lateral Offset']['Offset (CM)'], 
        df[df['Lateral and Longitudinal Offset'] == 'Longitudinal Offset']['Offset (CM)']]

# 在图中添加标注
annotate_boxplot(ax, data, ['FL', 'BL', 'LT'])

# 添加标题
# plt.title('Brick Placing Lateral and Longitudinal Offset (CM)', fontsize=16)
# plt.gca().set_aspect('equal', adjustable='box')

# 显示图形
plt.show()
