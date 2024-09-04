import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
import csv

file_path = 'F:\Cornell\Summer Session\Experiment\walk.csv'
file_path1 = 'F:\Cornell\Summer Session\Placing Brick Distribution Preprocessed.xlsx'
file_path2 = "F:\Cornell\Summer Session\Experiment\group7.xlsx"
df = pd.read_excel(file_path) # walk 20 times df
df1 = pd.read_excel(file_path1)  # placing 50 times df
df2 = pd.read_excel(file_path2)  # walk a single time df
 
data_array = df.iloc[1:, :7].to_numpy()
data_array1 = df1.iloc[4:, :6].to_numpy()
data_array2 = df2.iloc[:, 1:].to_numpy()
print("data_array shapes: [{}], [{}], [{}]".format(data_array.shape, data_array1.shape, data_array2.shape))

avgOffset = data_array[:, 1]
maxOffset = data_array[:, 2]
minOffset = data_array[:, 3]
avgAngularOffset = data_array[:, 4]
maxAngularOffset = data_array[:, 5]
minAngularOffset = data_array[:, 6]
dataSize = data_array.shape[0]

frontData = data_array1[:, 1]
endData = data_array1[:, 2]
overlapData = data_array1[:, 3]
dataSize1 = data_array1.shape[0]

latOffset = data_array2[0, :].T
yawOffset = data_array2[1, :].T
dataSize2 = data_array2.shape[1]

# 将数据转换为 pandas DataFrame，合并 Lateral 和 Yaw 数据
df = pd.DataFrame({
    'Pose Type': ['Lateral Offset'] * (3 * dataSize) + ['Yaw Offset'] * (3 * dataSize),
    'Offset Category': ['Average'] * dataSize + ['Max'] * dataSize + ['Min'] * dataSize + ['Average'] * dataSize + ['Max'] * dataSize + ['Min'] * dataSize,
    'Offset (Degrees / CM)': np.concatenate([avgOffset, maxOffset, minOffset, avgAngularOffset, maxAngularOffset, minAngularOffset])
})

df1 = pd.DataFrame({
    'Lateral and Longitudinal Offset': ['Front Lateral Offset'] * dataSize1 + ['Back Lateral Offset'] * dataSize1 + ['Longitudinal Offset'] * dataSize1, 
    'Offset (CM)': np.concatenate([frontData, endData, overlapData])
})

df2 = pd.DataFrame({
    'Pose Type': ['Lateral Offset'] * dataSize2 + ['Yaw Offset'] * dataSize2,
    'Offset Category': ['Lateral'] * dataSize2 + ['Yaw'] * dataSize2,
    'Offset': np.concatenate([latOffset, yawOffset])
})

fig, axes = plt.subplots(1, 5, figsize=(28, 6))
gs = gridspec.GridSpec(1, 5, width_ratios=[1, 1, 0.5, 0.5, 1])
axes[0] = plt.subplot(gs[0])
axes[1] = plt.subplot(gs[1])
axes[2] = plt.subplot(gs[2])
axes[3] = plt.subplot(gs[3])
axes[4] = plt.subplot(gs[4])

# 绘制第一个子图（Lateral Offset数据）
sns.boxplot(x='Offset Category', y='Offset (Degrees / CM)', data=df[df['Pose Type'] == 'Lateral Offset'], palette='Set2', ax=axes[0])
axes[0].set_xlabel('A', fontsize=14)
axes[0].set_ylabel('Offset (CM)', fontsize=14)
# axes[0].tick_params(axis='both', which='major', labelsize=15)
# axes[0].set_xticklabels(['Average', 'Max', 'Min'], fontsize=12)

# 绘制第二个子图（Yaw Offset数据）
sns.boxplot(x='Offset Category', y='Offset (Degrees / CM)', data=df[df['Pose Type'] == 'Yaw Offset'], palette='Set2', ax=axes[1])
axes[1].set_xlabel('B', fontsize=14)
axes[1].set_ylabel('Offset (Degrees)', fontsize=14)
# axes[1].tick_params(axis='both', which='major', labelsize=15)
# axes[1].set_xticklabels(['Average', 'Max', 'Min'], fontsize=12)

sns.boxplot(x = 'Offset Category', y = 'Offset', data = df2[df2['Pose Type'] == 'Lateral Offset'], palette='Set3', ax=axes[2])
axes[2].set_xlabel('C', fontsize=14)
axes[2].set_ylabel('Offset (CM)', fontsize=14)
# axes[2].tick_params(axis='both', which='major', labelsize=15)

sns.boxplot(x = 'Offset Category', y = 'Offset', data = df2[df2['Pose Type'] == 'Yaw Offset'], palette='Set3', ax=axes[3])
axes[3].set_xlabel('D', fontsize=14)
axes[3].set_ylabel('Offset (Degrees)', fontsize=14)
# axes[3].tick_params(axis='both', which='major', labelsize=15)

sns.boxplot(x='Lateral and Longitudinal Offset', y='Offset (CM)', data=df1, palette='Set1', ax=axes[4])
axes[4].set_xlabel('E', fontsize=14)
axes[4].set_ylabel('Offset (CM)', fontsize=14)
# axes[4].tick_params(axis='both', which='major', labelsize=15)
axes[4].set_xticklabels(['FL', 'BL', 'LT'], fontsize=12)

# 计算和标注关键数据的函数
def annotate_boxplot(ax, data):
    for i, group_data in enumerate(data):
        quartiles = np.percentile(group_data, [25, 50, 75])
        q1, median, q3 = quartiles

        # 中位数标注
        ax.text(i, median, f'{median:.2f}', horizontalalignment='center', color='black', fontsize=14, weight='bold')

        # 四分位数标注
        # ax.text(i, q1 - 0.65, f'{q1:.2f}', horizontalalignment='center', color='black', fontsize=12)
        # ax.text(i, q3 + 0.55, f'{q3:.2f}', horizontalalignment='center', color='black', fontsize=12)

# 获取数据并在两个子图中添加标注

data_lateral = [df[(df['Pose Type'] == 'Lateral Offset') & (df['Offset Category'] == 'Average')]['Offset (Degrees / CM)'],
                df[(df['Pose Type'] == 'Lateral Offset') & (df['Offset Category'] == 'Max')]['Offset (Degrees / CM)'], 
                df[(df['Pose Type'] == 'Lateral Offset') & (df['Offset Category'] == 'Min')]['Offset (Degrees / CM)']]

data_yaw = [df[(df['Pose Type'] == 'Yaw Offset') & (df['Offset Category'] == 'Average')]['Offset (Degrees / CM)'],
            df[(df['Pose Type'] == 'Yaw Offset') & (df['Offset Category'] == 'Max')]['Offset (Degrees / CM)'],
            df[(df['Pose Type'] == 'Yaw Offset') & (df['Offset Category'] == 'Min')]['Offset (Degrees / CM)']]

data_lateral1 = [df2[(df2['Pose Type'] == 'Lateral Offset')]['Offset']]

data_yaw1 = [df2[(df2['Pose Type'] == 'Yaw Offset')]['Offset']]

data_placing = [df1[df1['Lateral and Longitudinal Offset'] == 'Front Lateral Offset']['Offset (CM)'],
        df1[df1['Lateral and Longitudinal Offset'] == 'Back Lateral Offset']['Offset (CM)'], 
        df1[df1['Lateral and Longitudinal Offset'] == 'Longitudinal Offset']['Offset (CM)']]


annotate_boxplot(axes[0], data_lateral)
annotate_boxplot(axes[1], data_yaw)
annotate_boxplot(axes[2], data_lateral1)
annotate_boxplot(axes[3], data_yaw1)
annotate_boxplot(axes[4], data_placing)


# 显示图形
# fig.suptitle("Straight Walk Reliability Test", fontsize=17)
fig.subplots_adjust()
plt.tight_layout()
plt.show()
