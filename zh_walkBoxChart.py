import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 读取xlsx文件
file_path = 'F:\Cornell\Summer Session\Experiment\walk.csv'
df = pd.read_excel(file_path)

# 从第二行开始获取前7列的数据，并转换为numpy数组
data_array = df.iloc[1:, :7].to_numpy()
print("Data: \n{} \nShape: {}".format(data_array, data_array.shape))

avgOffset = data_array[:, 1]
maxOffset = data_array[:, 2]
minOffset = data_array[:, 3]
avgAngularOffset = data_array[:, 4]
maxAngularOffset = data_array[:, 5]
minAngularOffset = data_array[:, 6]
dataSize = data_array.shape[0]

# 将数据转换为 pandas DataFrame
df = pd.DataFrame({
    'Straight Walk Pose': ['Average Lateral Offset'] * dataSize + ['Max Lateral Offset'] * dataSize + ['Min Lateral Offset'] * dataSize + ['Average Yaw Offset'] * dataSize + ['Max Yaw Offset'] * dataSize + ['Min Yaw Offset'] * dataSize, 
    'Offset (Degrees / CM)': np.concatenate([avgOffset, maxOffset, minOffset, avgAngularOffset, maxAngularOffset, minAngularOffset])
})

# 创建两个子图
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# 绘制第一个子图（前三个数据）
sns.boxplot(x='Straight Walk Pose', y='Offset (Degrees / CM)', data=df[df['Straight Walk Pose'].isin(['Average Lateral Offset', 'Max Lateral Offset', 'Min Lateral Offset'])], palette='Set2', ax=axes[0])
axes[0].set_xlabel('Lateral Offset', fontsize=16)
axes[0].set_ylabel('Offset (CM)', fontsize=16)
axes[0].tick_params(axis='both', which='major', labelsize=15)
# axes[0].set_title('Lateral Offset', fontsize=16)
axes[0].set_xticklabels(['Average', 'Max', 'Min'], fontsize=15)
# 绘制第二个子图（后三个数据）
sns.boxplot(x='Straight Walk Pose', y='Offset (Degrees / CM)', data=df[df['Straight Walk Pose'].isin(['Average Yaw Offset', 'Max Yaw Offset', 'Min Yaw Offset'])], palette='Set2', ax=axes[1])
axes[1].set_xlabel('Yaw Offset', fontsize=16)
axes[1].set_ylabel('Offset (Degrees)', fontsize=16)
axes[1].tick_params(axis='both', which='major', labelsize=15)
# axes[1].set_title('Yaw Offset', fontsize=16)
axes[1].set_xticklabels(['Average', 'Max', 'Min'], fontsize=15)

# 计算和标注关键数据的函数
def annotate_boxplot(ax, data, labels):
    for i, group_data in enumerate(data):
        quartiles = np.percentile(group_data, [25, 50, 75])
        q1, median, q3 = quartiles
        iqr = q3 - q1
        whisker_low = q1 - 1.5 * iqr
        whisker_high = q3 + 1.5 * iqr

        # 中位数标注
        if i == 2:
            bias = -0.8
        else:
            bias = 0.8
        ax.text(i + bias, median, f'Median: {median:.2f}', horizontalalignment='center', color='black', fontsize=17, weight='bold')
        
        # 四分位数标注
        ax.text(i, q1 - 0.15, f'Q1: {q1:.2f}', horizontalalignment='center', color='black', fontsize=15)
        ax.text(i, q3 + 0.2, f'Q3: {q3:.2f}', horizontalalignment='center', color='black', fontsize=15)

        # 须的范围标注
        '''
        ax.text(i, whisker_low, f'Whisker Low: {whisker_low:.2f}', horizontalalignment='center', color='black', fontsize=14, verticalalignment='top')
        ax.text(i, whisker_high, f'Whisker High: {whisker_high:.2f}', horizontalalignment='center', color='black', fontsize=14, verticalalignment='bottom')
        '''

# 获取数据并在两个子图中添加标注
data_lateral = [df[df['Straight Walk Pose'] == 'Average Lateral Offset']['Offset (Degrees / CM)'],
                df[df['Straight Walk Pose'] == 'Max Lateral Offset']['Offset (Degrees / CM)'], 
                df[df['Straight Walk Pose'] == 'Min Lateral Offset']['Offset (Degrees / CM)']]

data_yaw = [df[df['Straight Walk Pose'] == 'Average Yaw Offset']['Offset (Degrees / CM)'],
            df[df['Straight Walk Pose'] == 'Max Yaw Offset']['Offset (Degrees / CM)'],
            df[df['Straight Walk Pose'] == 'Min Yaw Offset']['Offset (Degrees / CM)']]

annotate_boxplot(axes[0], data_lateral, ['Average', 'Max', 'Min'])
annotate_boxplot(axes[1], data_yaw, ['Average', 'Max', 'Min'])

# 显示图形
# plt.title("Straight Walk Reliability Test")
fig.suptitle("Straight Walk Reliability Test", fontsize = 17)
plt.tight_layout()
plt.show()
