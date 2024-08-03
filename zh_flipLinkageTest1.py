import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 创建绘图对象
fig, ax = plt.subplots()

# 创建圆对象
circle1 = patches.Circle((0, 0), 0.12, edgecolor='blue', facecolor='none')
circle2 = patches.Circle((0.03, 0), 0.05, edgecolor='red', facecolor='none')

# 添加圆到绘图对象
ax.add_patch(circle1)
ax.add_patch(circle2)

# 设置坐标轴的范围
ax.set_xlim(-0.2, 0.2)
ax.set_ylim(-0.2, 0.2)
ax.set_aspect('equal', adjustable='box')

# 显示绘图
plt.show()
