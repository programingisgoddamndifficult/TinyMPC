import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation

# 读取轨迹数据
data = np.loadtxt("/home/user/TinyMPC/build/mpc_solution_data.txt", delimiter=",", skiprows=1)

# 提取 x, y, z 位置和时间数据
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]
time = data[:, 3]

# 创建一个3D图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 设置图形的标签和标题
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('MPC Trajectory')

# 初始化绘图对象
line, = ax.plot([], [], [], label='Trajectory', color='b')

# 设置轴范围
ax.set_xlim(min(x), max(x))
ax.set_ylim(min(y), max(y))
ax.set_zlim(min(z), max(z))

# 初始化函数
def init():
    line.set_data([], [])
    line.set_3d_properties([])  # 设置 3D 坐标的 Z 属性
    return line,

# 更新函数，逐帧更新位置
def update(frame):
    line.set_data(x[:frame], y[:frame])  # 更新 x 和 y 数据
    line.set_3d_properties(z[:frame])    # 更新 z 数据
    return line,

# 创建动画
ani = FuncAnimation(fig, update, frames=len(x), init_func=init, blit=True, interval=100)

# 显示动画
plt.show()
