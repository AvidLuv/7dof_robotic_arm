import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np

# 读取 results.csv 文件
df = pd.read_csv('results.csv')

# 提取 Fx, Fy
Fx = df["Fx"]
Fy = df["Fy"]

# 提取 7 个 best_th（q1 ~ q7）
q_cols = [f"q{i+1}" for i in range(7)]

# 创建输出文件夹
output_dir = "qd_plots"
os.makedirs(output_dir, exist_ok=True)

# 依次绘制 7 个图
for i, q_col in enumerate(q_cols):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    Z = df[q_col]

    # 使用 plot_trisurf 并添加颜色映射
    trisurf = ax.plot_trisurf(Fx, Fy, Z,
                               cmap='viridis',
                               linewidth=0.2,
                               antialiased=True,
                               shade=True,
                               edgecolor='grey')

    # 加颜色条
    mappable = plt.cm.ScalarMappable(cmap='viridis')
    mappable.set_array(Z)
    fig.colorbar(mappable, ax=ax, shrink=0.6, aspect=10, pad=0.1, label=f'q{i+1}d value')

    ax.set_title(f"q{i+1}d vs Fx, Fy")
    ax.set_xlabel("Fx")
    ax.set_ylabel("Fy")
    ax.set_zlabel(f"q{i+1}d")

    plt.tight_layout()

    # 保存图像
    save_path = os.path.join(output_dir, f"q{i+1}d.png")
    plt.savefig(save_path)
    plt.close()

print("✅ 所有图已更新为彩色表面图并保存。")
