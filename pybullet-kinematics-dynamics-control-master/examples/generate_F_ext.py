import csv
import numpy as np

# 设置 x, y 的范围与步长
x_min, x_max, x_step = 0.0, 16.0, 2.0
y_min, y_max, y_step = 0.0, 16.0, 2.0

# 打开 CSV 文件写入
with open("forces.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["x", "y", "z"])  # 表头

    for x in np.arange(x_min, x_max + x_step, x_step):
        for y in np.arange(y_min, y_max + y_step, y_step):
            z = 0  # z 分量保持为 0
            writer.writerow([round(x, 3), round(y, 3), z])

print("forces.csv Gnerated successfully.")
