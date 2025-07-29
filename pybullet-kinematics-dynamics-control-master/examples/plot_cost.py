import os
import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV 文件
csv_file = 'cost_data.csv'
df = pd.read_csv(csv_file)

# 创建保存图像的目录
output_dir = 'gcost_plots'
os.makedirs(output_dir, exist_ok=True)

# 遍历每一行数据（每一组外力）
for idx, row in df.iterrows():
    fx, fy, fz = row["Fx"], row["Fy"], row["Fz"]
    costs = row[3:].values  # 从第4列开始为 cost 数据（iter_1 到 iter_N）

    # 生成图像
    plt.figure(figsize=(8, 5))
    x_values = list(range(1, len(costs) + 1))
    plt.plot(x_values, costs, marker='o')
    plt.title(f"gBest Cost Curve\nForce: Fx={fx}, Fy={fy}, Fz={fz}")
    plt.xlabel("Iteration")
    plt.ylabel("gBest Cost")
    plt.xticks(x_values)  # 设置 x 轴刻度为 1, 2, ..., max_iter
    plt.grid(True)
    plt.tight_layout()

    # 构造图像保存路径，注意避免非法字符
    filename = f"{fx}_{fy}_{fz}".replace(".", "p").replace("-", "n") + ".png"
    filepath = os.path.join(output_dir, filename)
    plt.savefig(filepath)
    plt.close()

print(f"✅ 所有图已保存至：{output_dir}")