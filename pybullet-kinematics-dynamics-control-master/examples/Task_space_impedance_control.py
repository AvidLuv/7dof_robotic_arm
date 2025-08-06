import numpy as np
import pandas as pd
from pybullet_controller import RobotController

# 加载外部力数据
force_df = pd.read_csv("forces.csv")  # 文件应有列名 x, y, z

# 创建机器人控制器
robot = RobotController(robot_type='7_dof', time_step=1/1000.)
robot.createWorld(GUI=False)  # 关闭 GUI 提高速度

# 设置期望末端位姿（保持不变）
desired_pose = np.array([0.35, 0.35, 0.65, 0, 0, 0])

# 初始关节角度（建议固定）
th_initial = np.array([0, -1.0, 1.0, -1.57, -1.57, -1.57, 0])
robot.setJointPosition(th_initial)

# 结果存储容器
results = []

# 遍历每一组外力
for idx, row in force_df.iterrows():
    Fx, Fy, Fz = row['x'], row['y'], row['z']
    force_vec = [Fx, Fy, Fz]
    print(f"\n>>> Testing external force: {force_vec}")

    tau, q, pos, quat = robot.task_space_impedance_control(
        th_initial=th_initial,
        desired_pose=desired_pose,
        controller_gain=110,
        force_ext=force_vec
    )

    # 存储 Fx, Fy, Fz, tau_1 ~ tau_7
    result_row = [Fx, Fy, Fz] + list(tau.flatten())
    results.append(result_row)

# 保存结果为 CSV
columns = ['Fx', 'Fy', 'Fz'] + [f'tau_{i+1}' for i in range(7)]
df_result = pd.DataFrame(results, columns=columns)
df_result.to_csv("torque_results_from_force_csv.csv", index=False)
print("\n[✓] All torque results saved to 'torque_results_from_force_csv.csv'")