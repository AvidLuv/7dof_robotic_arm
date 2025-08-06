import numpy as np
import pandas as pd
import csv
from pybullet_controller import RobotController
import pybullet as p

# 读取 forces.csv 文件中的所有外力行
force_csv_path = 'forces.csv'
forces = pd.read_csv(force_csv_path).values  # Nx3 array

# 创建机器人控制器
robot = RobotController(robot_type='7_dof', time_step=1/1000.)
robot.createWorld(GUI=True)

# 设置期望末端位姿
desired_pose = np.array([0.35, 0.35, 0.65, 0, 0, 0])

# 初始关节角度
th_initial = np.array([0, -1.0, 1.0, -1.57, -1.57, -1.57, 0])
base_pose = np.array([-0.71, 0.51, -0.22, 1.61, 6.31, -2.02, 0.9])
robot.setJointPosition(base_pose)

# 输出文件
output_csv = 'torque_results_initial.csv'

# 写入 CSV 文件头（不包含关节角）
with open(output_csv, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        "Fx", "Fy", "Fz",
        "tau1", "tau2", "tau3", "tau4", "tau5", "tau6", "tau7",
        "x", "y", "z", "roll", "pitch", "yaw"
    ])

    # 遍历每一组外力
    for idx, test_force in enumerate(forces):
        Fx, Fy, Fz = test_force.tolist()
        print(f"\n🔧 Force Sample {idx+1}/{len(forces)}:{test_force}")

        # 执行 impedance 控制器
        tau, q, pos, quat = robot.task_space_impedance_control(
            th_initial=th_initial,
            desired_pose=desired_pose,
            controller_gain=110,
            max_steps=15000,
            force_ext=[Fx, Fy, Fz]
        )

        # 转换四元数为欧拉角
        euler = p.getEulerFromQuaternion(quat)

        # 构造输出行
        row = [Fx, Fy, Fz] + list(tau.flatten()) + list(pos.flatten()) + list(euler)
        writer.writerow(row)

print(f"\n[✓] Results saved to: {output_csv}")