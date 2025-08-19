import numpy as np
import pandas as pd
import csv
from pybullet_controller import RobotController
import pybullet as p
import math

# 读取 forces.csv
force_csv_path = 'forces.csv'
forces = pd.read_csv(force_csv_path).values  # Nx3 array

# 机器人
robot = RobotController(robot_type='7_dof', time_step=1/1000.)
robot.createWorld(GUI=False)

# 期望末端位姿 & 初始关节
desired_pose = np.array([0.35, 0.35, 0.65, 0, 0, 0])
th_initial = np.array([0, -1.0, 1.0, -1.57, -1.57, -1.57, 0])
base_pose = np.array([-0.71, 0.51, -0.22, 1.61, 6.31, -2.02, 0.9])
robot.setJointPosition(base_pose)

# 输出文件（保持你之前的列顺序/名称）
output_csv = 'torque_results_initial.csv'

with open(output_csv, mode='w', newline='') as f:
    writer = csv.writer(f)
    # 写表头
    writer.writerow([
        "Fx","Fy","Fz","cost",
        "best_th1","best_th2","best_th3","best_th4","best_th5","best_th6","best_th7",
        "q1","q2","q3","q4","q5","q6","q7",
        "tau1","tau2","tau3","tau4","tau5","tau6","tau7",
        "x","y","z","roll","pitch","yaw"
    ])

    for idx, test_force in enumerate(forces):
        Fx, Fy, Fz = [float(v) for v in test_force.tolist()]
        print(f"\n🔧 Force Sample {idx+1}/{len(forces)}: {test_force}")

        # 运行控制器
        tau, q, pos, quat = robot.task_space_impedance_control(
            th_initial=th_initial,
            desired_pose=desired_pose,
            controller_gain=110,
            max_steps=30000,
            force_ext=[Fx, Fy, Fz]
        )

        # 四元数 -> 欧拉角 (roll,pitch,yaw)，单位：弧度
        roll, pitch, yaw = p.getEulerFromQuaternion(quat)

        # === 这里填 cost 和 best_th ===
        # 如果暂时没有 PSO 的 cost / best_th，先写 NaN 占位
        cost = float('nan')
        best_th = [float('nan')]*7
        # 如果你想先用 th_initial 充当 best_th（临时）：
        # best_th = th_initial.tolist()

        # 整理 q、tau、pos
        q = np.asarray(q).flatten().tolist()[:7]
        tau = np.asarray(tau).flatten().tolist()[:7]
        x, y, z = [float(pos[0]), float(pos[1]), float(pos[2])]

        row = (
            [Fx, Fy, Fz, cost] +
            best_th +                  # best_th1..7
            q +                        # q1..q7
            tau +                      # tau1..tau7
            [x, y, z, roll, pitch, yaw]
        )
        writer.writerow(row)

print(f"\n[✓] Results saved to: {output_csv}")