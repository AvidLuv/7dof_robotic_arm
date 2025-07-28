import numpy as np
import pybullet as p
import time
from pybullet_controller import RobotController
from pyswarm import pso

controller = RobotController(robot_type='7_dof', time_step=1e-3)
controller.createWorld(GUI=True)

#计算雅可比矩阵
def compute_jacobian_pybullet(q):
    joint_pos = list(q)

    joint_vel = [0.0] * len(q)
    joint_acc = [0.0] * len(q)
    local_pos = [0, 0, 0]  # 通常表示末端 link 的原点

    jac_t, jac_r= p.calculateJacobian(
        bodyUniqueId=controller.robot_id,
        linkIndex=controller.end_eff_index,
        localPosition=local_pos,
        objPositions=joint_pos,
        objVelocities=joint_vel,
        objAccelerations=joint_acc
    )
    # 合并为 6×n 雅可比矩阵
    J = np.vstack((jac_t, jac_r))  # shape: (6, 7)

    return np.array(J)

def cost_function(q):
    print("Evaluating cost for q:", np.round(q, 2))

    controller.setJointPosition(q)
    for _ in range(10):
        p.stepSimulation()

    # 计算雅可比矩阵
    J = compute_jacobian_pybullet(q)  # 6x7

    F_ext = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # τ = J^T * F_ext
    tau = J.T @ F_ext  # shape: (7,)

    tau_max = np.array([39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0])  # 每个关节的力矩上限

    # 成本函数：目标是让所有关节力矩尽量接近上限
    cost = np.sum((tau_max - np.abs(tau)) ** 2)
    return cost

# 设置外力
F_ext = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 外力向量

# 设置目标末端位置
desired_pose = np.array([0.3, 0.3, 0.6, 0, 0, 0])

total_joints_num = p.getNumJoints(controller.robot_id)

# 转换为目标位置 + 四元数姿态
target_position = desired_pose[:3]
target_orientation = p.getQuaternionFromEuler(desired_pose[3:])

# 使用 IK 求解出关节角
ik_q = p.calculateInverseKinematics(
    controller.robot_id,
    controller.end_eff_index,
    target_position,
    targetOrientation=target_orientation,
    maxNumIterations=100,
    residualThreshold=1e-5
)
controller.setJointPosition(ik_q)

# 打印当前末端位置
ee_pos, ee_ori_quat = controller.solve_ForwardPositionKinematics(ik_q)
ee_ori_euler = p.getEulerFromQuaternion(ee_ori_quat)
print("末端:", ee_pos, ee_ori_euler)
print("关节角:", ik_q)

# 可视化目标点
target_visual = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.02,
    rgbaColor=[1, 0, 0, 1])
p.createMultiBody(
    baseVisualShapeIndex=target_visual,
    basePosition=desired_pose[:3])

# 定义关节角范围
lb = [-np.pi] * 7
ub = [np.pi] * 7

# 运行 PSO 优化
best_q, best_cost = pso(
    cost_function,
    lb,
    ub,
    swarmsize=30,
    maxiter=5,
    debug=True,
)

# # 输出并设置结果
print("最优关节角:", best_q)
print("对应最小 cost:", best_cost)

controller.setJointPosition(best_q)

# 保持窗口显示
while True:

    # 实时显示末端位置
    pos, ori = controller.solve_ForwardPositionKinematics(ik_q)
    #print("实时末端位置:", pos)

    p.applyExternalForce(
        objectUniqueId=controller.robot_id,
        linkIndex=total_joints_num - 1,
        forceObj=F_ext[:3],
        posObj=ee_pos,
        flags=p.WORLD_FRAME
    )
    p.stepSimulation()
    time.sleep(controller.time_step)

