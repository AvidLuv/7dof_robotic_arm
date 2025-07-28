import numpy as np
from pyswarm import pso
import matplotlib.pyplot as plt

# -------------------------------
# 模拟部分：构造雅可比矩阵和期望末端力
# -------------------------------

def generate_jacobian():
    # 构造一个6x7的随机雅可比矩阵（可替换为真实的J）
    np.random.seed(42)
    return np.random.randn(6, 7)

def get_end_effector_force():
    # 模拟一个固定末端力，例如朝某个方向推
    return np.array([0, 0, 0, 0, 0, 0])  # Fx, Fy, Fz, τx, τy, τz

J = generate_jacobian()
F = get_end_effector_force()
tau_max = np.array([40, 40, 40, 30, 20, 10, 10])  # 每个关节的最大允许力矩

# -------------------------------
# 目标函数：使 τ 不接近饱和，代价函数越小越好
# -------------------------------

def cost_function(q_dummy):
    """
    模拟代价函数，只考虑 τ = Jᵀ * F（与q无关，只为测试PSO）
    """
    tau = np.dot(J.T, F)  # 计算当前任务力矩
    cost = np.sum((tau_max - np.abs(tau))**2)  # 离饱和越远，代价越小
    return cost

# -------------------------------
# PSO参数设置并运行优化
# -------------------------------

lower_bounds = -np.pi * np.ones(7)   # 关节下限
upper_bounds = np.pi * np.ones(7)    # 关节上限

best_q, best_cost = pso(cost_function, lower_bounds, upper_bounds, swarmsize=50, maxiter=100)

# -------------------------------
# 输出结果
# -------------------------------

print("最佳关节配置 q:", best_q)
print("最小代价函数值:", best_cost)

# 计算并打印最终的关节力矩
final_tau = np.dot(J.T, F)
print("最终力矩 τ:", final_tau)
print("与力矩上限的差距:", tau_max - np.abs(final_tau))
