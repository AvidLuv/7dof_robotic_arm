import numpy as np
from pybullet_controller import RobotController

robot = RobotController(robot_type='7_dof', time_step=1/1000.)
robot.createWorld(GUI=True)

# 设置初始关节角度
th_initial= np.array([-0.71, 0.51, -0.22, 1.61, 6.31, -2.02, 0.9])
robot.setJointPosition(th_initial)
desired_pose = np.array([0.35, 0.35, 0.65, 0.0, 0.0, 0.0])

robot.static_pose_opt(th_initial, desired_pose, controller_gain=110)