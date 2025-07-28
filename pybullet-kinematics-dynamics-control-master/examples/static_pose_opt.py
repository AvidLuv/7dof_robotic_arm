import numpy as np
from pybullet_controller import RobotController

robot = RobotController(robot_type='7_dof', time_step=1/1000.)
robot.createWorld(GUI=True)

    # 设置初始关节角度为 0
th_initial= np.array([ 2.6, -1.4, 4.0, -1.6, -1.7, -1.9, 2.6])
#th_initial= np.array([0, -1.0, 1.0, -1.57, -1.57, -1.57, 0])
robot.setJointPosition(th_initial)
desired_pose = np.array([0.3, 0.3, 0.6, 0.0, 0.0, 0.0])

robot.static_pose_opt(th_initial, desired_pose, controller_gain=110)