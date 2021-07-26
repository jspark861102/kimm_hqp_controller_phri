import mujoco_py
import numpy as np
import pinocchio as se3
import rospy
import kimm_hqp as hqp
import eigenpy

from os.path import dirname
# eigenpy.switchToNumpyArray()

# mujoco env
mjmodel = mujoco_py.load_model_from_path("/home/ggory15/catkin_ws/src/franka_panda_description/" +"/robots/panda_arm_single.xml")
sim = mujoco_py.MjSim(mjmodel)
viewer = mujoco_py.MjViewer(sim)
q = sim.data.qpos[:7]
v = sim.data.qvel[:7]

# ''.join(mjmodel.names)



# controller env
path = "/home/ggory15/catkin_ws/src/franka_panda_description/"
urdf = path + '/robots/panda_arm_l.urdf'
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = hqp.RobotWrapper(urdf, vector, False, False)

data = robot.data()

while True:
	viewer.render()
	robot.computeAllTerms(data, q, v)
	torque = robot.nonLinearEffect(data)
	
	for i in range(7):
		sim.data.ctrl[i] = torque[i]

	sim.step()
	

