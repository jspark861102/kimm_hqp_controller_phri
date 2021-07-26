import mujoco_py
import numpy as np
import pinocchio as se3
import rospy
import kimm_hqp as hqp
import eigenpy

from os.path import dirname

# mujoco env
mjmodel = mujoco_py.load_model_from_path("/home/ggory15/catkin_ws/src/franka_panda_description/" +"/robots/panda_arm_single.xml")
sim = mujoco_py.MjSim(mjmodel)
viewer = mujoco_py.MjViewer(sim)
q = sim.data.qpos[:7]
v = sim.data.qvel[:7]

# controller env
path = "/home/ggory15/catkin_ws/src/franka_panda_description/"
urdf = path + '/robots/panda_arm_l.urdf'
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = hqp.RobotWrapper(urdf, vector, False, False)
model = robot.model
data = robot.data()
na = robot.na

# variable Resize
q = sim.data.qpos[:7]
v = sim.data.qvel[:7]

torque = np.array(np.zeros((na, 1)))
time = 0.0

# make controller
tsid = hqp.InverseDynamicsFormulationAccForce("tsid", robot, False)
tsid.computeProblemData(time, q, v)

# make task
postureTask = hqp.TaskJointPosture("task-posture", robot)
postureTask.setKp(5000.0 * np.array(np.ones((na, 1))))
postureTask.setKd(2.0 * np.sqrt(5000.0) * np.ones((na, 1)) )

torqueBoundTask = hqp.TaskJointBounds("task-torque-bounds", robot)
dq_max = 1000.0 * np.ones((na, 1))
dq_min = -dq_max
torqueBoundTask.setBounds(dq_min, dq_max)

eeTask = hqp.TaskSE3Equality("task-se3", robot, "panda_joint7")
eeTask.setKp(1000.0 * np.ones((na, 1)))
eeTask.setKd(2.0 * np.sqrt(eeTask.Kp))

# make traj
trajPosture = hqp.TrajectoryEuclidianConstant("traj_posture")
samplePosture = hqp.TrajectorySample(na)

trajEE = hqp.TrajectorySE3Constant("traj-se3")
sampleEE = hqp.TrajectorySample(12, 6)

# solver
solver = hqp.SolverHQuadProg("qp solver")
solver.resize(tsid.nVar, tsid.nEq, tsid.nIn)

tsid.addMotionTask(postureTask, 1e-2, 0, 0.)
tsid.addMotionTask(torqueBoundTask, 1.0, 0, 0)

while True:
    

    q_ref = np.zeros((na, 1))
    q_ref[3] = -1.57
    q_ref[5] = 3.14

    trajPosture.setReference(q_ref)
    samplePosture = trajPosture.computeNext()
    postureTask.setReference(samplePosture)

    HQPData = tsid.computeProblemData(time, q, v)    
    sol = solver.solve(HQPData)
    torque = tsid.getActuatorForces(sol)
    
    for i in range(7):
        sim.data.ctrl[i] = torque[i]
    
    sim.step()
    viewer.render()
    time = time + 0.001
