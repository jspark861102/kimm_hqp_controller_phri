import numpy as np
import pinocchio as pin
import quadprog 
import sys, os

import gepetto.corbaserver
import subprocess, time
import copy

np.set_printoptions(precision=3, linewidth=500)

display = True
M_PI = np.pi
# import robot
filename = (str(os.path.dirname(os.path.abspath(__file__))))
path = filename + '/../../husky_description'
urdf = path + "/robots/husky_panda_free.urdf"
robot = pin.RobotWrapper.BuildFromURDF(urdf, [path, ])
data = robot.data
model = robot.model

if (display):
    # luanch viewer
    launched = subprocess.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
    if int(launched[1]) == 0:
        os.system('gepetto-gui &')
    time.sleep(1)

    viewer = pin.visualize.GepettoVisualizer
    viz = viewer(robot.model, robot.collision_model, robot.visual_model)
    viz.initViewer(loadModel=True)
    viz.displayCollisions(False)
    viz.displayVisuals(True)
    theta = 0. * M_PI / 180.0
    q = np.array([0, 0, theta, 0, 0, theta, theta, theta, theta, theta, theta, theta])
    viz.display(q) # 2 + 7?

    gui = viz.viewer.gui
    window_id = gui.getWindowID("python-pinocchio")
    
    # J = robot.computeJointJacobian(q, model.getJointId("panda_joint7"), pin.ReferenceFrame.WORLD)
    robot.forwardKinematics(q)
    id = model.getJointId("panda_joint7")
    H_ee_ref = copy.deepcopy(robot.placement(q, id))

    R = np.array(np.zeros((6,6)))
    R[:3, :3] = H_ee_ref.rotation
    R[3:6, 3:6] = R[:3, :3]

    # J = pin.getJointJacobian(model, data, model.getJointId("panda_joint7"), pin.ReferenceFrame.WORLD)
    J = np.dot (R, robot.jointJacobian(q, model.getJointId("panda_joint7")))
    # J = robot.getJointJacobian(q, model.getJointId("panda_joint7"), pin.ReferenceFrame.LOCAL)
    print (J[:, :])
    # print (robot.mass(q))
    # print( robot.jointJacobian(q, model.getJointId("panda_joint7"))[:, 9:])
# # Robot state
# q = robot.q0
# v = robot.v0
# na = robot.nq

# q[0] = M_PI/ 2.0
# q[1] = M_PI/ 6.0
# q[3] = -M_PI/2.0
# q[4] = -M_PI/2.0
# q[5] = M_PI/2.0
# q[6] = M_PI/4.0

# robot.forwardKinematics(q, v)

# id = model.getJointId("panda_joint7")
# H_ee_ref = copy.deepcopy(robot.placement(q, id))
# H_ee_ref.translation[0] += 0.2

# max_iter = 0
# dt = 0.01
# while (max_iter < 100):
#     R = np.array(np.zeros((6,6)))
#     R[:3, :3] = H_ee_ref.rotation
#     R[3:6, 3:6] = R[:3, :3]
#     kp = 400.0
#     kv = 40.0

#     H_ee = robot.placement(q, id)

#     J = np.dot(R, robot.computeJointJacobian(q, id) ) # world Jacobian 
#     M = robot.mass(q)
#     Lam = np.linalg.pinv( np.dot(np.dot (J,  np.linalg.pinv(M)) , J.transpose()))

#     f_star = np.dot(Lam[:3, :3], (kp * (H_ee_ref.translation - H_ee.translation) - kv * (np.dot(J, v))[0:3]))
#     f_star_6d = np.array([0, 0, 0, 0, 0, 0])
#     f_star_6d[:3] = f_star
    
#     dv = np.dot(np.linalg.pinv(M) , np.dot(J.transpose(), f_star_6d))
   
#     v += dt * dv
#     q = pin.integrate(model, q, dt * v)
    
#     max_iter += 1

#     if display:
#         time.sleep(0.1)
#         viz.display(q)