import numpy as np
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.resetSimulation
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

plane_id = p.loadURDF("plane.urdf",[0,0,0], [0,0,0,1])
targ_id = p.loadURDF("franka_panda/panda.urdf", [0,0,0], [0,0,0,1],useFixedBase=True)
obj_of_focus = targ_id

joint_id = 4
jlower = p.getJointInfo(targ_id, joint_id)[8]
jupper = p.getJointInfo(targ_id, joint_id)[9]

for step in range(300):
  joint_two_targ = np.random.uniform(jlower, jupper)
  joint_four_targ = np.random.uniform(jlower, jupper)

  p.setJointMotorControlArray(targ_id, [2,4], p.POSITION_CONTROL, targetPositions = [joint_two_targ, joint_four_targ])
  focus_position, _ = p.getBasePositionAndOrientation(targ_id)
  p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition= focus_position)
  p.stepSimulation()
  time.sleep(0.1)

  