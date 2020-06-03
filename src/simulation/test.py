import time

import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("src/model/spot.urdf", cubeStartPos, cubeStartOrientation)
for _ in range(300):
    pos, ori = p.getBasePositionAndOrientation(__A__)
    p.applyExternalForce(__B__, 0, [50, 0, 0], __C__, p.WORLD_FRAME)
    p.stepSimulation()
p.disconnect()
