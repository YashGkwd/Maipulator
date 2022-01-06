import time

import numpy as np
import pybullet
import pybullet as pb
import pybullet_data

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
robot = pb.loadURDF("C:\\Users\\91932\\Downloads\\pybullet-ur5-equipped-with-robotiq-140-20220104T131804Z-001\\pybullet-ur5-equipped-with-robotiq-140\\urdf\\ur5_robotiq_140.urdf")
table = pb.loadURDF("C:\\Users\\91932\\Downloads\pybullet-ur5-equipped-with-robotiq-140-20220104T131804Z-001\\pybullet-ur5-equipped-with-robotiq-140\\urdf\\objects\\table.urdf")
block = pb.loadURDF("C:\\Users\\91932\\Downloads\pybullet-ur5-equipped-with-robotiq-140-20220104T131804Z-001\\pybullet-ur5-equipped-with-robotiq-140\\urdf\\objects\\block.urdf")
pb.changeVisualShape(1,0,rgbaColor=(1,1,1,1))
pb.changeVisualShape(1,1,rgbaColor=(0,0,0,1))
pb.changeVisualShape(1,2,rgbaColor=(1,1,1,1))
pb.changeVisualShape(1,3,rgbaColor=(0,0,0,1))
pb.changeVisualShape(1,4,rgbaColor=(1,1,1,1))
pb.changeVisualShape(1,5,rgbaColor=(0,0,0,1))
pb.changeVisualShape(1,6,rgbaColor=(1,1,1,1))
pb.changeVisualShape(1,7,rgbaColor=(1,1,1,1))
pb.changeVisualShape(1,8,rgbaColor=(1,0,0,1))
#gripper
# pb.changeVisualShape(1,9,rgbaColor=(0,1,0,1))
# pb.changeVisualShape(1,10,rgbaColor=(1,0,0,1))
# pb.changeVisualShape(1,11,rgbaColor=(1,0,0,1))
# pb.changeVisualShape(1,12,rgbaColor=(1,1,1,1))
pb.changeVisualShape(1,15,rgbaColor=(1,0,0,1))#grip
# pb.changeVisualShape(1,14,rgbaColor=(1,0,0,1))
pb.changeVisualShape(1,16,rgbaColor=(1,0,0,1)) #grip
# pb.changeVisualShape(1,16,rgbaColor=(0,0,0,1))

def fu(a,b,n):
    if n==1:
      return a-b
    else:
        return a+b

fu = np.vectorize(fu)
sp = [0.5,0.2,0.8]
p1 = [0.48,-0.38,0.65]
p2 = [0.60,0.40,0.65]
px = [-0.1,0,0]
py = [0,-0.1,0]
tg_pick = [0.48,-0.38,1.1]
tg_place = [p2[0],p2[1],1.1]
pb.addUserDebugLine(fu(p1,px,0),fu(p1,px,1),[1,0,0],6,0)
pb.addUserDebugLine(fu(p1,py,0),fu(p1,py,1),[0,1,0],6,0)
pb.addUserDebugLine(fu(p2,px,0),fu(p2,px,1),[1,0,0],6,0)
pb.addUserDebugLine(fu(p2,py,0),fu(p2,py,1),[0,1,0],6,0)
#  pb.addUserDebugLine(tg_pick,[0.63+0.01,-0.27+0.01,0.8+0.01],[0,1,0],8,0)
pb.setGravity(0,0,-10)
pb.setRealTimeSimulation(1)
pb.createConstraint(table,-1,-1,-1,pb.JOINT_FIXED,[0,0,0],[0,0,0],[0.7,0,1.35])
bc = pb.createConstraint(block,-1,-1,-1,pb.JOINT_FIXED,[0,0,0],[0,0,0],p1,parentFrameOrientation=[np.cos(np.pi/2),np.sin(np.pi/2),np.sin(np.pi/2),0])
time.sleep(1)
pb.removeConstraint(bc)
time.sleep(1)


vec = []
traj=[]
for i in range(3):
   print(i)
   vec.append(tg_pick[i] - sp[i])
for i in range(100):
    t=[]
    for s in range(3):
        t.append(sp[s] + vec[s]*i/100)
    traj.append(t)

for i in range(len(traj)):
    pos = pb.calculateInverseKinematics(robot,8,traj[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    pb.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,pb.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)


# while True:
pos = pb.calculateInverseKinematics(robot,8,tg_pick,[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

print(len(pos))
for i in range(len(pos)):
    pb.setJointMotorControl2(robot,i+1,pb.POSITION_CONTROL,targetPosition = pos[i],force = 500)
time.sleep(1)

tg_pick_line = []
for i in range(1,17):
    tg_pick_line.append([0.48,-0.38,1.1 - 0.02*i])


for i in range(len(tg_pick_line)):
    pos = pb.calculateInverseKinematics(robot,8,tg_pick_line[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    pb.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,pb.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)

# pb.setJointMotorControl2(robot,9,pb.POSITION_CONTROL,targetPosition=np.pi)
pb.setJointMotorControlArray(robot,[9,10,11,12,13,14,15,16],pb.POSITION_CONTROL,targetPositions=[1,0,-np.pi/2,np.pi/2,0,0,1,1],forces=[8,8,8,8,8,8,30,30])
#pb.setJointMotorControl2(robot,14,pb.POSITION_CONTROL,targetPosition=-5,force=30)
time.sleep(0.5)
tg_pick_line_up = []
print(tg_pick_line[-1])
for i in range(1,11):
    tg_pick_line_up.append([p1[0],p1[1],tg_pick_line[-1][2] + 0.03*i])
for i in range(len(tg_pick_line_up)):
    pos = pb.calculateInverseKinematics(robot,8,tg_pick_line_up[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    pb.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,pb.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)



tg_pick2 = [0.63,-0.27,tg_pick_line_up[-1][2]]
tg_place =[p2[0],p2[1],1.1]
vec = []
traj=[]
for i in range(3):
   print(i)
   vec.append(tg_place[i] - tg_pick2[i])
for i in range(100):
    t=[]
    for s in range(3):
        t.append(tg_pick2[s] + vec[s]*i/100)
    traj.append(t)

for i in range(len(traj)):
    pos = pb.calculateInverseKinematics(robot,8,traj[i],[np.cos(np.pi/4),-np.sin(np.pi/4),0,0])

    pb.setJointMotorControlArray(robot, [1,2,3,4,5,6,7,8,9,10,11,12] ,pb.POSITION_CONTROL, targetPositions=pos, forces=[500,500,500,500,500,500,500,500,500,500,500,500])
    time.sleep(0.1)


pb.setJointMotorControlArray(robot,[9,10,11,12,13,14,15,16],pb.POSITION_CONTROL,targetPositions=[0,0,0,0,0,0,-np.pi/2,-np.pi/2],forces=[100,100,100,100,100,100,100,100])

time.sleep(100)
pb.disconnect(physicsClient)
