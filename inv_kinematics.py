import pybullet as pb
import time
import pybullet_data
import pylab as p
import sympy as s

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.loadURDF("plane.urdf")
robot = pb.loadURDF("C:\\Users\\Lenovo\\Downloads\\pybullet-force-control-main\\pybullet-force-control-main\\urdf\\ur5.urdf")
print(robot)

pb.changeVisualShape(1,0,rgbaColor=(1,1,1,1))
pb.changeVisualShape(1,1,rgbaColor=(1,0,0,1))
pb.changeVisualShape(1,2,rgbaColor=(0,1,0,1))
pb.changeVisualShape(1,3,rgbaColor=(0,0,1,1))
pb.changeVisualShape(1,4,rgbaColor=(0,1,1,1))
pb.changeVisualShape(1,5,rgbaColor=(0,0,0,1))
pb.changeVisualShape(1,6,rgbaColor=(1,0,0,1))
pb.createConstraint(robot,-1,-1,0,pb.JOINT_FIXED,[0, 0, 0], [0, 0, 0],[0,0,0])

l=0
pb.setRealTimeSimulation(1)
time.sleep(5)


p1 = [0.7,-0.4,1-0.13]
p2 = [0.6,0.4,1.75-0.21]
p3 = [-0.35,1,1.35-0.3]
pb.addUserDebugLine(p1,p2,[1,0,0],4,0)
pb.addUserDebugLine(p2,p3,[1,0,0],4,0)
pnts = [p1,p2,p3]
from sympy import binomial
t = s.Symbol("t")
n = s.Symbol("n")
i = s.Symbol("i")
f = binomial(n,i)*t**i*(1-t)**(n-i)
# x= 2*s.Sum(f,(i,0,n))
fx=0
for l in range(len(pnts)):
   fun = pnts[l][0]*f.subs({i:l,n:len(pnts)-1})
   fx += fun

fy=0
for l in range(len(pnts)):
   fun = pnts[l][1]*f.subs({i:l,n:len(pnts)-1})
   fy += fun

fz=0
for l in range(len(pnts)):
   fun = pnts[l][2]*f.subs({i:l,n:len(pnts)-1})
   fz += fun

x= []
y= []
z= []
for s in range(101):
    ts= 0.01*s
    x.append(fx.subs(t,ts))
    y.append(fy.subs(t, ts))
    z.append(fz.subs(t, ts))
print(len(x))
print(x[0],y[0],z[0])
for q in range(101):

     post = pb.calculateInverseKinematics(robot,7,[x[q],y[q],z[q]])
     pb.addUserDebugLine([x[q],y[q],z[q]], [x[q]+0.01,y[q]+0.01,z[q]+0.01], [0, 1, 0], 8, 0)
     #print(len(post))
     # print(pb.getNumJoints(robot))
     for i in range(6):
         pb.setJointMotorControl2(robot,i+1,pb.POSITION_CONTROL,targetPosition=post[i],force=200,physicsClientId=physicsClient)
     time.sleep(0.2)
time.sleep(2)
pb.disconnect(physicsClient)
