import math as mt
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits import mplot3d
import numpy as np
import sympy as s


t1=s.Symbol("t1")
t2=s.Symbol("t2")
t3=s.Symbol("t3")
x1=s.Symbol("h0")
x2=s.Symbol("h1")
x3=s.Symbol("h2")
T01 =s.Matrix([[s.cos(t1),-s.sin(t1),0,0],[s.sin(t1),s.cos(t1),0,0],[0,0,1,x1],[0,0,0,1]])
T12 =s.Matrix([[s.cos(t2),-s.sin(t2),0,0],[0,0,1,0],[-s.sin(t2),-s.cos(t2),0,0],[0,0,0,1]])
T23 =s.Matrix([[s.cos(t3),-s.sin(t3),0,x2],[0,0,-1,-x2],[s.sin(t3),s.cos(t3),0,0],[0,0,0,1]])
T34=s.Matrix([[1,0,0,x3],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

T02=T01*T12
T03=T02*T23
T04=T03*T34

x=s.Symbol("x")
y=s.Symbol("y")
z=s.Symbol("z")
fx=T04[0,-1]-x
fy=T04[1,-1]-y
fz=T04[2,-1]-z
F=s.Matrix([fx,fy,fz])
F=F.subs(x3,1)
F=F.subs(x2,1)
F=F.subs(x1,1)

#jacobian
J = F.jacobian(s.Matrix([t1, t2, t3]))


xt=[3]
yt=[-0.5]
zt=[2.5]
angles=[]
for i in range(1,100):
    xt.append((100-i*(3)+i*(-2))/100)
    yt.append((100 - i * (-0.5) + i * (-3.5)) / 100)
    zt.append((100 - i * (2.5) + i * (2.5)) / 100)
xt.append(-2)
yt.append(-3.5)
zt.append(2.5)

for i in range(len(xt)):
    fun = F.subs(x, xt[i]).subs(z, zt[i]).subs(y, yt[i])
    approx = 0.001
    i_g = np.array([[np.pi/2], [0],[np.pi/2]])

    while True:
        fn = fun.subs(t1, i_g[0].squeeze()).subs(t2, i_g[1].squeeze()).subs(t3, i_g[2].squeeze())
        #print(fn)
        fn=np.array(fn.tolist()).astype(np.float64)

        jac=J.subs(t1, i_g[0].squeeze()).subs(t2, i_g[1].squeeze()).subs(t3, i_g[2].squeeze())
        jac = np.array(jac.tolist()).astype(np.float64)
        jac_inv=np.linalg.inv(jac)
        dx =np.dot(jac_inv, (fn))
        #print(dx)
        i_g = i_g - dx

        a = fun.subs(t1, i_g[0].squeeze()).subs(t2, i_g[1].squeeze()).subs(t3, i_g[2].squeeze())

        a = np.array(a.tolist()).astype(np.float64)
        if (abs(a[0]) < approx) and (abs(a[1]) < approx) and ( abs(a[2]) < approx):
            break
    theta= i_g
    angles.append(theta)

x_data =[]
y_data =[]
z_data =[]
for i in range(len(angles)):
    t1=angles[i][0]
    t2 = angles[i][1]
    t3 = angles[i][2]
    x_data.append([0,mt.sin(t2)*mt.cos(t1)+mt.cos(t1)*mt.cos(t2),mt.sin(t2)*mt.cos(t1)+mt.cos(t1)*mt.cos(t2)-mt.sin(t1)*mt.sin(t3)+mt.cos(t1)*mt.cos(t2)*mt.cos(t3)])
    y_data.append([0,mt.sin(t1)*mt.sin(t2)+ mt.sin(t1)*mt.cos(t2),mt.sin(t1)*mt.sin(t2)+ mt.sin(t1)*mt.cos(t2)+mt.sin(t1),mt.sin(t1)*mt.sin(t2)+ mt.sin(t1)*mt.cos(t2)+mt.sin(t3)*mt.cos(t1)+ mt.sin(t1)*mt.cos(t2)*mt.cos(t3)])
    z_data.append([1,1-mt.sin(t2)+mt.cos(t2),1-mt.sin(t2)*mt.cos(t2)-mt.sin(t2)*mt.cos(t3)])
