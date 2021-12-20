def inv_kinematics(x,y,z,i,pi):
    fx = end_aff[0] - x
    fy = end_aff[1] - y
    fz = end_aff[2] - z
    fx_v=s.lambdify([t1,t2,t3],fx,"numpy")
    fy_v = s.lambdify([t1, t2, t3], fy, "numpy")
    fz_v = s.lambdify([t1, t2, t3], fz, "numpy")
    F = s.Matrix([fx,fy,fz])
    if i==0:
      ig = np.array([[0],[np.pi/3],[np.pi/2]])
    else:
        ig=pi
    J = s.simplify(F.jacobian([t1,t2,t3]))
    while abs(fx_v(ig[0][0],ig[1][0],ig[2][0]))>=0.001 or (abs(fy_v(ig[0][0],ig[1][0],ig[2][0]))>=0.001) or (abs(fz_v(ig[0][0],ig[1][0],ig[2][0]))>=0.001) :
         J_i = J.subs({t1:ig[0][0],t2:ig[1][0],t3:ig[2][0]})
         F_i = F.subs({t1: ig[0][0], t2: ig[1][0], t3: ig[2][0]})
         jac_inv=np.linalg.inv(np.array(J_i).astype(np.float64))
         F_ivalue=np.array(F_i).astype(np.float64)
         ig = ig - np.dot(jac_inv,F_ivalue)
    return ig

xt=[]
yt=[]
zt=[]
angles=[]
for i in range(50 ,100):
    xt.append(((100-i)*(3)+i*(0))/100)
    yt.append(((100 - i) * (1) + i * (1)) / 100)
    zt.append(((100 - i) * (0) + i * (1)) / 100)

x_cordi = s.Matrix([T02[0,-1],T03[0,-1],T04[0,-1]])
y_cordi = s.Matrix([T02[1,-1],T03[1,-1],T04[1,-1]])
z_cordi = s.Matrix([T02[2,-1],T03[2,-1],T04[2,-1]])
x_data = []
y_data = []
z_data = []

for i in range(len(xt)):
    print(i)

    theta = inv_kinematics(xt[i], yt[i], zt[i],i, theta)
    x_p = (x_cordi.subs({t1: theta[0][0], t2: theta[1][0], t3: theta[2][0]}))
    y_p = (y_cordi.subs({t1: theta[0][0], t2: theta[1][0], t3: theta[2][0]}))
    z_p = (z_cordi.subs({t1: theta[0][0], t2: theta[1][0], t3: theta[2][0]}))
    x_data.append([x_p[0],x_p[1],x_p[2]])
    y_data.append([y_p[0],y_p[1],y_p[2]])
    z_data.append([z_p[0],z_p[1],z_p[2]])
