R = rotx(pi/2)
trplot(R)
tranimate(R)
%%
R = rotx(pi/2) * roty(pi/2)
trplot(R)
%%
roty(pi/2)*rotx(pi/2)
%%
R = rotz(0.1) * roty(0.2) * rotz(0.3)
R = eul2r(0.1, 0.2, 0.3)
gamma = tr2eul(R)
%%
R = eul2r(0.1 , -0.2, 0.3)
tr2eul(R)
eul2r(ans)
%%
R = eul2r(0.1 , 0, 0.3)
tr2eul(R)
%%
R = rpy2r(0.1, 0.2, 0.3)
gamma = tr2rpy(R)