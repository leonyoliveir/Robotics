import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rotn(n,t):
    '''
    Computes teh 3x3 matrix of a rotation around direction \hat{n} of angle t
    Inputs:
    * n --> Direction around which rotation will happen
    * t --> Theta angle of rotation
    '''

    nx, ny, nz = n
    norm = np.sqrt(nx**2 + ny**2 + nz**2)
    nx = nx/norm
    ny = ny/norm
    nz = nz/norm
    t = np.deg2rad(t)
    ct = np.cos(t)
    vt = 1 - ct
    st = np.sin(t)

    return np.array([
        [nx*nx*vt + ct,     nx*ny*vt - nz*st,   nx*nz*vt + ny*st],
        [nx*ny*vt + nz*st,  ny*ny*vt + ct,      ny*nz*vt - nx*st],
        [nx*nz*vt - ny*st,  ny*nz*vt + nx*st,   nz*nz*vt + ct]
    ])

n = np.array([1, 1, 1])
r = rotn(n, 90)
pl = np.array([1, 1, 0])
p = np.dot(r, pl)
print(p)

fig = plt.figure()
ax = Axes3D(fig)
ax.set_autoscale_on(False)
ax.quiver(0, 0, 0, 1, 0, 0, color='r')
ax.quiver(0, 0, 0, 0, 1, 0, color='g')
ax.quiver(0, 0, 0, 0, 0, 1, color='b')
plt.show()

t = np.linspace(0, 360, 35)
p = np.zeros((3, len(t)))
for i in range(len(t)):
    print(t[i])
    r = rotn(n, t[i])
    v = np.array([0, 1, 0]).reshape(3, 1)
    p[:, i] = np.matmul(r,v). reshape(3,)
xs = p[0, :]
ys = p[1, :]
zs = p[2, :]
fig2 = plt.figure()
ax2 = Axes3D(fig2)
ax2.set_autoscale_on(False)
ax2.plot(xs, ys, zs, 'ro')
ax2.quiver(0, 0, 0, 1, 0, 0, color='r')
ax2.quiver(0, 0, 0, 0, 1, 0, color='g')
ax2.quiver(0, 0, 0, 0, 0, 1, color='b')
plt.show()