import matplotlib.pyplot as plt
import numpy as np
from zh_DrawScene import brickPoses, myBrickMap
from zh_Utilities import hmRPYG

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Make data
'''
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = 10 * np.outer(np.cos(u), np.sin(v))
y = 10 * np.outer(np.sin(u), np.sin(v))
z = 10 * np.outer(np.ones(np.size(u)), np.cos(v))
print(z.shape)
'''

def drawBrick(pose, vertices, ax):
    x, y, z = hmRPYG(*pose[:3], pose[3:]).dot(vertices)[:3, :]
    print("x = {}, y = {}, z = {}:".format(x, y, z))

    xx = np.linspace(np.min(x), np.max(x), 100)
    yy = np.linspace(np.min(y), np.max(y), 100)
    zz = np.linspace(np.min(z), np.max(z), 10)

    # Parallel to the X-Y plane
    xxGrid, yyGrid = np.meshgrid(xx, yy)
    zzGrid = np.full_like(xxGrid, np.max(z))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue')
    zzGrid = np.full_like(xxGrid, np.min(z))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue')

    # Parallel to X-Z plane
    xxGrid, zzGrid = np.meshgrid(xx, zz)
    yyGrid = np.full_like(xxGrid, np.max(y))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue')
    yyGrid = np.full_like(xxGrid, np.min(y))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue')

    # Parallel to Y-Z plane
    yyGrid, zzGrid = np.meshgrid(yy, zz)
    xxGrid = np.full_like(yyGrid, np.max(x))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue')
    xxGrid = np.full_like(yyGrid, np.min(x))
    ax.plot_surface(xxGrid, yyGrid, zzGrid, color = 'lightblue')

drawBrick(brickPoses[0, :], myBrickMap.brickVertices, ax)


# Plot the surface
# ax.plot_surface(x, y, z)


# Set an equal aspect ratio
ax.set_aspect('equal')

plt.show()