""" Plot the adaptive force on the end-effector in real-time """
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import seaborn
import redis
import numpy as np

import abr_jaco2

# connect to redis
r = redis.StrictRedis(host='localhost')

# create config
rc = abr_jaco2.robot_config(
    use_cython=True, hand_attached=True)
# generate functions needed
rc.J('EE', q=np.zeros(rc.num_joints))

folder = '/home/tdewolf/src/parameter_testing/adaptive/baseline/trial0'

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# create quiver plot
line = ax.plot([0, 1], [0, 1], [0,1 ], lw=5)
ax.quiver(0, 0, 0, 1, 0, 0, color='r')
ax.quiver(0, 0, 0, 0, 1, 0, color='g')
ax.quiver(0, 0, 0, 0, 0, 1, color='b')

def update_lines(t, line):
    # direction = np.random.random(6)
    # r.set('u_adapt', '%.3f %.3f %.3f %.3f %.3f %.3f' % tuple(direction))
    # r.set('q', '%.3f %.3f %.3f %.3f %.3f %.3f' % tuple(direction))

    # load in data
    q = r.get('q')
    u_adapt = r.get('u_adapt')

    if q is None or u_adapt is None:
        return line
    # if we have values, parse them
    q = [float(val) for val in q.decode('ascii').split()]
    u_adapt = [float(val) for val in u_adapt.decode('ascii').split()]

    # calculate the Jacobian
    J = rc.J('EE', q=q)
    # transform from 6D space to 3D
    u3D = np.dot(np.linalg.pinv(J.T), u_adapt) * .1

    # # plot force from ee xyz outward
    # plt.quiver([0], [0], [0], u3D[0], u3D[1], u3D[2])

    line.set_data([0, u3D[0]], [0, u3D[1]])
    line.set_3d_properties([0, u3D[2]])

    return line


# Setting the axes properties
ax.set_xlim3d([-1.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-1.0, 1.0])
ax.set_zlabel('Z')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 100,
                        fargs=(line), interval=5, blit=False)

plt.show()
