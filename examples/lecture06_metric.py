import os
import numpy as np
import matplotlib.pyplot as plt
import mengine as m
np.set_printoptions(precision=3, suppress=True)

# Create environment and ground plane
env = m.Env()
ground = m.Ground([0, 0, -0.5])
env.set_gui_camera(look_at_pos=[0, 0, 0])

# Point to rotate
x = np.array([0.2, 0.2, 0.2])

# Create point to rotate around axis
point_x = m.Shape(m.Sphere(radius=0.02), static=True, position=x, rgba=[1, 0, 0, 0.5])
point_y = m.Shape(m.Sphere(radius=0.02), static=True, position=x, rgba=[0, 1, 0, 0.5])
point_z = m.Shape(m.Sphere(radius=0.02), static=True, position=x, rgba=[0, 0, 1, 0.5])

m.visualize_coordinate_frame()

x_dist = []
y_dist = []
z_dist = []
x_dist2 = []
y_dist2 = []
z_dist2 = []

for i in range(360):
    # Compute quaternion distances to the identity quaternion
    p = np.array([0, 0, 0, 1])

    q = m.get_quaternion([np.radians(i), 0, 0])
    point_x.set_base_pos_orient(m.rotate_point(x, q))
    x_dist.append(np.linalg.norm(p-q))
    x_dist2.append(np.arccos(2*np.square(p.dot(q)) - 1))

    q = m.get_quaternion([np.radians(i), np.radians(i), 0])
    point_y.set_base_pos_orient(m.rotate_point(x, q))
    y_dist.append(np.linalg.norm(p-q))
    y_dist2.append(np.arccos(2*np.square(p.dot(q)) - 1))

    q = m.get_quaternion([np.radians(i), np.radians(i), np.radians(i)])
    point_z.set_base_pos_orient(m.rotate_point(x, q))
    z_dist.append(np.linalg.norm(p-q))
    z_dist2.append(np.arccos(2*np.square(p.dot(q)) - 1))

    m.step_simulation(realtime=True)

plt.figure(1)
plt.title('Euclidean distance')
plt.plot(x_dist, c=(1,0,0), label='x')
plt.plot(y_dist, c=(0,1,0), label='xy')
plt.plot(z_dist, c=(0,0,1), label='xyz')
plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlabel('time')
plt.ylabel('quaternion distance from identity')
plt.tight_layout()

plt.figure(2)
plt.title('arccos distance')
plt.plot(x_dist2, c=(1,0,0), label='x')
plt.plot(y_dist2, c=(0,1,0), label='y')
plt.plot(z_dist2, c=(0,0,1), label='z')
plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlabel('time')
plt.ylabel('quaternion distance from identity')
plt.tight_layout()

plt.show()

