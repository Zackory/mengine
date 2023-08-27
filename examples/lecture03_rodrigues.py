import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create axis
axis = m.Shape(m.Cylinder(radius=0.02, length=0.5), static=True, position=[0, 0, 1], orientation=[-np.pi/4, -np.pi/4, 0], rgba=[0.8, 0.8, 0.8, 1])

# Create point to rotate around axis
point = m.Shape(m.Sphere(radius=0.02), static=True, position=[0.1, 0.1, 1.1], rgba=[1, 0, 0, 1])

# Define axis as point on top - point on bottom of axis
origin = axis.get_base_pos_orient()[0]
n = axis.local_to_global_coordinate_frame(pos=[0, 0, 0.25])[0] - axis.local_to_global_coordinate_frame(pos=[0, 0, -0.25])[0]
n = n / np.linalg.norm(n) # normalize
x = point.get_base_pos_orient()[0] - origin

for i in range(10000):
    theta = np.radians(i)
    # Rodrigues' formula (all 3 equations below are equivalent)
    x_new = n*n.dot(x) + np.sin(theta)*np.cross(n, x) - np.cos(theta)*np.cross(n, np.cross(n, x))

    # x_new = x + np.sin(theta)*np.cross(n, x) + ( 1- np.cos(theta))*np.cross(n, np.cross(n, x))
    # x_new = x*np.cos(theta) + np.sin(theta)*np.cross(n, x) + (1 - np.cos(theta))*n*n.dot(x)

    point.set_base_pos_orient(x_new + origin)

    m.step_simulation(realtime=True)


