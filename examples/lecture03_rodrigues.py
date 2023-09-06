import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create point to rotate around axis
point = m.Shape(m.Sphere(radius=0.02), static=True, position=[0.1, 0.1, 1.1], rgba=[1, 0, 0, 1])

# Define an origin, direction vector n, and vector x from origin to point
origin = np.array([0, 0, 1])
n = np.array([-0.5, 1, 0.5])
n = n / np.linalg.norm(n) # normalize
x = point.get_base_pos_orient()[0] - origin

# Create axis
axis = m.Line(origin - n/3, origin + n/3, radius=0.02, rgba=[0.8, 0.8, 0.8, 1])

for i in range(10000):
    theta = np.radians(i)
    # Rodrigues' formula (all 3 equations below are equivalent)
    x_new = n*n.dot(x) + np.sin(theta)*np.cross(n, x) - np.cos(theta)*np.cross(n, np.cross(n, x))

    # x_new = x + np.sin(theta)*np.cross(n, x) + ( 1- np.cos(theta))*np.cross(n, np.cross(n, x))
    # x_new = x*np.cos(theta) + np.sin(theta)*np.cross(n, x) + (1 - np.cos(theta))*n*n.dot(x)

    point.set_base_pos_orient(x_new + origin)

    m.step_simulation(realtime=True)


