import os
import numpy as np
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

for i in range(10000):
    # Euler rotation to quaternion
    q = m.get_quaternion([np.radians(i), 0, 0])
    x_new = m.rotate_point(x, q)
    point_x.set_base_pos_orient(x_new)

    y_new = m.rotate_point(x, m.get_quaternion([0, np.radians(i), 0]))
    point_y.set_base_pos_orient(y_new)

    z_new = m.rotate_point(x, m.get_quaternion([0, 0, np.radians(i)]))
    point_z.set_base_pos_orient(z_new)

    m.step_simulation(realtime=True)

