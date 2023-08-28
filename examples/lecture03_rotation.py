import os
import numpy as np
import mengine as m
np.set_printoptions(precision=3, suppress=True)

# Create environment and ground plane
env = m.Env()
ground = m.Ground([0, 0, -0.5])
env.set_gui_camera(look_at_pos=[0, 0, 0])

# Create 2 coordinate frames
# rpy1 = [np.pi/4, np.pi/4, 0]
rpy1 = [0, 0, 0]
rpy2 = [-np.pi/4, 0, np.pi/4]
cf1 = m.visualize_coordinate_frame(position=[0, 0, 0], orientation=rpy1, alpha=0.5)
cf2 = m.visualize_coordinate_frame(position=[0, 0, 0], orientation=rpy2)

# Compute rotation matrix between these two frames
R_AB = m.get_rotation_matrix(rpy1).T.dot(m.get_rotation_matrix(rpy2))
# R = m.get_rotation_matrix(m.get_difference_quaternion(rpy1, rpy2))
R_BA = R_AB.T

# Create a point in coordinate frame 1
x_A = [0.1, 0.2, 0]
point = m.Shape(m.Sphere(radius=0.02), static=True, position=x_A, rgba=[1, 0, 0, 0.5])

# Transform while remaining in frame A
x_A_new = R_AB.dot(x_A)
point_new = m.Shape(m.Sphere(radius=0.02), static=True, position=x_A_new, rgba=[1, 0, 0, 1])

print('x_A:', x_A)
print('x\'_A:', x_A_new)

# Coordinate transform this new point to frame B
print('x\'_B:', R_BA.dot(x_A_new))

m.step_simulation(steps=10000, realtime=True)

