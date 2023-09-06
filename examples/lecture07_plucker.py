import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground([0, 0, -0.5])
env.set_gui_camera(look_at_pos=[0, 0, 0])
m.visualize_coordinate_frame()

# Point on line
p = np.array([0.4, 0.3, 0.2])
# Direction of line
q = np.array([0.1, 0.2, 0.3])
q_norm = q / np.linalg.norm(q)

q_0 = np.cross(p, q)

line = m.Line(p - q_norm, p + q_norm, radius=0.005, rgba=[1, 0, 0, 1])
point = m.Shape(m.Sphere(radius=0.02), static=True, position=p, rgba=[1, 0, 0, 1])

# Distance to origin
print('Distance of the line to the origin:', np.linalg.norm(q_0) / np.linalg.norm(q))

# Find point on line closest to origin
p_close = np.cross(q, q_0) / q.dot(q)
print('Point on line closest to origin:', p_close)
point_closest = m.Shape(m.Sphere(radius=0.02), static=True, position=p_close, rgba=[0, 1, 0, 1])
print('Distance of this point to the origin:', np.linalg.norm(p_close))
print('Distance of p to the origin:', np.linalg.norm(p))

# Test if points are on the line
print('p+q=[0.5, 0.5, 0.5] on line?', np.allclose(np.cross([0.5, 0.5, 0.5], q), q_0))
print('p+2q=[0.6, 0.7, 0.8] on line?', np.allclose(np.cross([0.6, 0.7, 0.8], q), q_0))
print('[0.6, 0.7, 0.81] on line?', np.allclose(np.cross([0.6, 0.7, 0.81], q), q_0))

m.step_simulation(steps=10000, realtime=True)


