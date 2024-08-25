import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
env.set_gui_camera(look_at_pos=[0, 0, 0])

# Show global coordinate frame
m.visualize_coordinate_frame()

cube = m.Shape(m.Box(half_extents=[0.1]*3), static=True, position=[0.2, 0, 0], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
p, q = cube.get_base_pos_orient()

# Rotate around the origin first
for i in range(100):
    p2, q2 = m.multiply_transforms([0, 0, 0], [0, 0, np.radians(i)], p, q)
    cube.set_base_pos_orient(p2, q2)
    m.step_simulation(realtime=True)

# Then translate along x-axis
for i in range(100):
    # p3, q3 = m.multiply_transforms(p2, q2, [i*0.005, 0, 0], [0, 0, 0])
    p3, q3 = m.multiply_transforms([i*0.005, 0, 0], [0, 0, 0], p2, q2)
    cube.set_base_pos_orient(p3, q3)
    m.step_simulation(realtime=True)


cube2 = m.Shape(m.Box(half_extents=[0.1]*3), static=True, position=[0.2, 0, 0], orientation=[0, 0, 0, 1], rgba=[0, 0, 1, 0.5])
p, q = cube2.get_base_pos_orient()

# Translate along x-axis first
for i in range(100):
    # p2, q2 = m.multiply_transforms(p, q, [i*0.005, 0, 0], [0, 0, 0])
    p2, q2 = m.multiply_transforms([i*0.005, 0, 0], [0, 0, 0], p, q)
    cube2.set_base_pos_orient(p2, q2)
    m.step_simulation(realtime=True)

# Then rotate around the origin
for i in range(100):
    p3, q3 = m.multiply_transforms([0, 0, 0], [0, 0, np.radians(i)], p2, q2)
    cube2.set_base_pos_orient(p3, q3)
    m.step_simulation(realtime=True)

while True:
    m.step_simulation(realtime=True)
