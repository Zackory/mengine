import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
env.set_gui_camera(look_at_pos=[0, 0, 0])

cube = m.Shape(m.Box(half_extents=[0.1]*3), static=True, position=[0.2, 0, 0], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
p, q = cube.get_base_pos_orient()

local_points = [[0.1, 0.1, 0.1], [0.1, -0.1, 0.1], [0.1, 0, 0.1]]
global_points = []
previous_global_points = []
lines = []

for position in local_points:
    global_points.append(cube.local_to_global_coordinate_frame(position)[0])
    previous_global_points.append(global_points[-1])
    point = m.Shape(m.Sphere(radius=0.02), static=True, position=global_points[-1], rgba=[0, 0, 1, 1])

rotation_center = [0.2, -0.2, 0]
# rotation_center = [0, 0, 0]
point_rc = m.Shape(m.Sphere(radius=0.02), static=True, position=rotation_center + np.array([0,0,0.1]), collision=False, rgba=[1, 0, 0, 1])

m.step_simulation(steps=30, realtime=True)

# Plot instantaneous rotation center
for i in range(100):
    # Perform a rotation around a non-zero rotation center
    p2, q2 = m.multiply_transforms(rotation_center, [0, 0, np.radians(i+1)], p-rotation_center, q)
    cube.set_base_pos_orient(p2, q2)

    for j, (position, global_position, previous_global_position) in enumerate(zip(local_points, global_points, previous_global_points)):
        p_new = cube.local_to_global_coordinate_frame(position)[0]
        ic_vector_of_motion = p_new - previous_global_position
        ic_bisector = np.cross(ic_vector_of_motion, [0,0,1])
        ic_bisector = ic_bisector / np.linalg.norm(ic_bisector)
        previous_global_points[j] = p_new

        if j >= len(lines):
            lines.append([])
            # lines[j].append(m.Line(global_position, p_new, radius=0.005, rgba=[1, 1, 1, 0.5]))
            # lines[j].append(m.Line(midpoint, midpoint-bisector/2, radius=0.005, rgba=[1, 0, 0, 0.5]))
            lines[j].append(m.Line(p_new, p_new+ic_vector_of_motion*10, radius=0.005, rgba=[1, 0, 0, 0.5]))
            lines[j].append(m.Line(p_new, p_new-ic_bisector/2, radius=0.005, rgba=[0, 0, 1, 0.5]))
        else:
            # m.clear_visual_item(lines[j][0])
            # lines[j][0] = m.Line(global_position, p_new, radius=0.005, rgba=[1, 1, 1, 0.5])
            # lines[j][1] = m.Line(midpoint, midpoint-bisector/2, radius=0.005, rgba=[1, 0, 0, 0.5], replace_line=lines[j][1])
            lines[j][0] = m.Line(p_new, p_new+ic_vector_of_motion*10, radius=0.005, rgba=[1, 0, 0, 0.5], replace_line=lines[j][0])
            lines[j][1] = m.Line(p_new, p_new-ic_bisector/2, radius=0.005, rgba=[0, 0, 1, 0.5], replace_line=lines[j][1])

    m.step_simulation(realtime=True)

# Plot the intersecting lines for the rotation center
for i, (position, global_position, previous_global_position) in enumerate(zip(local_points, global_points, previous_global_points)):
    p_new = cube.local_to_global_coordinate_frame(position)[0]
    vector_of_motion = p_new - global_position
    midpoint = global_position + vector_of_motion/2
    bisector = np.cross(vector_of_motion, [0,0,1])
    bisector = bisector / np.linalg.norm(bisector)

    m.Line(global_position, p_new, radius=0.005, rgba=[1, 1, 1, 0.5])
    m.Line(midpoint, midpoint-bisector/2, radius=0.005, rgba=[1, 0, 0, 0.5])

while True:
    m.step_simulation(realtime=True)

