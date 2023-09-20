import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
env.set_gui_camera(look_at_pos=[0, 0, 0], yaw=30)

fbl = m.URDF(filename=os.path.join(m.directory, 'fourbarlinkage.urdf'), static=True, position=[0, 0, 0.3], orientation=[0, 0, 0, 1])
fbl.controllable_joints = [0, 1, 2]
# Create a constraint for the 4th joint to create a closed loop
fbl.create_constraint(parent_link=1, child=fbl, child_link=4, joint_type=m.p.JOINT_POINT2POINT, joint_axis=[0, 0, 0], parent_pos=[0, 0, 0], child_pos=[0, 0, 0])
m.step_simulation(steps=20, realtime=False)

coupler_links = [1, 3, 5]

links = [[1, 0], [3, 0]]
# links = [[1, 0], [3, 0], [3, -0.05]]
global_points = []
previous_global_points = []
lines = []

for [link, offset] in links:
    global_points.append(fbl.local_to_global_coordinate_frame([0, offset, 0], link=link)[0])
    previous_global_points.append(global_points[-1])
    point = m.Shape(m.Sphere(radius=0.02), static=True, collision=False, position=global_points[-1], rgba=[0, 0, 1, 1])

for i in range(10000):
    fbl.control([np.radians(i)]*3)

    if i % 5 == 0:
        # Draw coupler curve
        for j, link in enumerate(coupler_links):
            p = fbl.get_link_pos_orient(link)[0]
            color = [0, 0, 0, 1]
            color[j] = 1
            m.Shape(m.Sphere(radius=0.005), static=True, position=p, collision=False, rgba=color)

    if i > 3: # Ignore the first few time steps so that we can compute delta changes in position
        for j, (link_offset, global_position, previous_global_position) in enumerate(zip(links, global_points, previous_global_points)):
            # p_new = fbl.get_link_pos_orient(link)[0]
            p_new = fbl.local_to_global_coordinate_frame([0, link_offset[1], 0], link=link_offset[0])[0]
            ic_vector_of_motion = p_new - previous_global_position
            ic_bisector = np.cross(ic_vector_of_motion, [0,1,0])
            ic_bisector = ic_bisector / np.linalg.norm(ic_bisector)
            previous_global_points[j] = p_new

            if j >= len(lines):
                lines.append([])
                lines[j].append(m.Line(p_new, p_new+ic_vector_of_motion*10, radius=0.005, rgba=[1, 0, 0, 0.5]))
                lines[j].append(m.Line(p_new-ic_bisector, p_new+ic_bisector, radius=0.005, rgba=[0, 0, 1, 0.5]))
            else:
                lines[j][0] = m.Line(p_new, p_new+ic_vector_of_motion*10, radius=0.005, rgba=[1, 0, 0, 0.5], replace_line=lines[j][0])
                lines[j][1] = m.Line(p_new-ic_bisector, p_new+ic_bisector, radius=0.005, rgba=[0, 0, 1, 0.5], replace_line=lines[j][1])

    m.step_simulation(realtime=True)

