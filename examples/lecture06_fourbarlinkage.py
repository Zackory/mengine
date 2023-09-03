import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
env.set_gui_camera(look_at_pos=[0, 0, 0], yaw=30)

fbl = m.URDF(filename=os.path.join(m.directory, 'fourbarlinkage.urdf'), static=True, position=[0, 0, 0.3], orientation=[0, 0, 0, 1])
fbl.controllable_joints = [0, 1, 2]

fbl.create_constraint(parent_link=1, child=fbl, child_link=4, joint_type=m.p.JOINT_POINT2POINT, joint_axis=[0, 0, 0], parent_pos=[0, 0, 0], child_pos=[0, 0, 0])

# m.visualize_coordinate_frame()

for i in range(10000):
    j = np.radians(i)
    fbl.control([j]*3)
    m.step_simulation(realtime=True)

