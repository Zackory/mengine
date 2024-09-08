import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
m.visualize_coordinate_frame()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create box on table
width = 0.02
center = np.array([0, 0, 0.85])
m.Shape(m.Box(half_extents=[width, 0.2, 0.1]), static=True, position=center+[-0.2, 0, 0], rgba=[1, 1, 1, 0.75])
m.Shape(m.Box(half_extents=[0.2, width, 0.1]), static=True, position=center+[-width, -0.2+width, 0], rgba=[1, 1, 1, 0.75])
m.Shape(m.Box(half_extents=[0.2, width, 0.1]), static=True, position=center+[-width, 0.2-width, 0], rgba=[1, 1, 1, 0.75])
m.Shape(m.Box(half_extents=[width, 0.2, 0.1]), static=True, position=center+[0.2, 0, 0], rgba=[1, 1, 1, 0.75])

# Create Panda manipulator
robot = m.Robot.Panda(position=[0.5, 0, 0.75])

# Move end effector to a starting position using IK
pos = [0.3, 0, 1]
orient = m.get_quaternion([np.pi, 0, 0])
target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
robot.control(target_joint_angles, set_instantly=True)

items = ['cheezit.obj', 'spam.obj', 'mustard.obj', 'tomato_soup_can.obj', 'bowl.obj', 'mug.obj']
objects = []
for i in range(10000):
    if i % 50 == 0 and len(items) > 0:
        objects.append(m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', items.pop()), scale=[1, 1, 1]), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True))
    m.step_simulation(realtime=True)
