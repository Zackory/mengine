import os
import numpy as np
import mengine as m

env = m.Env()
ground = m.Ground()

# sphere = m.Shape(m.Sphere(radius=0.1), static=True, mass=0.0, position=[0, 0, 0], orientation=[0, 0, 0, 1], rgba=[0, 1, 1, 1])
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

robot = m.Robot.Panda(position=[0.5, 0, 0.75])

pos = [0, 0, 1.2]
orient = None#m.get_quaternion([0, -np.pi/2, 0])
# sphere = m.Shape(m.Sphere(radius=0.05), static=True, mass=0.0, position=pos, orientation=[0, 0, 0, 1], rgba=[0, 1, 1, 1])
target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
robot.control(target_joint_angles)
robot.set_joint_angles(target_joint_angles)


# Show global coordinate frame
m.visualize_coordinate_frame()
cf = None

# while True:
for i in range(1000):
    pos = [0.2, 0.2, 0.9] if (i % 200) < 100 else [0, 0, 1.2]
    target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient, use_current_joint_angles=True)
    robot.control(target_joint_angles)

    # Show local coordinate frame at robot end effector
    position, orientation = robot.get_link_pos_orient(robot.end_effector)
    cf = m.visualize_coordinate_frame(position, orientation, replace_old_cf=cf)

    m.step_simulation()


