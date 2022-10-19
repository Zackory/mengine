import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create Panda manipulator
robot = m.Robot.Panda(position=[0.5, 0, 0.75])

# Move end effector to a starting position using IK
pos = [0, 0, 1.2]
orient = None
target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
robot.control(target_joint_angles)
robot.set_joint_angles(target_joint_angles)


# Show global coordinate frame
m.visualize_coordinate_frame()
cf = None

for i in range(1000):
    # Move the end effector between two different positions
    pos = [0.2, 0.2, 0.9] if (i % 200) < 100 else [0, 0, 1.2]
    target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient, use_current_joint_angles=True)
    robot.control(target_joint_angles)

    # Show local coordinate frame at robot end effector
    position, orientation = robot.get_link_pos_orient(robot.end_effector)
    m.clear_visual_item(cf)
    cf = m.visualize_coordinate_frame(position, orientation)

    m.step_simulation()


