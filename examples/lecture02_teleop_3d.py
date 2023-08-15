import os, time
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
pos = [0, 0, 1]
orient = m.get_quaternion([np.pi, 0, 0])
target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
robot.control(target_joint_angles, set_instantly=True)

# End effector pose
position, orientation = robot.get_link_pos_orient(robot.end_effector)
orientation = m.get_euler(orientation)

# Show global coordinate frame
m.visualize_coordinate_frame()
cf = None

# Map keys to position and orientation end effector movements
pos_keys_actions = {'j': [-0.01, 0, 0], 'l': [0.01, 0, 0],
                    'u': [0, -0.01, 0], 'o': [0, 0.01, 0],
                    'k': [0, 0, -0.01], 'i': [0, 0, 0.01]}
rpy_keys_actions = {'k': [-0.05, 0, 0], 'i': [0.05, 0, 0],
                    'u': [0, -0.05, 0], 'o': [0, 0.05, 0],
                    'j': [0, 0, -0.05], 'l': [0, 0, 0.05]}

while True:
    keys = m.get_keys()
    # Process position movement keys ('u', 'i', 'o', 'j', 'k', 'l')
    for key, action in pos_keys_actions.items():
        if 'shift' not in keys and key in keys:
            position += action
    for key, action in rpy_keys_actions.items():
        if 'shift' in keys and key in keys:
            orientation += action
    # Move the end effector to the new pose
    target_joint_angles = robot.ik(robot.end_effector, target_pos=position, target_orient=m.get_quaternion(orientation), use_current_joint_angles=True)
    robot.control(target_joint_angles)

    # Show local coordinate frame at robot end effector
    p, o = robot.get_link_pos_orient(robot.end_effector)
    cf = m.visualize_coordinate_frame(p, o, replace_old_cf=cf)

    m.step_simulation()
