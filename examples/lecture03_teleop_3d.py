import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])

# Let the cube drop onto the table
m.step_simulation(steps=20)

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

coordinate_frames = []
last_marker_time = 0

while True:
    keys = m.get_keys()
    # Process position movement keys ('u', 'i', 'o', 'j', 'k', 'l')
    for key, action in pos_keys_actions.items():
        if 'shift' not in keys and key in keys:
            position += action
    for key, action in rpy_keys_actions.items():
        if 'shift' in keys and key in keys:
            orientation += action
    if 'm' in keys and time.time() > last_marker_time + 1:
        # Mark (save) the coordinate frame for the current end effector pose
        coordinate_frames.append(robot.get_link_pos_orient(robot.end_effector))
        m.visualize_coordinate_frame(coordinate_frames[-1][0], coordinate_frames[-1][1])
        last_marker_time = time.time()
        # Print out distance metrics between each coordinate frame
        for i, (p1, q1) in enumerate(coordinate_frames):
            for j, (p2, q2) in enumerate(coordinate_frames[i+1:]):
                print('Euclidean between (%d) and (%d):' % (i, i+1+j), np.linalg.norm(p2 - p1))
                print('Manhattan between (%d) and (%d):' % (i, i+1+j), np.sum(np.abs(p2 - p1)))
                print('Chebyshev between (%d) and (%d):' % (i, i+1+j), np.max(np.abs(p2 - p1)))
                print('Quaternion distance between (%d) and (%d)' % (i, i+1+j), np.arccos(2*np.square(q1.dot(q2)) - 1))
        print('-'*20)
    if 'c' in keys:
        # Clear all coordinate frames
        m.clear_all_visual_items()
        coordinate_frames = []
        cf = None

    # Move the end effector to the new pose
    target_joint_angles = robot.ik(robot.end_effector, target_pos=position, target_orient=m.get_quaternion(orientation), use_current_joint_angles=True)
    robot.control(target_joint_angles)

    # Show local coordinate frame at robot end effector
    p, o = robot.get_link_pos_orient(robot.end_effector)
    cf = m.visualize_coordinate_frame(p, o, replace_old_cf=cf)

    m.step_simulation(realtime=True)
