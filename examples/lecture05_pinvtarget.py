import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])

# Let the cube drop onto the table
m.step_simulation(steps=20)

# Create Panda manipulator
robot = m.Robot.Panda(position=[0.5, 0, 0.75])

# Move end effector to a starting position using IK
# pos = [0, 0, 1]
pos = [0.3, 0, 0.95]
orient = m.get_quaternion([np.pi, 0, 0])
target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
robot.control(target_joint_angles, set_instantly=True)

desired_pos = robot.get_link_pos_orient(robot.end_effector)[0]
target = m.Shape(m.Sphere(radius=0.02), static=True, mass=0.0, collision=False, position=desired_pos, rgba=[0, 1, 0, 1])

keys_actions = {'j': [-0.01, 0, 0], 'l': [0.01, 0, 0],
                    'u': [0, -0.01, 0], 'o': [0, 0.01, 0],
                    'k': [0, 0, -0.01], 'i': [0, 0, 0.01]}

while True:
    keys = m.get_keys()
    # Process position movement keys ('u', 'i', 'o', 'j', 'k', 'l')
    for key, action in keys_actions.items():
        if 'shift' not in keys and key in keys:
            desired_pos += action

    target.set_base_pos_orient(desired_pos)

    # Compute distance between cube center and gripper as the desired gripper velocity
    ee_position = robot.get_link_pos_orient(robot.end_effector)[0]
    v_G_desired = (desired_pos - ee_position) * 0.75

    J = robot.get_linear_jacobian(robot.end_effector)
    v_target = np.linalg.pinv(J).dot(v_G_desired)
    v_target = np.array([v_target[j] for j in robot.controllable_joints])

    robot.control(v_target, velocity_control=True)
    m.step_simulation(realtime=True)

