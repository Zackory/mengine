import os, time
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

mu = 0.1 # Coefficient of friction
restitution = 1
end_effector_pos = np.array([0.3, 0, 0.925])
end_effector_orient = np.array([np.pi, 0, 0])

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
table.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
cube.set_whole_body_frictions(lateral_friction=1, spinning_friction=0, rolling_friction=0)
cube.set_restitution(1)

# Let the cube drop onto the table
m.step_simulation(steps=50, realtime=False)

# Get cube starting position for later computing distance traveled
cube_init_position = cube.get_base_pos_orient()[0]

# Create Panda robot and initialize joint angles
robot = m.Robot.Panda(position=[0.5, 0, 0.75])
target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos, target_orient=end_effector_orient)
robot.control(target_joint_angles, set_instantly=True)
robot.set_restitution(restitution)

contacted = False
for i in range(100):
    cp = robot.get_contact_points(bodyB=cube)
    if cp is not None and not contacted:
        print('Restitution:', restitution, 'Robot velocity:', robot.get_link_velocity(robot.end_effector), 'Cube velocity:', cube.get_base_linear_velocity())
        contacted = True
        distance_estimate = np.square(np.linalg.norm(cube.get_base_linear_velocity())) / (2*mu*9.81)

    # Move the end effector to the left along a linear trajectory
    if end_effector_pos[0] > -0.27 and not contacted:
        end_effector_pos += np.array([-0.01, 0, 0])
    else:
        end_effector_pos = robot.get_link_pos_orient(robot.end_effector)[0]
    target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos, target_orient=end_effector_orient, use_current_joint_angles=True)
    robot.control(target_joint_angles)

    m.step_simulation()

cube_final_position = cube.get_base_pos_orient()[0]
print('Distance traveled:', np.linalg.norm(cube_final_position - cube_init_position), '| Estimated distance traveled:', distance_estimate)
