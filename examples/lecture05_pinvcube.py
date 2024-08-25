import os, time
import numpy as np
import matplotlib.pyplot as plt
import mengine as m
np.set_printoptions(precision=4, suppress=True)

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

all_v_G_desired = []
all_v_G = []
for i in range(300):
    # Compute distance between cube center and gripper as the desired gripper velocity
    cube_position = cube.get_base_pos_orient()[0]
    ee_position = robot.get_link_pos_orient(robot.end_effector)[0]
    v_G_desired = (cube_position - ee_position) * 0.75

    J = robot.get_linear_jacobian(robot.end_effector)
    v_target = np.linalg.pinv(J).dot(v_G_desired)
    v_target = np.array([v_target[j] for j in robot.controllable_joints])

    robot.control(v_target, velocity_control=True)
    m.step_simulation(realtime=True)

    v_G = robot.get_link_velocity(robot.end_effector)
    # v = robot.get_joint_velocities(include_fixed_joints=False)
    # print(i, '\n\t', v_target, '\n\t', v, '\n\t', v_G_desired, '\n\t', v_G)

    all_v_G_desired.append(v_G_desired)
    all_v_G.append(v_G)

all_v_G_desired = np.array(all_v_G_desired)
all_v_G = np.array(all_v_G)

# Plot the desired and actual end effector velocities
plt.plot(all_v_G_desired[:, 0], c=(1,0,0,0.2), label='x desired velocity')
plt.plot(all_v_G[:, 0], c=(1,0,0), label='x actual velocity')
plt.plot(all_v_G_desired[:, 1], c=(0,1,0,0.2), label='y desired velocity')
plt.plot(all_v_G[:, 1], c=(0,1,0), label='y actual velocity')
plt.plot(all_v_G_desired[:, 2], c=(0,0,1,0.2), label='z desired velocity')
plt.plot(all_v_G[:, 2], c=(0,0,1), label='z actual velocity')
plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
plt.xlabel('time')
plt.ylabel('velocity (m/s)')
plt.tight_layout()
plt.show()

