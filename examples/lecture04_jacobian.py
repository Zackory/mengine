import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env(time_step=0.1)
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

for i in range(25):
    # Move the end effector to a new pose
    target_joint_angles = robot.ik(robot.end_effector, target_pos=[0.2, 0.2, 0.8], target_orient=m.get_quaternion([np.pi, np.pi/4, 0]), use_current_joint_angles=True)
    robot.control(target_joint_angles)

    v = robot.get_joint_velocities(include_fixed_joints=False)
    J = robot.get_linear_jacobian(robot.end_effector)
    v_G = robot.get_link_velocity(robot.end_effector)

    # Is v_G = J.dot(v) ?
    print('Equal:', np.allclose(v_G, J.dot(v)), '\n\t v_G', v_G, '\n\t J.v', J.dot(v))

    m.step_simulation(realtime=True)

