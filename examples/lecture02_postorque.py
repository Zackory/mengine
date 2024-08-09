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

# Create a cube to make contact with
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1, position=[-0.25, 0, 0.85], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 1])

# Move end effector to a starting position using IK
original_joint_angles = robot.ik(robot.end_effector, target_pos=[0, 0, 0.9], target_orient=m.get_quaternion([np.pi, 0, 0]))
robot.control(original_joint_angles, set_instantly=True)



# Position control
for i in range(100):
    position, orientation = robot.get_link_pos_orient(robot.end_effector)
    target_joint_angles = robot.ik(robot.end_effector, target_pos=position+[-0.04, 0, 0], target_orient=orientation, use_current_joint_angles=True)
    robot.control(target_joint_angles)

    m.step_simulation(realtime=True)

# Reset env
cube.set_base_pos_orient([-0.25, 0, 0.85], [0, 0, 0, 1])
robot.control(original_joint_angles, set_instantly=True)



# Velocity control
for i in range(100):
    position, orientation = robot.get_link_pos_orient(robot.end_effector)
    target_joint_angles = robot.ik(robot.end_effector, target_pos=position+[-0.1, 0, 0], target_orient=orientation, use_current_joint_angles=True)
    current_joint_angles = robot.get_joint_angles()[robot.controllable_joints]
    delta_angles = target_joint_angles - current_joint_angles
    robot.control(delta_angles, velocity_control=True)

    m.step_simulation(realtime=True)

# Stop the robot motors (otherwise they will keep moving at the last set velocity)
robot.control([0]*len(robot.controllable_joints), velocity_control=True)
# Reset env
cube.set_base_pos_orient([-0.25, 0, 0.85], [0, 0, 0, 1])
robot.control(original_joint_angles, set_instantly=True)



# Torque control
position, orientation = robot.get_link_pos_orient(robot.end_effector)
theta_e_prev = None
for i in range(100):
    M = robot.get_mass_matrix()

    position += [-0.002, 0, 0]
    target_joint_angles = robot.ik(robot.end_effector, target_pos=position, target_orient=orientation, use_current_joint_angles=True)
    current_joint_angles = robot.get_joint_angles()[robot.controllable_joints]
    delta_angles = target_joint_angles - current_joint_angles
    theta_e = np.zeros(len(M))
    theta_e[robot.controllable_joints] = delta_angles

    # Computed torque control - PD torque control
    Kp = np.eye(len(M))*5000
    Kd = np.eye(len(M))*100
    if theta_e_prev is None:
        theta_e_prev = theta_e
    tau = M.dot(Kp.dot(theta_e) + Kd.dot(theta_e - theta_e_prev))[robot.controllable_joints]
    theta_e_prev = theta_e

    robot.control(tau, torque_control=True)

    m.step_simulation(realtime=True)

m.step_simulation(steps=1000, realtime=True)

