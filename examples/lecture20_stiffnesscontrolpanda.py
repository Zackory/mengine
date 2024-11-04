import os
import scipy
import numpy as np
import mengine as m
np.set_printoptions(precision=4, suppress=True)

print('TODO: FIX WITH NEW JACOBIAN FUNCTION')
exit()

mu_table = 3.0
mu_finger = 0.5
end_effector_pos_desired = np.array([0.3, 0, 0.95])
end_effector_orient_desired = np.array([np.pi, 0, 0])

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and box
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
table.set_whole_body_frictions(lateral_friction=mu_table, spinning_friction=0, rolling_friction=0)
wall = m.Shape(m.Box(half_extents=[0.05, 0.3, 0.2]), static=True, mass=0, position=[-0.5, 0, 0.9], orientation=[0, 0, 0, 1], rgba=[1, 1, 1, 1])
box = m.Shape(m.Box(half_extents=[0.1, 0.1, 0.05]), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])

# Let the box drop onto the table
m.step_simulation(steps=30, realtime=False)

# Create Panda robot and initialize joint angles
robot = m.Robot.Panda(position=[0.5, 0, 0.75])
target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos_desired, target_orient=end_effector_orient_desired)
robot.control(target_joint_angles, set_instantly=True)

target = m.Shape(m.Sphere(radius=0.02), static=True, mass=0.0, collision=False, position=end_effector_pos_desired, rgba=[1, 1, 0, 1])

keys = {m.p.B3G_UP_ARROW: [0, 0, 0.01], m.p.B3G_DOWN_ARROW: [0, 0, -0.01], m.p.B3G_LEFT_ARROW: [-0.01, 0, 0], m.p.B3G_RIGHT_ARROW: [0.01, 0, 0], ord('['): [0, 0.01, 0], ord(']'): [0, -0.01, 0]}
while True:
    key_events = m.p.getKeyboardEvents()
    for key, value in keys.items():
        if key in key_events:
            end_effector_pos_desired += np.array(value)

    target.set_base_pos_orient(end_effector_pos_desired)

    b = 1
    k = 1
    end_effector_pos, end_effector_orient = robot.get_link_pos_orient(robot.end_effector)
    end_effector_velocity = robot.get_link_velocity(robot.end_effector)
    force = -b*end_effector_velocity - k*(end_effector_pos - end_effector_pos_desired)
    direction = force / np.linalg.norm(force)
    print(force, direction, np.linalg.norm(force))
    # print(end_effector_velocity, end_effector_pos - end_effector_pos_desired)

    motor_indices, motor_positions, motor_velocities, motor_torques = robot.get_motor_joint_states()
    local_inertial_frame_position = m.p.getLinkState(robot.body, robot.end_effector, computeLinkVelocity=True, computeForwardKinematics=True, physicsClientId=env.id)[2]
    zeros = [0.0]*len(motor_positions)
    J = m.p.calculateJacobian(bodyUniqueId=robot.body, linkIndex=robot.end_effector, localPosition=local_inertial_frame_position, objPositions=motor_positions, objVelocities=zeros, objAccelerations=zeros, physicsClientId=env.id)[0]
    J = np.array(J)
    print(np.shape(J))

    K = np.eye(3)

    current_joint_angles = robot.get_joint_angles()
    target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos + np.array([-0.01, 0, 0]), target_orient=end_effector_orient_desired, use_current_joint_angles=True)
    print(np.shape(J), np.shape(np.matmul(np.matmul(J.T, K), J)), np.shape(target_joint_angles), np.shape(current_joint_angles))
    T = np.matmul(np.matmul(np.matmul(J.T, K), J), target_joint_angles - current_joint_angles)
    print(T.shape, robot.controllable_joints.shape)
    # m.p.setJointMotorControlArray(robot.body, jointIndices=robot.controllable_joints, controlMode=p.POSITION_CONTROL, targetPositions=targets, positionGains=gains, forces=forces, physicsClientId=self.id)

    # target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos + force*0.1, target_orient=end_effector_orient_desired, use_current_joint_angles=True)
    # robot.control(target_joint_angles)

    # target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos + direction*0.05, target_orient=end_effector_orient_desired, use_current_joint_angles=True)
    # robot.control(target_joint_angles, forces=50+np.linalg.norm(force)*100)#, gains=0.2

    m.step_simulation(realtime=True)
