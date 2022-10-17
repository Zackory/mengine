import os, time
import numpy as np
import pybullet as p
import mengine as m

env = m.Env()
orient = m.get_quaternion([np.pi, 0, 0])

# Investigate how the mass of the cube, the lateral_friction of the table, and the motor_force of the robot affects pushing the block.
    # At mass=100, lateral_friction=0.5, and force=50, the robot fails to break static friction
    # At mass=100, lateral_friction=0.5, and force=100, the robot can break static friction
# Investigate how the contact point with the cube impacts how the cube slides across the table

def reset(position, table_friction=0.5, cube_mass=100, cube_friction=0.5):
    env.reset()
    ground = m.Ground()
    m.visualize_coordinate_frame()

    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1], maximal_coordinates=True)
    table.set_whole_body_frictions(lateral_friction=table_friction, spinning_friction=0, rolling_friction=0)
    cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=cube_mass, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 1])
    cube.set_whole_body_frictions(lateral_friction=cube_friction, spinning_friction=0, rolling_friction=0)

    robot = m.Robot.Panda(position=[0.5, 0, 0.75])

    target_joint_angles = robot.ik(robot.end_effector, target_pos=position, target_orient=orient)
    robot.control(target_joint_angles)
    robot.set_joint_angles(target_joint_angles)
    return robot

scenarios = [dict(position=np.array([0.3, 0, 0.95]), table_friction=0.5, cube_mass=100, cube_friction=0.5, robot_force=50)]
scenarios.append(dict(position=np.array([0.3, 0, 0.95]), table_friction=0.5, cube_mass=100, cube_friction=0.5, robot_force=100))

scenarios.append(dict(position=np.array([0.3, 0, 0.95]), table_friction=0.5, cube_mass=1, cube_friction=0.5, robot_force=50))
scenarios.append(dict(position=np.array([0.3, 0.05, 0.95]), table_friction=0.5, cube_mass=1, cube_friction=0.5, robot_force=50))
# Watch how the end effector slides along the cube
scenarios.append(dict(position=np.array([0.3, 0.05, 0.95]), table_friction=0.5, cube_mass=1, cube_friction=0.01, robot_force=50))

for s in scenarios:
    pos = s['position']
    robot = reset(s['position'], s['table_friction'], s['cube_mass'], s['cube_friction'])

    for i in range(500):
        if pos[0] > -0.2:
            pos += np.array([-0.002, 0, 0])
        target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient, use_current_joint_angles=True)
        robot.control(target_joint_angles, forces=s['robot_force'])

        p.stepSimulation(physicsClientId=env.id)
        env.slow_time()

