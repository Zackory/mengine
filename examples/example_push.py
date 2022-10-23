import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
orient = m.get_quaternion([np.pi, 0, 0])

def reset(position, table_friction=0.5, cube_mass=100, cube_friction=0.5):
    # Create environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and cube
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1], maximal_coordinates=True)
    table.set_whole_body_frictions(lateral_friction=table_friction, spinning_friction=0, rolling_friction=0)
    cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=cube_mass, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 1])
    cube.set_whole_body_frictions(lateral_friction=cube_friction, spinning_friction=0, rolling_friction=0)

    # Create Panda robot
    robot = m.Robot.Panda(position=[0.5, 0, 0.75])

    # Initialize joint angles
    target_joint_angles = robot.ik(robot.end_effector, target_pos=position, target_orient=orient)
    robot.control(target_joint_angles)
    robot.set_joint_angles(target_joint_angles)
    return robot, cube

# Investigate how the mass of the cube, the lateral_friction of the table, and the motor_force of the robot affects pushing the block.

# Robot not strong enough to break static friction and push block
scenarios = [dict(position=np.array([0.3, 0, 0.95]), table_friction=0.5, cube_mass=100, cube_friction=0.5, robot_force=50)]
# Robot stronger and can break static friction
scenarios.append(dict(position=np.array([0.3, 0, 0.95]), table_friction=0.5, cube_mass=100, cube_friction=0.5, robot_force=100))

# Robot pushes cube along center axis (in a straight line)
scenarios.append(dict(position=np.array([0.3, 0, 0.95]), table_friction=0.5, cube_mass=1, cube_friction=0.5, robot_force=50))
# Robot pushes cube off-diagonal, causing the cube to spin
scenarios.append(dict(position=np.array([0.3, 0.05, 0.95]), table_friction=0.5, cube_mass=1, cube_friction=0.5, robot_force=50))
# Decrease friction between cube and robot. Watch how the end effector slides along the cube
scenarios.append(dict(position=np.array([0.3, 0.05, 0.95]), table_friction=0.5, cube_mass=1, cube_friction=0.01, robot_force=50))

for s in scenarios:
    pos = s['position']
    # Reset simulator
    robot, cube = reset(s['position'], s['table_friction'], s['cube_mass'], s['cube_friction'])

    for i in range(300):
        # Move the end effector to the left along a linear trajectory
        if pos[0] > -0.2:
            pos += np.array([-0.0025, 0, 0])
        target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient, use_current_joint_angles=True)
        robot.control(target_joint_angles, forces=s['robot_force'])

        m.step_simulation()

        # Show contact normals
        cp = robot.get_contact_points(bodyB=cube)
        m.clear_all_visual_items()
        if cp is not None:
            for c in cp:
                line = m.Line(c['posB'], np.array(c['posB']) + np.array(c['contact_normal'])*0.2, rgb=[1, 0, 0])

