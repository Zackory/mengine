import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
m.visualize_coordinate_frame()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

wall = m.Shape(m.Box(half_extents=[0.02, 0.1, 0.1]), static=True, position=[0, 0, 0.85], orientation=[0, 0, 0, 1], rgba=[0, 1, 1, 0.75])

# Create cubes to grasp
cubes = []
for i in range(3):
    size = 0.025 - i*0.0025
    # position = [-0.2+i*0.1, 0.1-i*0.1, 1]
    position = [-0.1, 0.1-i*0.1, 1]
    yaw = np.pi/4*i
    cubes.append(m.Shape(m.Box(half_extents=[size]*3), static=False, mass=1, position=position, orientation=m.get_quaternion([0, 0, yaw]), rgba=[0, (i+1)/5.0, (i+1)/5.0, 0.75]))
    cubes[-1].set_whole_body_frictions(lateral_friction=50, spinning_friction=50, rolling_friction=0)

# Let the cube drop onto the table
m.step_simulation(steps=50)

# Create Panda manipulator
robot = m.Robot.Panda(position=[0.5, 0, 0.75])
robot.motor_gains = 0.02 # Slow the robot a bit (default is 0.05)

# Move end effector to a starting position using IK
pos = [0.2, 0, 0.85]
default_euler = np.array([np.pi, 0, 0])
orient = m.get_quaternion(default_euler)
target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
robot.control(target_joint_angles, set_instantly=True)
robot.set_gripper_position([1]*2, set_instantly=True) # Open gripper

def moveto(pos, orient):
    target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient, use_current_joint_angles=True)
    robot.control(target_joint_angles)
    while np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - target_joint_angles) > 0.03:
        m.step_simulation(realtime=True)

for i, current_cube in enumerate(range(len(cubes))):
    # MOVETO cube
    cube_pos, cube_orient = cubes[current_cube].get_base_pos_orient()
    gripper_orient = default_euler + [0, 0,  m.get_euler(cube_orient)[-1]]
    moveto(cube_pos + [0, 0, 0.05], gripper_orient)
    moveto(cube_pos, gripper_orient)

    # CLOSE
    robot.set_gripper_position([0]*2, force=500)
    m.step_simulation(steps=50, realtime=True)

    # MOVETO goal
    p, o = robot.get_link_pos_orient(robot.end_effector)
    moveto(p + [0, 0, 0.2], o)
    moveto([0.1, 0, 0.8+i*0.05], default_euler)

    # OPEN
    robot.set_gripper_position([1]*2)
    m.step_simulation(steps=50, realtime=True)
    p, o = robot.get_link_pos_orient(robot.end_effector)
    moveto(p + [0, 0, 0.1], o)

m.step_simulation(steps=1000, realtime=True)

