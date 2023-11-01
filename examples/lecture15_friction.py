import os
import numpy as np
import mengine as m

np.set_printoptions(precision=2, suppress=True)

mu = 0.3 # Coefficient of friction
end_effector_pos = np.array([0.3, 0.0, 0.95]) # TODO: Change 0.0 to 0.05 and watch at what point the end effector begins to slip across the block edge
end_effector_orient = np.array([np.pi, 0, 0])
cone_angle = np.arctan(mu)

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
cube.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)

# Let the cube drop onto the table
m.step_simulation(steps=50)

# Create Panda robot and initialize joint angles
robot = m.Robot.Panda(position=[0.5, 0, 0.75])
robot.set_whole_body_frictions(lateral_friction=1, spinning_friction=0, rolling_friction=0)
target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos, target_orient=end_effector_orient)
robot.control(target_joint_angles, set_instantly=True)

line1=line2=line3=line4=line5 = None
for i in range(1000):
    # Move the end effector to the left along a linear trajectory
    if end_effector_pos[0] > -0.27:
        end_effector_pos += np.array([-0.001, 0, 0])
    target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos, target_orient=end_effector_orient, use_current_joint_angles=True)
    robot.control(target_joint_angles)

    m.step_simulation()

    cp = robot.get_contact_points(bodyB=cube, average=True)
    if cp is not None:
        # Create our friction cone by rotating contact normal (a) around lateral friction dir 2 (b)
        # See for equation: https://math.stackexchange.com/questions/511370/how-to-rotate-one-vector-about-another
        a = -np.array(cp['contact_normal'])
        b = np.array(cp['lateral_friction_dir_2'])
        a_parallel_b = (a.dot(b) / b.dot(b)) * b
        a_perp_b = a - a_parallel_b
        w = np.cross(b, a_perp_b)
        a_b_angle = np.linalg.norm(a_perp_b) * ((np.cos(cone_angle)/np.linalg.norm(a_perp_b))*a_perp_b + (np.sin(cone_angle)/np.linalg.norm(w))*w)
        left_edge = a_b_angle + a_parallel_b
        a_b_angle2 = np.linalg.norm(a_perp_b) * ((np.cos(-cone_angle)/np.linalg.norm(a_perp_b))*a_perp_b + (np.sin(-cone_angle)/np.linalg.norm(w))*w)
        right_edge = a_b_angle2 + a_parallel_b

        f_t = mu * cp['normal_force']
        print('Friction magnitude:', f_t)
        print(cp['lateral_friction_dir_1'], cp['lateral_friction_dir_2'])

        # Visualize friction cone, contact normals, and friction directions
        n = np.array(cp['contact_normal']) / np.linalg.norm(cp['contact_normal'])
        d1 = np.array(cp['lateral_friction_dir_1']) / np.linalg.norm(cp['lateral_friction_dir_1'])
        d2 = np.array(cp['lateral_friction_dir_2']) / np.linalg.norm(cp['lateral_friction_dir_2'])
        line1 = m.Line(cp['posB'], np.array(cp['posB']) - n*0.2, rgb=[1, 0, 0], replace_line=line1)
        # line2 = m.Line(cp['posB'], np.array(cp['posB']) + d1*0.2, rgb=[0, 1, 0], replace_line=line2)
        # line3 = m.Line(cp['posB'], np.array(cp['posB']) + d2*0.2, rgb=[0, 0, 1], replace_line=line3)
        line4 = m.Line(cp['posB'], np.array(cp['posB']) + left_edge*0.2, rgb=[1, 1, 1], replace_line=line4)
        line5 = m.Line(cp['posB'], np.array(cp['posB']) + right_edge*0.2, rgb=[1, 1, 1], replace_line=line5)
