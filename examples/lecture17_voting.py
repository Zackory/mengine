import os, time
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

mu = 0.9 # Coefficient of friction
end_effector_pos = np.array([0.3, 0, 0.95])
end_effector_orient = np.array([np.pi, 0, 0])
cone_angle = np.arctan(mu)

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
cube = m.Shape(m.Box(half_extents=[0.27, 0.05, 0.1]), static=False, mass=1.0, position=[-0.05, -0.03, 1], orientation=[0, 0, np.pi/8], rgba=[0, 1, 0, 0.5])
cube.set_whole_body_frictions(lateral_friction=0.1, spinning_friction=0, rolling_friction=0)

# Create Panda robot and initialize joint angles
robot = m.Robot.Panda(position=[0.5, 0, 0.75])
target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos, target_orient=end_effector_orient)
robot.control(target_joint_angles, set_instantly=True)
robot.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)

# Let the cube drop onto the table
m.step_simulation(steps=50)

cube_pos, cube_orient = cube.get_link_pos_orient(cube.base, center_of_mass=True)
com = m.Shape(m.Sphere(radius=0.02), static=True, mass=0.0, collision=False, position=cube_pos, rgba=[1, 0, 0, 1])

line1=line2=line3=line4=line5=line6 = None
for i in range(1000):
    # Move the end effector to the left along a linear trajectory
    if end_effector_pos[0] > -0.27:
        end_effector_pos += np.array([-0.001, 0, 0])
    target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos, target_orient=end_effector_orient, use_current_joint_angles=True)
    robot.control(target_joint_angles)

    cube_pos, cube_orient = cube.get_link_pos_orient(cube.base, center_of_mass=True)
    com.set_base_pos_orient(cube_pos)

    m.step_simulation()

    if 350 < i < 450:
        time.sleep(0.05)
    # if 200 < i < 350:
    #     end_effector_pos += np.array([0.0016, 0.0006, 0])
    #     time.sleep(0.05)

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
        a_proj_b = a_b_angle + a_parallel_b
        a_b_angle2 = np.linalg.norm(a_perp_b) * ((np.cos(-cone_angle)/np.linalg.norm(a_perp_b))*a_perp_b + (np.sin(-cone_angle)/np.linalg.norm(w))*w)
        a_proj_b2 = a_b_angle2 + a_parallel_b

        # Visualize friction cone, contact normals, and friction directions
        line1 = m.Line(cp['posB'], np.array(cp['posB']) - np.array(cp['contact_normal'])*0.2, rgb=[1, 0, 0], replace_line=line1)
        line2 = m.Line(cp['posB'], np.array(cp['posB']) + np.array(cp['lateral_friction_dir_1'])*0.2, rgb=[0, 1, 0], replace_line=line2)
        line3 = m.Line(cp['posB'], np.array(cp['posB']) + np.array(cp['lateral_friction_dir_2'])*0.2, rgb=[0, 0, 1], replace_line=line3)
        line4 = m.Line(cp['posB'], np.array(cp['posB']) + a_proj_b*0.2, rgb=[1, 1, 1], replace_line=line4)
        line5 = m.Line(cp['posB'], np.array(cp['posB']) + a_proj_b2*0.2, rgb=[1, 1, 1], replace_line=line5)

        # ee_pos, ee_orient = robot.get_link_pos_orient(robot.end_effector)
        velocity = robot.get_link_velocity(robot.end_effector)
        line6 = m.Line(cp['posB'], cp['posB'] + velocity*3, rgb=[1, 1, 0], replace_line=line6)


