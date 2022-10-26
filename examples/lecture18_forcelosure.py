import os
import scipy
import numpy as np
import mengine as m
# np.set_printoptions(precision=2, suppress=True)


def contact_screw_2d(point, normal):
    normal = np.array(normal)
    C = normal / np.linalg.norm(normal)
    C0 = np.cross(point, normal)
    return np.array([C[0], C[1], C0])

def friction_cone_2d(mu, normal):
    cone_angle = np.arctan(mu)
    n = normal
    rotate_vector = lambda n, a: np.array([np.cos(a)*n[0] - np.sin(a)*n[1], np.sin(a)*n[0] + np.cos(a)*n[1]])
    left_edge = rotate_vector(normal, cone_angle)
    right_edge = rotate_vector(normal, -cone_angle)
    return left_edge, right_edge

def is_force_closure(W):
    # W = set of normalized contact screws
    wc = 1.0 / len(W) * np.sum(W, axis=0)
    T = W - wc
    p = lambda x: scipy.optimize.linprog(x, A_ub=T, b_ub=np.ones(len(W))).fun
    print('T rank:', np.linalg.matrix_rank(T))
    if np.linalg.matrix_rank(T) < 3: # "< 3" for planar, "< 6" for 3D
        return False
        # TTwc = np.matmul(np.matmul(T.T, np.linalg.pinv(T).T), wc.T)
        # print('TTwc - wc:', np.linalg.norm(TTwc - wc))
        # if not np.isclose(np.linalg.norm(TTwc - wc), 0):
        #     return False
    # NOTE: linprog does minimize, but we need maximize, so compute -p(wc) rather than p(-wc)
    return -p(wc) < 1

# Example from note sheet on Matt Mason's website. This is force closure, but note it is for a 1D space with 2D wrenches.
# W = np.array([[2, -1], [0, 1], [-2, -3]])
# print('Force closure:', is_force_closure(W))

# NOTE: This is the force closure grasp from lecture 13
p1, n1 = [1, -1], [0, 1]
p2, n2 = [1, -1], [-1, 0]
p3, n3 = [-1, 1], [0, -1]
p4, n4 = [-1, 1], [1, 0]
w1 = contact_screw_2d(p1, n1)
w2 = contact_screw_2d(p2, n2)
w3 = contact_screw_2d(p3, n3)
w4 = contact_screw_2d(p4, n4)

print(w1, w2, w3, w4)
print('Force closure:', is_force_closure([w1, w2, w3, w4]))

# If we change the location of contact point 1 from [1,-1] to [-1,-1], we no longer have force closure!
# If you look at the wrenches in wrench space, w1, w2, and w4 all lie on the n_0z = -1 plane. See lecture 13 figure. Origin is on boundary on convex hull, but not in interior!
w1 = contact_screw_2d([-1, -1], [0, 1])
print('Force closure:', is_force_closure([w1, w2, w3, w4]))


mu = 0.1
# TODO: Look at frictional force closure
# Verify the frictional force closure results in manipulation engine
l1, r1 = friction_cone_2d(mu, n1)
l2, r2 = friction_cone_2d(mu, n2)
l3, r3 = friction_cone_2d(mu, n3)
l4, r4 = friction_cone_2d(mu, n4)
wl1, wr1 = contact_screw_2d(p1, l1), contact_screw_2d(p1, r1)
wl2, wr2 = contact_screw_2d(p2, l2), contact_screw_2d(p2, r2)
wl3, wr3 = contact_screw_2d(p3, l3), contact_screw_2d(p3, r3)
wl4, wr4 = contact_screw_2d(p4, l4), contact_screw_2d(p4, r4)
print(l1, r1)
print(wl1, wr1)
print('Force closure:', is_force_closure([wl1, wr1, wl2, wr2, wl3, wr3, wl4, wr4]))

# Two contact forces facing each other, slightly offset, note p1=[1, 0.01]
p1, n1 = [1, 0.01], [-1, 0]
p2, n2 = [-1, 0], [1, 0]
l1, r1 = friction_cone_2d(mu, n1)
l2, r2 = friction_cone_2d(mu, n2)

w1 = contact_screw_2d(p1, n1)
w2 = contact_screw_2d(p2, n2)
print('Frictionless:', is_force_closure([w1, w2]))
wl1, wr1 = contact_screw_2d(p1, l1), contact_screw_2d(p1, r1)
wl2, wr2 = contact_screw_2d(p2, l2), contact_screw_2d(p2, r2)
print('With friction:', is_force_closure([wl1, wr1, wl2, wr2]))

exit()


mu = 0.1 # Coefficient of friction
end_effector_pos = np.array([0.3, 0, 0.95])
end_effector_orient = np.array([np.pi, 0, 0])
cone_angle = np.arctan(mu)

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
cube = m.Shape(m.Box(half_extents=[0.27, 0.05, 0.1]), static=False, mass=1.0, position=[-0.05, -0.03, 1], orientation=[0, 0, np.pi/8], rgba=[0, 1, 0, 0.5])
cube.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)

# Create Panda robot and initialize joint angles
robot = m.Robot.Panda(position=[0.5, 0, 0.75])
target_joint_angles = robot.ik(robot.end_effector, target_pos=end_effector_pos, target_orient=end_effector_orient)
robot.control(target_joint_angles)
robot.set_joint_angles(target_joint_angles)

# Let the cube drop onto the table
m.step_simulation(steps=50)

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
        a_proj_b = a_b_angle + a_parallel_b
        a_b_angle2 = np.linalg.norm(a_perp_b) * ((np.cos(-cone_angle)/np.linalg.norm(a_perp_b))*a_perp_b + (np.sin(-cone_angle)/np.linalg.norm(w))*w)
        a_proj_b2 = a_b_angle2 + a_parallel_b

        # Visualize friction cone, contact normals, and friction directions
        line1 = m.Line(cp['posB'], np.array(cp['posB']) - np.array(cp['contact_normal'])*0.2, rgb=[1, 0, 0], replace_line=line1)
        line2 = m.Line(cp['posB'], np.array(cp['posB']) + np.array(cp['lateral_friction_dir_1'])*0.2, rgb=[0, 1, 0], replace_line=line2)
        line3 = m.Line(cp['posB'], np.array(cp['posB']) + np.array(cp['lateral_friction_dir_2'])*0.2, rgb=[0, 0, 1], replace_line=line3)
        line4 = m.Line(cp['posB'], np.array(cp['posB']) + a_proj_b*0.2, rgb=[1, 1, 1], replace_line=line4)
        line5 = m.Line(cp['posB'], np.array(cp['posB']) + a_proj_b2*0.2, rgb=[1, 1, 1], replace_line=line5)

