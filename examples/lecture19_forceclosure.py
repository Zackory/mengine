import os
import scipy
import numpy as np


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
    print('W:', W, '\n', 'wc:', wc, '\n', 'T:', T, '\n', 'T rank:', np.linalg.matrix_rank(T), '\n', 'p(-wc):', -p(wc))
    if np.linalg.matrix_rank(T) < 3: # "< 3" for planar, "< 6" for 3D
        return False
    # NOTE: linprog does minimize, but we need maximize, so compute -p(wc) rather than p(-wc)
    return -p(wc) < 1

# NOTE: This is similar to the force closure grasp from lecture 13
p1, n1 = [0, -1], [0, 1]
p2, n2 = [1, -1], [-1, 0]
p3, n3 = [-1, 1], [0, -1]
p4, n4 = [-1, 1], [1, 0]
w1 = contact_screw_2d(p1, n1)
w2 = contact_screw_2d(p2, n2)
w3 = contact_screw_2d(p3, n3)
w4 = contact_screw_2d(p4, n4)

# print(w1, w2, w3, w4)
# print('Force closure:', is_force_closure([w1, w2, w3, w4]))
# exit()

# If we change the location of contact point 1 from [0,-1] to [-1,-1], we no longer have force closure!
# If you look at the wrenches in wrench space, w1, w2, and w4 all lie on the n_0z = -1 plane. See lecture 13 figure. Origin is on boundary on convex hull, but not in interior!
# w1 = contact_screw_2d([-1, -1], [0, 1])
# print('Force closure:', is_force_closure([w1, w2, w3, w4]))
# exit()


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
# print(l1, r1)
# print(wl1, wr1)
# print('Force closure:', is_force_closure([wl1, wr1, wl2, wr2, wl3, wr3, wl4, wr4]))
# exit()

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

