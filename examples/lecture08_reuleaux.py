import os
import numpy as np
import mengine as m

env = m.Env(gravity=[0, 0, 0])

def reset(scenario=0):
    # Create environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create point to rotate around axis
    origin = np.array([0.15, 0, 1.0])
    if scenario == 2:
        origin = np.array([0.1, 0, 0.8])
    m.Shape(m.Sphere(radius=0.02), static=True, collision=False, position=origin, rgba=[1, 1, 1, 1])

    p = np.array([0, 0, 1])
    r = np.sqrt(2*(0.02**2))/2.5
    triangle = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'triangle.obj'), scale=[1]*3), static=False, position=p, rgba=[0, 1, 0, 0.5])
    s1 = m.Shape(m.Sphere(radius=0.02), static=True, position=p + [0, 0, -0.12], rgba=[1, 0, 0, 1])
    if scenario == 0:
        s2 = m.Shape(m.Sphere(radius=0.02), static=True, position=p + [0.05+r, 0, r], rgba=[1, 0, 0, 1])
    s3 = m.Shape(m.Sphere(radius=0.02), static=True, position=p + [-(0.05+r), 0, r], rgba=[1, 0, 0, 1])

    for i in range(500):
        pos, quat = triangle.get_base_pos_orient()
        axis, angle = m.get_axis_angle(quat)

        # Add a small rotation to the current angle
        theta = angle+np.radians(0.1)

        # Rotate the triangle around the y-axis of the given point
        x_new, q_new = m.multiply_transforms([0, 0, 0], m.get_quaternion([0, theta, 0]), p-origin, [0, 0, 0, 1])

        if i % 10 == 0:
            # Visualize the contact normals for Reuleaux's method
            m.clear_all_visual_items()
            cp = triangle.get_contact_points()
            if cp is not None:
                for c in cp:
                    m.Line(c['posB'], np.array(c['posB']) + np.array(c['contact_normal'])*0.3, radius=0.002, rgb=[0, 0, 1])

        triangle.set_base_pos_orient(x_new + origin, q_new)

        m.step_simulation(realtime=True)

reset(scenario=0) # No rotation possible, the object is fully constrained
reset(scenario=1) # No rotation possible with the chosen rotation center
reset(scenario=2) # We move the rotation center into the overlaping (-) negative region and then perform a negative rotation

