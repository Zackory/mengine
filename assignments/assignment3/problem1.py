import matplotlib.pyplot as plt
import os
import numpy as np
import mengine as m
np.set_printoptions(precision=3, suppress=True)

# NOTE: This assignment asks you to apply Reuleaux's method.


env = m.Env(gravity=[0, 0, 0])
camera = m.Camera()
camera.set_camera_rpy(look_at_pos=[0, 0, 0], distance=0.7, rpy=[0, -89.99, 0])
obj_pos = np.array([0, 0, 0])


def get_camera_image():
    rgba, _, _ = camera.get_rgba_depth()
    plt.imshow(rgba[:, :, :-1])
    plt.axis('off')
    plt.show()


def add_contact_point(position):
    # input: position: position of the contact point relativ to object position
    return m.Shape(m.Sphere(radius=0.02), static=True,
                   position=obj_pos + position, rgba=[1, 0, 0, 1])


def reset(scenario=0, n_steps=5, step_size=0.1):
    # Create environment and ground plane
    env.reset()
    ground = m.Ground([0, 0, -0.06])
    env.set_gui_camera(look_at_pos=obj_pos, distance=0.7, pitch=-89.99)

    # Create object
    box = m.Shape(m.Box(half_extents=[
        0.1, 0.08, 0.05]), static=False, position=obj_pos, rgba=[0, 1, 0, 0.5])

    radius_contact_spheres = 0.02
    rotation_axis = np.array([0, 0, 1])
    rotation_origin = None

    if scenario == 1.1:
        s1 = add_contact_point([-0.1 - radius_contact_spheres, 0, 0])
        s2 = add_contact_point([0.06, 0.08 + radius_contact_spheres, 0])

    if scenario == 1.2:
        s1 = add_contact_point([-0.1 - radius_contact_spheres, 0, 0])
        s2 = add_contact_point([0.06, 0.08 + radius_contact_spheres, 0])

        # ------ TODO Student answer below -------
        # Create point to rotate around axis
        # rotation_axis = ?
        # rotation_origin = ?
        # ------ Student answer above -------

    if scenario == 1.3:
        s1 = add_contact_point([-0.1 - radius_contact_spheres, 0, 0])
        s2 = add_contact_point([0.06, 0.08 + radius_contact_spheres, 0])

        # ------ TODO Student answer below -------
        # Add contact point(s) to fully constrain object
        # ------ Student answer above -------


################ Try to rotate object ##################
    for i in range(n_steps):
        if i % 4 == 0:
            # Visualize the contact normals for Reuleaux's method
            m.clear_all_visual_items()
            cp = box.get_contact_points()
            if cp is not None:
                for c in cp:
                    m.Line(c['posB'], np.array(
                        c['posB']) + np.array(c['contact_normal'])*0.3, radius=0.002, rgb=[0, 0, 1])

        if rotation_origin is not None:
            rotation_marker = m.Shape(m.Sphere(radius=0.02), static=True,
                                      collision=False, position=rotation_origin, rgba=[1, 1, 1, 1])

            pos, quat = box.get_base_pos_orient()
            axis, angle = m.get_axis_angle(quat)

            # Add a small rotation to the current angle
            theta = angle + np.radians(step_size)

            # Rotate the box around the axis of the given point
            x_new, q_new = m.multiply_transforms(
                [0, 0, 0], m.get_quaternion(rotation_axis*theta), obj_pos-rotation_origin, [0, 0, 0, 1])

            box.set_base_pos_orient(x_new + rotation_origin, q_new)

        m.step_simulation(realtime=True)


def wait_for_key(message):
    # NOTE: Press enter to continue
    print(message)
    keys = m.get_keys()
    while True:
        keys = m.get_keys()
        if 'return' in keys:
            break
        m.step_simulation(realtime=True)


reset(scenario=1.1, n_steps=5)
get_camera_image()
wait_for_key('Scenario 1.1: Press enter to continue...')
reset(scenario=1.2, n_steps=100)
get_camera_image()
wait_for_key('Scenario 1.2: Press enter to continue...')
reset(scenario=1.3, n_steps=5)
get_camera_image()
wait_for_key('Scenario 1.3: Press enter to continue...')
