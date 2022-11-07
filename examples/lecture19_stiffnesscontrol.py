import os
import scipy
import numpy as np
import mengine as m
np.set_printoptions(precision=4, suppress=True)

mu_table = 3.0
mu_finger = 0.5
init_pos = [0.15, 0, 0.025]

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

# Create a sphere (finger) to collide with the box
finger = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=box.local_to_global_coordinate_frame(init_pos)[0], rgba=[1, 0, 0, 1])
finger.set_whole_body_frictions(lateral_friction=mu_finger, spinning_friction=10000, rolling_friction=10000)

finger_pos, finger_orient = finger.get_base_pos_orient()
desired_pos = finger_pos
x_desired = finger_pos[0]
z_desired = finger_pos[-1]

target = m.Shape(m.Sphere(radius=0.02), static=True, mass=0.0, collision=False, position=finger_pos, rgba=[1, 1, 0, 1])

keys = {m.p.B3G_UP_ARROW: [0, 0, 0.01], m.p.B3G_DOWN_ARROW: [0, 0, -0.01], m.p.B3G_LEFT_ARROW: [-0.01, 0, 0], m.p.B3G_RIGHT_ARROW: [0.01, 0, 0]}
while True:
    key_events = m.p.getKeyboardEvents()
    for key, value in keys.items():
        if key in key_events:
            desired_pos += np.array(value)

    # Apply a force to the finger
    force = -finger.get_link_mass(finger.base)*env.gravity # Gravity compensation force

    target.set_base_pos_orient(desired_pos)

    b = 25
    k = 50
    finger_pos, finger_orient = finger.get_base_pos_orient()
    finger.set_base_pos_orient([finger_pos[0], 0, finger_pos[-1]]) # Remove any movement in y-axis
    finger_velocity = finger.get_base_linear_velocity()
    force += np.array([-b*finger_velocity[0] - k*(finger_pos[0] - desired_pos[0]), 0, -b*finger_velocity[-1] - k*(finger_pos[-1] - desired_pos[-1])])
    # print(force)

    finger.apply_external_force(link=finger.base, force=force, pos=finger.get_base_pos_orient()[0], local_coordinate_frame=False)

    m.step_simulation(realtime=True)
