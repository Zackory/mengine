import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env(gravity=[0, 0, 0])
ground = m.Ground([0, 0, -0.5])
env.set_gui_camera(look_at_pos=[0, 0, 0], yaw=30)

sphere = m.Shape(m.Sphere(radius=0.1), static=False, mass=1.0, collision=True, position=[0, 0.1, 0.2], rgba=[0, 1, 0, 1])

# Create a Salisbury hand
hand = m.salisbury_hand()
hand.control([0]*9, velocity_control=True, forces=0) # Disable motors in the fingers

for link in [2, 5, 8]:
    c = hand.get_closest_points(sphere, distance=4.0, linkA=link, linkB=sphere.base)
    print(c)
    if len(c) > 0:
        # m.Shape(m.Sphere(radius=0.02), static=True, collision=False, position=c[2][0]+np.array([0,0,0.03]), rgba=[1, 0, 0, 1])
        # m.Shape(m.Sphere(radius=0.02), static=True, collision=False, position=c[3][0], rgba=[0, 0, 1, 1])
        hand_local_pos, _ = hand.global_to_local_coordinate_frame(c[2][0], link=link)
        hand_local_pos += [0, 0, 0.03] # Use the tips of the fingers
        sphere_local_pos, _ = sphere.global_to_local_coordinate_frame(c[3][0])
        hand.create_constraint(parent_link=link, child=sphere, child_link=sphere.base, joint_type=m.p.JOINT_POINT2POINT, joint_axis=[0, 0, 0], parent_pos=hand_local_pos, child_pos=sphere_local_pos)

pos_keys_actions = {',': [-0.01, 0, 0], '/': [0.01, 0, 0],
                    'l': [0, -0.01, 0], '\'': [0, 0.01, 0],
                    '.': [0, 0, -0.01], ';': [0, 0, 0.01]}

for i in range(10000):
    pos, orient = sphere.get_base_pos_orient()
    keys = m.get_keys()
    # Process position movement keys (',', '/', 'l', ''', ';', '.')
    for key, action in pos_keys_actions.items():
        if key in keys:
            pos += action
    sphere.set_base_pos_orient(pos)
    m.step_simulation(realtime=True)

