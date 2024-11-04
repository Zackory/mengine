import os
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

applied_pos = [0.3, 0, 0.05]
mu = 0.1

# Create environment and ground plane
env = m.Env()

def impact_test(f_n=-100.0):
    # Reset environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and cube
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
    table.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)
    cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])

    # Let the cube drop onto the table
    m.step_simulation(steps=30, realtime=False)

    # Create a sphere (finger) to collide with the cube
    finger = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=cube.local_to_global_coordinate_frame(applied_pos)[0], rgba=[1, 0, 0, 1])
    finger.set_whole_body_frictions(lateral_friction=100, spinning_friction=100, rolling_friction=100)

    # Create a collision-free sphere to visualize where we are applying force
    sphere = m.Shape(m.Sphere(radius=0.01), static=True, mass=0, position=cube.local_to_global_coordinate_frame([0.1, 0, 0])[0], collision=False, rgba=[1, 0, 0, 1])
    line = None

    for i in range(100):
        # Apply a force to the cube
        applied_down_force = [0, 0, f_n]
        applied_down_pos = [0, 0, 0.1]
        cube.apply_external_force(link=cube.base, force=applied_down_force, pos=applied_down_pos, local_coordinate_frame=True)

        # Apply a force to the finger
        force = -finger.get_link_mass(finger.base)*env.gravity # Gravity compensation force
        force += np.array([-3, 0, 0])
        finger.apply_external_force(link=finger.base, force=force, pos=finger.get_base_pos_orient()[0], local_coordinate_frame=False)

        m.step_simulation(realtime=True)

        # Show position of applied force and a line indicating the applied force magnitude
        pos_global = cube.local_to_global_coordinate_frame(applied_down_pos)[0]
        force_global = cube.local_to_global_coordinate_frame(applied_down_force)[0]
        sphere.set_base_pos_orient(pos_global)
        m.clear_visual_item(line)
        line = m.Line(pos_global, pos_global + (force_global-pos_global)/np.linalg.norm(force_global-pos_global)/2, rgb=[1, 0, 0])

impact_test(f_n=-100.0)
impact_test(f_n=-300.0)
