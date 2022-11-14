import os
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

mu = 0
applied_pos = [0.2, 0, 0.025]

# Create environment
env = m.Env()

def test_impact(restitution):
    # Reset environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and cube
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
    cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
    cube.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)
    cube.set_restitution(1)

    # Let the cube drop onto the table
    m.step_simulation(steps=30, realtime=False)

    # Create a sphere (finger) to collide with the cube
    finger = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=cube.local_to_global_coordinate_frame(applied_pos)[0], rgba=[1, 0, 0, 1])
    finger.set_restitution(restitution)
    finger.set_base_velocity([-1, 0, 0])

    printed = False
    for i in range(100):
        cp = finger.get_contact_points(bodyB=cube)
        if cp is not None and not printed:
            print('Restitution:', restitution, 'Finger velocity:', finger.get_base_linear_velocity(), 'Cube velocity:', cube.get_base_linear_velocity())
            printed = True

        # Apply a gravity compensation force to the finger
        force = -finger.get_link_mass(finger.base)*env.gravity # Gravity compensation force
        finger.apply_external_force(link=finger.base, force=force, pos=finger.get_base_pos_orient()[0], local_coordinate_frame=False)

        m.step_simulation()

test_impact(0)
test_impact(0.5)
test_impact(1)

