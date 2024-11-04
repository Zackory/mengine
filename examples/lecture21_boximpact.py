import os
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

# Create environment
env = m.Env()

def test_impact(restitution=0.5, mu=0.5, desired_distance=0.5, box_mass=1.0):
    print('-'*20)
    # Reset environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and box
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
    table.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)
    box = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=box_mass, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
    box.set_whole_body_frictions(lateral_friction=1, spinning_friction=0, rolling_friction=0)
    box.set_restitution(1)

    # Let the box drop onto the table
    m.step_simulation(steps=30, realtime=False)

    # Get box starting position for later computing distance traveled
    box_init_position = box.get_base_pos_orient()[0]

    # Create a sphere (finger) to collide with the box
    finger = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=box.local_to_global_coordinate_frame([0.5, 0, 0.025])[0], rgba=[1, 0, 0, 1])
    finger.set_restitution(restitution)

    m_b = box.get_link_mass(box.base)
    m_f = finger.get_link_mass(finger.base)
    v_f = (m_b + m_f)*np.sqrt(-2*mu*-9.81*desired_distance) / (m_f*(1+restitution))
    print('Velocity of finger:', v_f)

    finger.set_base_velocity([-v_f, 0, 0])

    v_b_expected = (m_f*v_f + m_f*restitution*v_f) / (m_b + m_f)

    impact = False
    for i in range(100):
        cp = finger.get_contact_points(bodyB=box)
        if cp is not None and not impact:
            print('Box velocity after impact:', np.linalg.norm(box.get_base_linear_velocity()), '| Box velocity expected:', v_b_expected)
            impact = True
        if not impact:
            finger.set_base_velocity([-v_f, 0, 0])

        # Apply a gravity compensation force to the finger
        force = -finger.get_link_mass(finger.base)*env.gravity # Gravity compensation force
        finger.apply_external_force(link=finger.base, force=force, pos=finger.get_base_pos_orient()[0], local_coordinate_frame=False)

        m.step_simulation()

    # Print out how far the box traveled
    box_final_position = box.get_base_pos_orient()[0]
    print('Distance traveled:', np.linalg.norm(box_final_position - box_init_position), '| Desired distance:', desired_distance)
    print('-'*20)


test_impact(restitution=1, mu=0.1, desired_distance=0.5, box_mass=1.0)
test_impact(restitution=0.75, mu=0.5, desired_distance=0.5, box_mass=1.0)
test_impact(restitution=0.75, mu=0.5, desired_distance=0.25, box_mass=1.0)
test_impact(restitution=0.75, mu=0.5, desired_distance=0.25, box_mass=3.0)

