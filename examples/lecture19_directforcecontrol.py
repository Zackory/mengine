import os
import scipy
import numpy as np
import mengine as m
np.set_printoptions(precision=4, suppress=True)

mu_table = 3.0
mu_finger = 0.2
applied_pos = [0.15, 0, 0.025]

# Create environment
env = m.Env()

def direct_force_control(constant_force=False):
    # Reset environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and box
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
    table.set_whole_body_frictions(lateral_friction=mu_table, spinning_friction=0, rolling_friction=0)
    box = m.Shape(m.Box(half_extents=[0.1, 0.1, 0.05]), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
    box_width = 0.2

    # Let the box drop onto the table
    m.step_simulation(steps=30, realtime=False)

    # Create a sphere (finger) to collide with the box
    finger = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=box.local_to_global_coordinate_frame(applied_pos)[0], rgba=[1, 0, 0, 1])
    finger.set_whole_body_frictions(lateral_friction=mu_finger, spinning_friction=10000, rolling_friction=10000)

    theta_desired = -np.pi/2
    integral_error = 0
    cf = None

    for i in range(150):
        # Apply a force to the finger
        force = -finger.get_link_mass(finger.base)*env.gravity # Gravity compensation force
        if constant_force:
            force += np.array([-10, 0, 0])

        box_pos, box_orient = box.get_base_pos_orient()
        theta = m.get_euler(box_orient)[1] # Get the pitch of the box (rotation along global y-axis)
        theta_dot = m.get_euler(box.get_base_angular_velocity())[1]
        cf = m.visualize_coordinate_frame(box_pos, box_orient, replace_old_cf=cf)

        kp = 20.0
        ki = 0.0
        kd = 0.0
        integral_error += theta_desired - theta
        pid = kp*(theta_desired - theta) + ki*integral_error - kd*theta_dot

        # Tranform force vector to local box coordiante frame (just consider rotation of box, we don't want to translate the force vector)
        force_on_box_frame = lambda f: box.global_to_local_coordinate_frame([f[0], 0, f[1]], orient=[0, 0, 0, 1], rotation_only=True)[0][[0, -1]] # Get only x and z axis [0, -1], throw away y-axis

        # The difference in torque if the finger applied force at the box at the current finger position, versus the actual box rotation (from PID)
        cost = lambda force_control: (box_width*force_on_box_frame(force_control)[0] - pid)**2
        # cost = lambda force_control: (np.linalg.norm(np.cross(box_pos - finger_pos, [force_control[0], 0, force_control[1]])) - pid)**2
        # cost = lambda force_control: (box_width*(np.sin(theta)*force_control[0] + np.cos(theta)*force_control[1]) - pid)**2

        constraints = []
        # Ensure that the tangential (z-axis) force does not exceed the friction cone (mu_finger * f_x). Note it is -mu_finger, since we are moving along negative x-axis
        constraints.append({'type': 'ineq', 'fun': lambda force_control: -mu_finger * force_on_box_frame(force_control)[0] - force_on_box_frame(force_control)[-1]})
        constraints.append({'type': 'ineq', 'fun': lambda force_control: -mu_finger * force_on_box_frame(force_control)[0] + force_on_box_frame(force_control)[-1]})

        # z-axis force can never exceed box_mass*gravity >> box.get_link_mass(box.base)*env.gravity[-1] + force_control[1]
        # Ensure that the tangential force applied (force_control[0] which is along x-axis) does not exceed the friction cone (mu_table * f_z). f_z is (box_mass*g - z-axis force). -mu_table, since gravity is negative and these inequalities are x >= 0
        constraints.append({'type': 'ineq', 'fun': lambda force_control: -mu_table * (box.get_link_mass(box.base)*env.gravity[-1] + force_control[1]) - force_control[0]})
        constraints.append({'type': 'ineq', 'fun': lambda force_control: -mu_table * (box.get_link_mass(box.base)*env.gravity[-1] + force_control[1]) + force_control[0]})

        # This ensures that our point moves inwards to the box
        constraints.append({'type': 'ineq', 'fun': lambda force_control: -force_on_box_frame(force_control)[0] - 1})

        bounds = ((-20, 20), (-20, 20))
        res = scipy.optimize.minimize(cost, x0=(0.0, 0.0), method='SLSQP', bounds=bounds, constraints=constraints)
        fx, fz = res.x
        if not constant_force:
            force += np.array([fx, 0, fz])
        # print(res.x, force_on_box_frame(res.x))

        finger.apply_external_force(link=finger.base, force=force, pos=finger.get_base_pos_orient()[0], local_coordinate_frame=False)

        m.step_simulation(realtime=True)

direct_force_control(constant_force=False)
direct_force_control(constant_force=True)
