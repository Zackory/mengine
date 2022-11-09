from imp import lock_held
import os
import numpy as np
import mengine as m


# use the functions created in part1 of the homework to find a force closure grasp
from part1 import friction_cone_3d, contact_screw_3d, is_force_closure


# Create environment and ground plane
env = m.Env(time_step=0.005)
env.set_gui_camera(look_at_pos=[0, 0, 0.95])
orient = m.get_quaternion([np.pi, 0, 0])


def sample_spherical(npoints, ndim=3):
    # write your code here
    #
    #
    return points


def sample_cube(npoints, ndim=3):
    # write your code here
    #
    #

    return points


def find_force_closure_grasp(testobj, mu) -> tuple:

    # write your code here
    #
    #

    return contact_positions, contact_normals


# Reset simulation env


def reset(positions, table_friction=0.5, obj_mass=100, obj_friction=0.5, finger_mass=10.0, obj_type='sphere'):
    # Create environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and cube
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[
                   0, 0, 0], orientation=[0, 0, 0, 1], maximal_coordinates=True, scale=1.0)
    table.set_whole_body_frictions(
        lateral_friction=table_friction, spinning_friction=0, rolling_friction=0)
    if obj_type == 'sphere':
        obj = m.Shape(m.Sphere(radius=0.1), static=False, mass=obj_mass,
                      position=[0, 0, 1.2], rgba=[0, 1, 0, 1])
    elif obj_type == 'cube':
        obj = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=obj_mass, position=[
            0, 0, 1.2], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 1])
    else:
        raise TypeError(
            'Object Type unknown. Use either sphere or cube as obj.')

    obj.set_whole_body_frictions(
        lateral_friction=obj_friction, spinning_friction=0, rolling_friction=0)

    # Create n-spheres(fingers) to make contact with the cube.
    fingers = []
    for i, p in enumerate(positions):
        fingers.append(m.Shape(m.Sphere(radius=0.02), static=False, mass=finger_mass,
                               position=obj.local_to_global_coordinate_frame(positions[i])[0], rgba=[1, 0, 0, 1]))
        fingers[i].set_whole_body_frictions(
            lateral_friction=0.5, spinning_friction=100, rolling_friction=100)
    return fingers, obj


# visualize force closure grasps in simulation


def visualize_grasps(grasps):
    for g in grasps:
        # Reset simulator
        fingers, obj = reset(g['contact_positions'],  g['table_friction'],
                             g['obj_mass'], g['obj_friction'], g['finger_mass'])
        for i in range(100):

            for idx, finger in enumerate(fingers):

                # Gravity compensation force
                force = -finger.get_link_mass(finger.base)*env.gravity
                # Apply force in contact point in the direction of the contact normal
                force += np.array(g['force_magnitude'] *
                                  g['contact_normals'][idx])
                finger.apply_external_force(link=finger.base, force=force, pos=finger.get_base_pos_orient()[
                                            0], local_coordinate_frame=False)

            m.step_simulation()

            # Show contact normals
            cp = finger.get_contact_points(bodyB=obj)
            m.clear_all_visual_items()
            if cp is not None:
                for c in cp:
                    line = m.Line(c['posB'], np.array(c['posB']) +
                                  np.array(c['contact_normal'])*0.2, rgb=[1, 0, 0])


def main(testobj, friction=True):

    # parameters to play with
    obj_mass = 100
    obj_friction = 0.5
    finger_mass = 10.0
    force_magnitude = 1000
    if friction:
        mu = 0.5
    else:
        mu = 0.0

    contact_positions, contact_normals = find_force_closure_grasp(
        testobj, mu)

    # spawn fingers slightly away from the obj
    grasps = [dict(contact_positions=1.1*contact_positions, contact_normals=contact_normals,
                   table_friction=0.5, obj_type=testobj, obj_mass=obj_mass, obj_friction=obj_friction, force_magnitude=force_magnitude, finger_mass=finger_mass)]

    # append grasps with other parameters here:
    # grasps.append()

    visualize_grasps(grasps)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description='Parse different test objects.')
    parser.add_argument('testobject', type=str,

                        help='Call file either with argument \'sphere\' or \'cube\'.')
    parser.add_argument('friction', type=bool,

                        help='Frictionless => False , frictional=>True')
    try:
        args = parser.parse_args()
        testobj = args.testobj
        friction = args.friction
    except:
        print("No testobj specified. Using sphere as default.")
        testobj = 'sphere'
        friction = False
    main(testobj, friction)
