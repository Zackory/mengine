import os
import numpy as np
import matplotlib.pyplot as plt
import mengine as m
np.set_printoptions(precision=2, suppress=True)

mu = 0.5 # Coefficient of friction
applied_pos = [0.3, 0, 0.075]

# Create environment and ground plane
env = m.Env()

def force_test(force_magnitude=1):
    # Reset environment and ground plane
    env.reset()
    ground = m.Ground()

    # Create table and cube
    table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
    table.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)
    wall = m.Shape(m.Box(half_extents=[0.05, 0.3, 0.3]), static=True, mass=0, position=[-0.5, 0, 0.8], orientation=[0, 0, 0, 1], rgba=[1, 1, 1, 1])
    cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])

    # Let the cube drop onto the table
    m.step_simulation(steps=20, realtime=False)

    # Create a sphere (finger) to collide with the cube
    sphere = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=cube.local_to_global_coordinate_frame(applied_pos)[0], rgba=[1, 0, 0, 1])
    sphere.set_whole_body_frictions(lateral_friction=0.5, spinning_friction=100, rolling_friction=100)

    positions = []
    for i in range(150):
        # Apply a force to the sphere
        force = -sphere.get_link_mass(sphere.base)*env.gravity # Gravity compensation force
        force += np.array([-abs(force_magnitude), 0, 0])
        sphere.apply_external_force(link=sphere.base, force=force, pos=sphere.get_base_pos_orient()[0], local_coordinate_frame=False)

        m.step_simulation(realtime=True)

        # Capture x position of sphere
        positions.append(sphere.get_base_pos_orient()[0][0])
    return positions

# Run simulation with varying table frictions
magnitudes = [0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
all_positions = []
for force_mag in magnitudes:
    positions = force_test(force_mag)
    all_positions.append(positions)

# Plot position curve of sphere (finger) for each table friction
for p, m in zip(all_positions, magnitudes):
    plt.plot(p, label='f^cd = %.1f' % m)
plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=4, fancybox=True, shadow=True)
plt.xlabel('time (s)')
plt.ylabel('x (m)')
plt.show()
