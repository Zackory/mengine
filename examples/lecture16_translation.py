import os
import numpy as np
import mengine as m
np.set_printoptions(precision=2, suppress=True)

mu = 0.5 # Coefficient of friction

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1.0, position=[0, 0, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
cube.set_whole_body_frictions(lateral_friction=mu, spinning_friction=0, rolling_friction=0)

# Let the cube drop onto the table
m.step_simulation(steps=50)

# Give the cube some velocity and let it begin slidding along the table
cube.set_base_velocity(linear_velocity=[-2.0, 0, 0], angular_velocity=[0, 0, 0])
m.step_simulation(steps=5)

# Now that the cube is moving, record the resultant normal and frictional forces between the cube and table
normal, friction_1, friction_2 = cube.get_resultant_contact_forces(bodyB=table)

# Compute Coulomb's law for translation of point contact!
velocity = cube.get_base_linear_velocity()
print('Estimate of friction:', -mu * velocity / np.linalg.norm(velocity) * np.linalg.norm(normal), '| Ground truth friction:', friction_2)

