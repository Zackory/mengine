import os
import numpy as np
import mengine as m
np.set_printoptions(precision=4, suppress=True)

# Create environment and ground plane
env = m.Env()
# ground = m.Ground()
#
# # Create table and cube
# table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
# table.set_restitution(1)
#
# # Create a sphere
# ball = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=[0, 0, 1.5], rgba=[1, 0, 0, 1])
# ball.set_whole_body_frictions(lateral_friction=0, spinning_friction=0, rolling_friction=0)
# ball.set_restitution(0.5)
#
# m.step_simulation(steps=100)



# Now try giving the ball some tangential velocity
env.reset()
ground = m.Ground()

# Create table and cube
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])
table.set_restitution(1)

# Create a sphere
ball = m.Shape(m.Sphere(radius=0.02), static=False, mass=1.0, position=[0.5, 0, 1.5], rgba=[1, 0, 0, 1])
ball.set_whole_body_frictions(lateral_friction=0, spinning_friction=0, rolling_friction=0)
ball.set_restitution(0.5)
ball.set_base_velocity([-0.5, 0, 0])

m.step_simulation(steps=100)


