import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
m.visualize_coordinate_frame()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create 6 YCB objects
cheezit = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'cheezit.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[-0.5, 0, 0.85], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)
spam = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'spam.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[-0.3, 0, 0.8], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)
mustard = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'mustard.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[-0.1, 0, 0.85], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)
tomato_soup_can = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'tomato_soup_can.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[0.1, 0, 0.8], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)
bowl = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'bowl.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[0.3, 0, 0.75], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)
mug = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'mug.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[0.5, 0, 0.75], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)

m.step_simulation(steps=10000, realtime=True)
