import os
import mengine as m
import pybullet as p

env = m.Env()
ground = m.Ground()

sphere = m.Shape(m.Sphere(radius=0.1), static=True, mass=0.0, position=[0, 0, 0], orientation=[0, 0, 0, 1], rgba=[0, 1, 1, 1])
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

robot = m.Robot.Panda(position=[0, 0, 0.75])

while True:
    p.stepSimulation(physicsClientId=env.id)

