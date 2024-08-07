import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

env.set_gui_camera(distance=2.5)

# Create robots
robot = m.Robot.Baxter(position=[-2, 0, 0.92], orientation=[0, 0, -np.pi/2])

robot2 = m.Robot.PR2(position=[-0.5, 0, 0], orientation=[0, 0, -np.pi/2])

robot3 = m.Robot.Stretch(position=[0.5, 0, 0], orientation=[0, 0, -np.pi/2])

robot4 = m.Robot.Panda(position=[1.5, 0, 0])

robot5 = m.Robot.Jaco(position=[2, 0, 0])

m.step_simulation(steps=1000, realtime=True)


