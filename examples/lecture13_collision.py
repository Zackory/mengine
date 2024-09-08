import os, time
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env(gravity=[0, 0, 0])
ground = m.Ground()

cube_static = m.Shape(m.Box(half_extents=[0.2]*3), static=True, position=[-0.2, 0, 1], orientation=[0, 0, 0, 1], rgba=[1, 1, 1, 0.5])
cube = m.Shape(m.Box(half_extents=[0.2]*3), static=False, mass=1, position=[0.3, 0.1, 0.65], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.5])
m.Line([0, -0.2, 0.8], [-0.4, -0.2, 1.2], radius=0.005, rgba=[0, 0, 0, 0.5])

lines = []
for i in range(1000):
    pos, orient = cube.get_base_pos_orient()
    cube.set_base_pos_orient(pos=pos + np.array([-0.002, 0, 0]))
    # m.redraw()

    # m.step_simulation()

    m.compute_collision_detection()

    c = cube.get_contact_points(bodyB=None, linkA=None, linkB=None, average=False)
    new_lines = []
    if c is not None:
        for cp in c:
            new_lines.append(m.Line(cp['posA'], cp['posA'] + np.array(cp['contact_normal'])*0.2, radius=0.005, rgba=[1, 0, 0, 0.75], replace_line=(lines.pop() if len(lines) > 0 else None)))
            # m.Shape(m.Sphere(radius=0.01), static=True, position=cp['posA'], rgba=[1, 0, 0, 0.75])
            # m.Line(cp['posB'], cp['posB'] + np.array(cp['contact_normal'])*0.2, radius=0.005, rgba=[0, 0, 1, 0.75])
            # m.Shape(m.Sphere(radius=0.01), static=True, position=cp['posB'], rgba=[0, 0, 1, 0.75])
    lines = new_lines
    m.redraw()
    time.sleep(0.1)
    # env.slow_time()

while True:
    time.sleep(0.01)
    m.redraw()

# time.sleep(10)
# m.step_simulation(steps=10000, realtime=True)

