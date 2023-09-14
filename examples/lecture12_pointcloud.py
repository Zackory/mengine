import os
import numpy as np
import mengine as m

# Create environment and ground plane
env = m.Env()
ground = m.Ground()
m.visualize_coordinate_frame()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create mustard bottle
mustard = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'mustard.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[0, 0, 0.85], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)

# Let the object settle on the table
m.step_simulation(steps=100)

# Create a camera
camera = m.Camera(camera_pos=[0, -0.25, 1], look_at_pos=mustard.get_base_pos_orient()[0], fov=60, camera_width=1920//4, camera_height=1080//4)
# Capture a point cloud from the camera
pc, rgba = camera.get_point_cloud(body=mustard)
# Visualize the point cloud
m.DebugPoints(pc, points_rgb=rgba[:, :3], size=10)
# m.Points(pc, rgba=[0, 0, 0, 1], radius=0.001)
# m.DebugPoints(pc, points_rgb=[0, 0, 0], size=10)

# Hide the mustard bottle
mustard.change_visual(link=mustard.base, rgba=[1, 1, 1, 0])

# Create a cube representing the camera
# m.Shape(m.Box(half_extents=[0.05]*3), static=True, position=[0, -0.25, 1], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 0.75])

m.step_simulation(steps=10000, realtime=True)
