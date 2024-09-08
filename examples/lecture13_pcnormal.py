import os
import numpy as np
from sklearn.decomposition import PCA
import mengine as m

# NOTE: First install open3d using: 'python3 -m pip install open3d'
# On Mac you also need 'brew install libomp'

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create mustard bottle
mustard = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'mustard.obj'), scale=[1, 1, 1]), static=False, mass=1.0, position=[0, 0, 0.85], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)

# Let the object settle on the table
m.step_simulation(steps=100)

# Create a camera
camera = m.Camera(camera_pos=[0, -0.25, 1], look_at_pos=mustard.get_base_pos_orient()[0], fov=60, camera_width=1920//4, camera_height=1080//4)
camera_direction = mustard.get_base_pos_orient()[0] - np.array([0, -0.25, 1])
# Capture a point cloud from the camera
pc, rgba = camera.get_point_cloud(body=mustard)
# Hide the mustard bottle
mustard.change_visual(link=mustard.base, rgba=[1, 1, 1, 0])

# Consider a specific point in the cloud
point_index = 1500
# Find its neighbors with distance less than 0.005, paint green.
distances = np.linalg.norm(pc - pc[point_index], axis=-1)
indices = (distances < 0.005).nonzero()
rgba[indices] = [0, 1, 0, 1]
# Paint the point red
rgba[point_index] = [1, 0, 0, 1]

# Visualize the point cloud
m.DebugPoints(pc, points_rgb=rgba[:, :3], size=10)

points = pc[indices]

# Use PCA to find a best-fitting plane
pca = PCA(n_components=2)
pca.fit(points)
normal = np.cross(pca.components_[0], pca.components_[1])
# Flip normals to point at camera
normal *= 1 if camera_direction.dot(normal) > 0 else -1
# Plot the normal
m.Line(pc[point_index], pc[point_index] - normal*0.03, radius=0.0001, rgba=[0, 0, 1, 1])

# Use singular value decomposition to find a best-fitting plane
# points = points.T
# # Subtract out the centroid and take the SVD
# svd = np.linalg.svd(points - np.mean(points, axis=1, keepdims=True))
# # Extract the left singular vectors
# left = svd[0]
# normal = left[:, -1]
# normal = normal / np.linalg.norm(normal)
# # Flip normals to point at camera
# normal *= 1 if camera_direction.dot(normal) > 0 else -1
# # Plot the normal
# m.Line(pc[point_index], pc[point_index] + normal*0.02, radius=0.0001, rgba=[1, 0, 0, 1])

m.step_simulation(steps=10000, realtime=True)
