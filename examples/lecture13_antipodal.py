import os
import numpy as np
import open3d as o3d
import mengine as m

# NOTE: First install open3d using: 'python3 -m pip install open3d'
# On Mac you also need 'brew install libomp'

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create mustard bottle
mustard = m.Shape(m.Mesh(filename=os.path.join(m.directory, 'ycb', 'mustard.obj'), scale=[1, 1, 1]), static=True, mass=1.0, position=[0, 0, 0.85], orientation=[0, 0, 0, 1], rgba=None, visual=True, collision=True)
# mustard = m.Shape(m.Box([0.03]*3), static=True, position=[0, 0, 0.8], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 1])

# Create Panda manipulator
robot = m.Robot.Panda(position=[0.5, 0, 0.75])

# Move end effector to a starting position using IK
target_joint_angles = robot.ik(robot.end_effector, target_pos=[0, 0, 1], target_orient=m.get_quaternion(np.array([np.pi, 0, 0])))
robot.control(target_joint_angles, set_instantly=True)
robot.set_gripper_position([1]*2, set_instantly=True) # Open gripper

# Create a region that will allow us to identify points within the gripper
position, orientation = robot.get_link_pos_orient(robot.end_effector)
half_extents = np.array([0.01, 0.04, 0.01])
antipodal_region = m.Shape(m.Box(half_extents), static=True, collision=False, position=position, orientation=orientation, rgba=[0, 1, 0, 0])
gripper_line_vector = robot.local_to_global_coordinate_frame([0, 0.2, 0], link=robot.end_effector)[0]
# gripper_line = m.Line(position, gripper_line_vector, radius=0.005, rgba=[0, 0, 0, 1])

# Create two cameras
camera1 = m.Camera(camera_pos=[0, -0.25, 1], look_at_pos=mustard.get_base_pos_orient()[0], fov=60, camera_width=1920//4, camera_height=1080//4)
camera2 = m.Camera(camera_pos=[0, 0.25, 1], look_at_pos=mustard.get_base_pos_orient()[0], fov=60, camera_width=1920//4, camera_height=1080//4)
# Capture a point cloud from the camera
pc1, rgba1 = camera1.get_point_cloud(body=mustard)
pc2, rgba2 = camera2.get_point_cloud(body=mustard)
pc = np.concatenate([pc1, pc2], axis=0)
rgba = np.concatenate([rgba1, rgba2], axis=0)
# Visualize the point cloud
# m.DebugPoints(pc, points_rgb=rgba[:, :3], size=10)
# Hide the mustard bottle
mustard.change_visual(link=mustard.base, rgba=[1, 1, 1, 0.75])

# Create open3d point cloud from array of points
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pc)

# Estimate normals for each point
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
normals = np.asarray(pcd.normals)

# End effector pose
position, orientation = robot.get_link_pos_orient(robot.end_effector)
orientation = m.get_euler(orientation)

# Map keys to position and orientation end effector movements
pos_keys_actions = {'j': [-0.01, 0, 0], 'l': [0.01, 0, 0],
                    'u': [0, -0.01, 0], 'o': [0, 0.01, 0],
                    'k': [0, 0, -0.01], 'i': [0, 0, 0.01]}
rpy_keys_actions = {'k': [-0.05, 0, 0], 'i': [0.05, 0, 0],
                    'u': [0, -0.05, 0], 'o': [0, 0.05, 0],
                    'j': [0, 0, -0.05], 'l': [0, 0, 0.05]}

# Create a set points to visualize which points are in our gripper
visual_points = m.Points([[-10, -10, -10]]*1000, rgba=[0, 0, 0, 1], radius=0.001)

while True:
    keys = m.get_keys()
    # Process position movement keys ('u', 'i', 'o', 'j', 'k', 'l')
    for key, action in pos_keys_actions.items():
        if 'shift' not in keys and key in keys:
            position += action
    for key, action in rpy_keys_actions.items():
        if 'shift' in keys and key in keys:
            orientation += action
    # Move the end effector to the new pose
    target_joint_angles = robot.ik(robot.end_effector, target_pos=position, target_orient=m.get_quaternion(orientation), use_current_joint_angles=True)
    robot.control(target_joint_angles)

    # Update our antipodal region of interest
    p, o = robot.get_link_pos_orient(robot.end_effector)
    antipodal_region.set_base_pos_orient(p, o)

    # Get current gripper finger axis
    gripper_line_vector = robot.local_to_global_coordinate_frame([0, 0.2, 0], link=robot.end_effector)[0]
    gripper_line_vector = gripper_line_vector - p
    gripper_line_vector = gripper_line_vector / np.linalg.norm(gripper_line_vector)
    # gripper_line = m.Line(p, p+gripper_line_vector*0.2, radius=0.005, rgba=[0, 0, 0, 1], replace_line=gripper_line)

    # Find points that are inside our antipodal_region
    # Transform points to antipodal_region frame
    points = np.array([antipodal_region.global_to_local_coordinate_frame(p)[0] for p in pc])
    # Check if within bounding box of antipodal_region
    left_bound = np.all(points > -half_extents, axis=-1)
    right_bound = np.all(points < half_extents, axis=-1)
    indices = np.logical_and(left_bound, right_bound)
    points_in_gripper = pc[indices]
    normals_in_gripper = normals[indices]

    # Visualize points in gripper
    limit = min(len(points_in_gripper), len(visual_points))
    for i in range(limit):
        visual_points[i].set_base_pos_orient(points_in_gripper[i])
    for i in range(limit, len(visual_points)):
        visual_points[i].set_base_pos_orient([-10, -10, -10])
    # m.clear_all_debug_items()
    # print(np.shape(points_in_gripper))
    # if points_in_gripper:
    #     m.DebugPoints(points_in_gripper, points_rgb=[0, 0, 0], size=10)

    # Compute grasp score
    if len(normals_in_gripper) > 0:
        score = np.mean(np.abs(normals_in_gripper.dot(gripper_line_vector)))
        print('Grasp score:', score)

    m.step_simulation(realtime=True)

