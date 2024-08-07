import os
import numpy as np
import mengine as m
import matplotlib.pyplot as plt
np.set_printoptions(precision=3, suppress=True)

# Create environment and ground plane
env = m.Env()
ground = m.Ground()

env.set_gui_camera(distance=2, yaw=30)

# Create table
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0], orientation=[0, 0, 0, 1])

# Create robots
panda = m.Robot.Panda(position=[0.5, 0, 0.75])
panda.enable_force_torque_sensor(8) # Create a F/T sensor at the wrist
# panda.print_joint_info()

stretch = m.Robot.Stretch(position=[0.25, -1, 0], orientation=[0, 0, np.pi/2])
# stretch.print_joint_info()
# Rotate the camera
stretch.set_joint_angles(angles=[-np.pi/8], joints=[21])
# stretch.set_joint_angles(angles=[-np.pi/4], joints=[20])

# Create a cube to make contact with
cube = m.Shape(m.Box(half_extents=[0.1]*3), static=False, mass=1, position=[-0.25, 0, 0.85], orientation=[0, 0, 0, 1], rgba=[0, 1, 0, 1])

for i in range(50):
    # Move the end effector into a cube and print sensor values
    position, orientation = panda.get_link_pos_orient(panda.end_effector)
    target_joint_angles = panda.ik(panda.end_effector, target_pos=position+[-0.08, 0, 0], target_orient=orientation, use_current_joint_angles=True)
    panda.control(target_joint_angles)

    m.step_simulation(realtime=True)

    if i % 20 == 0:
        # Print joint position, velocities, and torques
        motor_indices, motor_positions, motor_velocities, motor_torques = panda.get_motor_joint_states()
        print('Joint angles:', np.array(motor_positions))
        print('Joint velocities:', np.array(motor_velocities))
        print('Joint torques:', np.array(motor_torques))
        print('Force-torque:', panda.get_force_torque_sensor(8))
        linkA, linkB, posA, posB, contact_distance = panda.get_closest_points(bodyB=cube, linkA=8)
        print('Proximity (shortest distance from end effector to cube):', contact_distance[0])
        print('')

# Get the position and orientation for the camera_color_optical_joint
head_pos, head_orient = stretch.get_link_pos_orient(30)
# Move link slightly out of the mesh
head_pos, head_orient = m.multiply_transforms(head_pos, head_orient, [0.01, 0, 0], [0, 0, 0])
# m.visualize_coordinate_frame(head_pos, head_orient)

# Create a virtual camera for the Stretch's RealSense
camera = m.Camera(fov=60, camera_width=1920//4, camera_height=1080//4)
rpy = m.get_euler(head_orient)
# Align axes between head and camera
rpy = [rpy[0]-np.pi/2, rpy[1], rpy[2]+np.pi/2]
camera.set_camera_rpy(look_at_pos=head_pos, distance=-0.001, rpy=np.degrees(rpy))

img, depth, segmentation_mask = camera.get_rgba_depth()

plt.figure(); plt.title('RGB'); plt.imshow(img)
plt.figure(); plt.title('Depth'); plt.imshow(depth)
plt.figure(); plt.title('Segmentation Mask'); plt.imshow(segmentation_mask)
plt.show()

# Capture a point cloud from the camera
pc, rgba = camera.get_point_cloud(body=panda)
# pc, rgba = camera.get_point_cloud()
# Visualize the point cloud
m.DebugPoints(pc, points_rgb=rgba[:, :3], size=10)
# Hide the panda to see the visualized point cloud
panda.set_base_pos_orient([0.5, 0, 100.75])

m.step_simulation(steps=1000, realtime=True)

