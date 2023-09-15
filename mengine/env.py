import os, time
import numpy as np
from scipy.spatial.transform import Rotation as R
from screeninfo import get_monitors
import pybullet as p

from .util import Util
from .bodies.body import Body
from .bodies.panda import Panda

envir = None
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'assets')

class Env:
    def __init__(self, time_step=0.02, gravity=[0, 0, -9.81], render=True, gpu_rendering=False, seed=1001, deformable=False):
        global envir
        envir = self
        self.time_step = time_step
        self.gravity = np.array(gravity)
        self.id = None
        self.render = render
        self.gpu_rendering = gpu_rendering
        self.view_matrix = None
        self.deformable = deformable
        self.seed(seed)
        self.directory = directory
        self.visual_items = []
        if self.render:
            try:
                self.width = get_monitors()[0].width
                self.height = get_monitors()[0].height
            except Exception as e:
                self.width = 1920
                self.height = 1080
            self.id = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0 --width=%d --height=%d' % (self.width, self.height))
        else:
            self.id = p.connect(p.DIRECT)
        self.util = Util(self.id)

        self.reset()

    def seed(self, seed=1001):
        np.random.seed(seed)
        self.np_random = None

    def disconnect(self):
        p.disconnect(self.id)

    def reset(self):
        if self.deformable:
            p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD, physicsClientId=self.id)
        else:
            p.resetSimulation(physicsClientId=self.id)
        if self.gpu_rendering:
            self.util.enable_gpu()
        self.set_gui_camera()
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=self.id)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, lightPosition=[0, 5, 10], physicsClientId=self.id)
        p.setTimeStep(self.time_step, physicsClientId=self.id)
        # Disable real time simulation so that the simulation only advances when we call stepSimulation
        p.setRealTimeSimulation(0, physicsClientId=self.id)
        p.setGravity(self.gravity[0], self.gravity[1], self.gravity[2], physicsClientId=self.id)
        self.last_sim_time = time.time()

    def set_gui_camera(self, look_at_pos=[0, 0, 0.75], distance=1, yaw=0, pitch=-30):
        p.resetDebugVisualizerCamera(cameraDistance=distance, cameraYaw=yaw, cameraPitch=pitch, cameraTargetPosition=look_at_pos, physicsClientId=self.id)

    def slow_time(self):
        # Slow down time so that the simulation matches real time
        t = time.time() - self.last_sim_time
        if t < self.time_step:
            time.sleep(self.time_step - t)
        self.last_sim_time = time.time()

def step_simulation(steps=1, realtime=True, env=None):
    env = env if env is not None else envir
    for _ in range(steps):
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=env.id) # Enable rendering
        p.stepSimulation(physicsClientId=env.id)
        # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=env.id) # Disable rendering, this allows us to create and delete objects without object flashing
        if realtime and env.render:
            env.slow_time()

def compute_collision_detection(env=None):
    env = env if env is not None else envir
    p.performCollisionDetection(physicsClientId=env.id)

def redraw(env=None):
    env = env if env is not None else envir
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=env.id)

def get_keys():
    specials = {p.B3G_ALT: 'alt', p.B3G_SHIFT: 'shift', p.B3G_CONTROL: 'control', p.B3G_RETURN: 'return', p.B3G_LEFT_ARROW: 'left_arrow', p.B3G_RIGHT_ARROW: 'right_arrow', p.B3G_UP_ARROW: 'up_arrow', p.B3G_DOWN_ARROW: 'down_arrow'}
    # return {chr(k) if k not in specials else specials[k] : v for k, v in p.getKeyboardEvents().items()}
    return [chr(k) if k not in specials else specials[k] for k in p.getKeyboardEvents().keys()]

class Robot:
    class Panda(Panda):
        def __init__(self, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True, env=None):
            env = env if env is not None else envir
            super().__init__(env, position, get_quaternion(orientation), controllable_joints, fixed_base)

class Camera:
    def __init__(self, camera_pos=[0.5, -0.5, 1.5], look_at_pos=[0, 0, 0.75], fov=60, camera_width=1920//4, camera_height=1080//4, env=None):
        self.env = env if env is not None else envir
        self.fov = fov
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.view_matrix = p.computeViewMatrix(camera_pos, look_at_pos, [0, 0, 1], physicsClientId=self.env.id)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.camera_width / self.camera_height, 0.01, 100, physicsClientId=self.env.id)

    def set_camera_rpy(self, look_at_pos=[0, 0, 0.75], distance=1.5, rpy=[0, -35, 40]):
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(look_at_pos, distance, rpy[2], rpy[1], rpy[0], 2, physicsClientId=self.env.id)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.camera_width / self.camera_height, 0.01, 100, physicsClientId=self.env.id)

    def get_rgba_depth(self, light_pos=[0, -3, 1], shadow=False, ambient=0.8, diffuse=0.3, specular=0.1):
        w, h, img, depth, segmentation_mask = p.getCameraImage(self.camera_width, self.camera_height, self.view_matrix, self.projection_matrix, lightDirection=light_pos, shadow=shadow, lightAmbientCoeff=ambient, lightDiffuseCoeff=diffuse, lightSpecularCoeff=specular, renderer=p.ER_BULLET_HARDWARE_OPENGL, physicsClientId=self.env.id)
        img = np.reshape(img, (h, w, 4))
        depth = np.reshape(depth, (h, w))
        segmentation_mask = np.reshape(segmentation_mask, (h, w))
        return img, depth, segmentation_mask

    def get_point_cloud(self, body=None):
        # get a depth image
        rgba, depth, segmentation_mask = self.get_rgba_depth()
        rgba = rgba.reshape((-1, 4))
        depth = depth.flatten()
        segmentation_mask = segmentation_mask.flatten()

        # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
        proj_matrix = np.asarray(self.projection_matrix).reshape([4, 4], order="F")
        view_matrix = np.asarray(self.view_matrix).reshape([4, 4], order="F")
        tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

        # create a grid with pixel coordinates and depth values
        y, x = np.mgrid[-1:1:2 / self.camera_height, -1:1:2 / self.camera_width]
        y *= -1.
        x, y, z = x.reshape(-1), y.reshape(-1), depth
        h = np.ones_like(z)

        pixels = np.stack([x, y, z, h], axis=1)

        # Filter point cloud to only include points on the target body
        pixels = pixels[segmentation_mask == body.body]
        z = z[segmentation_mask == body.body]
        rgba = rgba[segmentation_mask == body.body]

        # filter out "infinite" depths
        pixels = pixels[z < 0.99]
        rgba = rgba[z < 0.99]
        pixels[:, 2] = 2 * pixels[:, 2] - 1

        # turn pixels to world coordinates
        points = np.matmul(tran_pix_world, pixels.T).T
        points /= points[:, 3: 4]
        points = points[:, :3]

        return points, rgba/255

def get_euler(quaternion, env=None):
    env = env if env is not None else envir
    return np.array(quaternion) if len(quaternion) == 3 else np.array(p.getEulerFromQuaternion(np.array(quaternion), physicsClientId=env.id))

def get_quaternion(euler, env=None):
    env = env if env is not None else envir
    return R.from_matrix(euler).as_quat() if np.array(euler).ndim > 1 else (np.array(euler) if len(euler) == 4 else np.array(p.getQuaternionFromEuler(np.array(euler), physicsClientId=env.id)))

def get_rotation_matrix(quaternion, env=None):
    env = env if env is not None else envir
    return np.array(p.getMatrixFromQuaternion(get_quaternion(quaternion), physicsClientId=env.id)).reshape((3,3))

def get_axis_angle(quaternion, env=None):
    env = env if env is not None else envir
    q = get_quaternion(quaternion)
    sqrt = np.sqrt(1-q[-1]**2)
    return np.array([q[0]/sqrt, q[1]/sqrt, q[2]/sqrt]), 2*np.arccos(q[-1])

def get_difference_quaternion(q1, q2, env=None):
    env = env if env is not None else envir
    return p.getDifferenceQuaternion(get_quaternion(q1), get_quaternion(q2), physicsClientId=env.id)

def quaternion_product(q1, q2, env=None):
    env = env if env is not None else envir
    # Return Hamilton product of 2 quaternions
    return p.multiplyTransforms([0, 0, 0], get_quaternion(q1), [0, 0, 0], q2, physicsClientId=env.id)[1]

def multiply_transforms(p1, q1, p2, q2, env=None):
    env = env if env is not None else envir
    return p.multiplyTransforms(p1, get_quaternion(q1), p2, get_quaternion(q2), physicsClientId=env.id)

def rotate_point(point, quaternion, env=None):
    env = env if env is not None else envir
    return p.multiplyTransforms([0, 0, 0], get_quaternion(quaternion), point, [0, 0, 0, 1], physicsClientId=env.id)[0]


class Obj:
    def __init__(self, type=None, radius=0, half_extents=[0, 0, 0], length=0, normal=[0, 0, 1], filename='', scale=[1, 1, 1]):
        self.type = type
        self.radius = radius
        self.half_extents = half_extents
        self.length = length
        self.normal = normal
        self.filename = filename
        self.scale = scale

class Sphere(Obj):
    def __init__(self, radius=1):
        super().__init__(type=p.GEOM_SPHERE, radius=radius)

class Box(Obj):
    def __init__(self, half_extents=[1, 1, 1]):
        super().__init__(type=p.GEOM_BOX, half_extents=half_extents)

class Capsule(Obj):
    def __init__(self, radius=1, length=1):
        super().__init__(type=p.GEOM_CAPSULE, radius=radius, length=length)

class Cylinder(Obj):
    def __init__(self, radius=1, length=1):
        super().__init__(type=p.GEOM_CYLINDER, radius=radius, length=length)

class Plane(Obj):
    def __init__(self, normal=[0, 0, 1]):
        super().__init__(type=p.GEOM_PLANE, normal=normal)

class Mesh(Obj):
    def __init__(self, filename='', scale=[1, 1, 1]):
        super().__init__(type=p.GEOM_MESH, filename=filename, scale=scale)

def Shape(shape, static=False, mass=1.0, position=[0, 0, 0], orientation=[0, 0, 0, 1], visual=True, collision=True, rgba=[0, 1, 1, 1], maximal_coordinates=False, return_collision_visual=False, position_offset=[0, 0, 0], orientation_offset=[0, 0, 0, 1], env=None):
    env = env if env is not None else envir
    collision = p.createCollisionShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, height=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, collisionFramePosition=position_offset, collisionFrameOrientation=orientation_offset, physicsClientId=env.id) if collision else -1
    if rgba is not None:
        visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, rgbaColor=rgba, visualFramePosition=position_offset, visualFrameOrientation=orientation_offset, physicsClientId=env.id) if visual else -1
    else:
        visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, visualFramePosition=position_offset, visualFrameOrientation=orientation_offset, physicsClientId=env.id) if visual else -1
    if return_collision_visual:
        return collision, visual
    body = p.createMultiBody(baseMass=0 if static else mass, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=position, baseOrientation=get_quaternion(orientation), useMaximalCoordinates=maximal_coordinates, physicsClientId=env.id)
    return Body(body, env, collision_shape=collision, visual_shape=visual)

def Shapes(shape, static=False, mass=1.0, positions=[[0, 0, 0]], orientation=[0, 0, 0, 1], visual=True, collision=True, rgba=[0, 1, 1, 1], maximal_coordinates=False, return_collision_visual=False, position_offset=[0, 0, 0], orientation_offset=[0, 0, 0, 1], env=None):
    env = env if env is not None else envir
    collision = p.createCollisionShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, height=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, collisionFramePosition=position_offset, collisionFrameOrientation=orientation_offset, physicsClientId=env.id) if collision else -1
    if rgba is not None:
        visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, rgbaColor=rgba, visualFramePosition=position_offset, visualFrameOrientation=orientation_offset, physicsClientId=env.id) if visual else -1
    else:
        visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, visualFramePosition=position_offset, visualFrameOrientation=orientation_offset, physicsClientId=env.id) if visual else -1
    if return_collision_visual:
        return collision, visual
    shape_ids = p.createMultiBody(baseMass=0 if static else mass, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=positions[0], baseOrientation=get_quaternion(orientation), batchPositions=positions, useMaximalCoordinates=maximal_coordinates, physicsClientId=env.id)
    shapes = []
    for body in shape_ids:
        shapes.append(Body(body, env, collision_shape=collision, visual_shape=visual))
    return shapes

def URDF(filename, static=False, position=[0, 0, 0], orientation=[0, 0, 0, 1], maximal_coordinates=False, env=None):
    env = env if env is not None else envir
    body = p.loadURDF(filename, basePosition=position, baseOrientation=get_quaternion(orientation), useMaximalCoordinates=maximal_coordinates, useFixedBase=static, physicsClientId=env.id)
    return Body(body, env)

def Ground(position=[0, 0, 0], orientation=[0, 0, 0, 1], env=None):
    env = env if env is not None else envir
    return URDF(filename=os.path.join(directory, 'plane', 'plane.urdf'), static=True, position=position, orientation=get_quaternion(orientation), env=env)
    # Randomly set friction of the ground
    # self.ground.set_frictions(self.ground.base, lateral_friction=self.np_random.uniform(0.025, 0.5), spinning_friction=0, rolling_friction=0)

def Line(start, end, radius=0.005, rgba=None, rgb=[1, 0, 0], replace_line=None, env=None):
    env = env if env is not None else envir
    if rgba is None:
        rgba = list(rgb) + [1]
    # line = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=end, lineColorRGB=rgba[:-1], lineWidth=1, lifeTime=0, physicsClientId=env.id)
    v1 = np.array([0, 0, 1])
    v2 = np.array(end) - start
    orientation = np.cross(v1, v2).tolist() + [np.sqrt((np.linalg.norm(v1)**2) * (np.linalg.norm(v2)**2)) + np.dot(v1, v2)]
    orientation = [0, 0, 0, 1] if np.linalg.norm(orientation) == 0 else orientation / np.linalg.norm(orientation)
    if replace_line is not None:
        replace_line.set_base_pos_orient(start + (np.array(end)-start)/2, orientation)
        return replace_line
    else:
        l = Shape(Cylinder(radius=radius, length=np.linalg.norm(np.array(end)-start)), static=True, position=start + (np.array(end)-start)/2, orientation=orientation, collision=False, rgba=rgba)
        env.visual_items.append(l)
        return l

def Points(point_positions, rgba=[1, 0, 0, 1], radius=0.01, replace_points=None, env=None):
    env = env if env is not None else envir
    if type(point_positions[0]) not in (list, tuple, np.ndarray):
        point_positions = [point_positions]
    if replace_points is not None:
        for i in range(min(len(point_positions), len(replace_points))):
            replace_points[i].set_base_pos_orient(point_positions[i])
            return replace_points
    else:
        points = Shapes(Sphere(radius=radius), static=True, positions=point_positions, orientation=[0, 0, 0, 1], visual=True, collision=False, rgba=rgba)
        return points

def DebugPoints(point_positions, points_rgb=[[0, 0, 0, 1]], size=10, replace_points=None, env=None):
    env = env if env is not None else envir
    if type(point_positions[0]) not in (list, tuple, np.ndarray):
        point_positions = [point_positions]
    if type(points_rgb[0]) not in (list, tuple, np.ndarray):
        points_rgb = [points_rgb]*len(point_positions)
    for i in range(len(point_positions)//4000 + 1):
        points = -1
        while points < 0:
            if replace_points is None:
                points = p.addUserDebugPoints(pointPositions=point_positions[i*4000:(i+1)*4000], pointColorsRGB=points_rgb[i*4000:(i+1)*4000], pointSize=size, lifeTime=0, physicsClientId=env.id)
            else:
                points = p.addUserDebugPoints(pointPositions=point_positions[i*4000:(i+1)*4000], pointColorsRGB=points_rgb[i*4000:(i+1)*4000], pointSize=size, lifeTime=0, replaceItemUniqueId=replace_points, physicsClientId=env.id)
    return points

def visualize_coordinate_frame(position=[0, 0, 0], orientation=[0, 0, 0, 1], alpha=1.0, replace_old_cf=None, env=None):
    env = env if env is not None else envir
    transform = lambda pos: p.multiplyTransforms(position, get_quaternion(orientation), pos, [0, 0, 0, 1], physicsClientId=env.id)[0]
    x = Line(start=transform([0, 0, 0]), end=transform([0.2, 0, 0]), rgba=[1, 0, 0, alpha], replace_line=None if replace_old_cf is None else replace_old_cf[0])
    y = Line(start=transform([0, 0, 0]), end=transform([0, 0.2, 0]), rgba=[0, 1, 0, alpha], replace_line=None if replace_old_cf is None else replace_old_cf[1])
    z = Line(start=transform([0, 0, 0]), end=transform([0, 0, 0.2]), rgba=[0, 0, 1, alpha], replace_line=None if replace_old_cf is None else replace_old_cf[2])
    return x, y, z

def clear_visual_item(items, env=None):
    if items is None:
        return
    env = env if env is not None else envir
    if type(items) in (list, tuple):
        for item in items:
            p.removeBody(item.body, env.id)
            # p.removeUserDebugItem(item, physicsClientId=env.id)
            for i in range(len(env.visual_items)):
                if env.visual_items[i] == item:
                    del env.visual_items[i]
                    break
    else:
        p.removeBody(items.body, env.id)
        # p.removeUserDebugItem(items, physicsClientId=env.id)
        for i in range(len(env.visual_items)):
            if env.visual_items[i] == items:
                del env.visual_items[i]
                break

def clear_all_visual_items(env=None):
    env = env if env is not None else envir
    for item in env.visual_items:
        p.removeBody(item.body, env.id)
    env.visual_items = []
    # p.removeAllUserDebugItems(physicsClientId=env.id)

def clear_all_debug_items(env=None):
    env = env if env is not None else envir
    p.removeAllUserDebugItems(physicsClientId=env.id)

def salisbury_hand(finger_length=0.075, env=None):
    env = env if env is not None else envir

    length = finger_length

    link_c1, link_v1 = Shape(Box(half_extents=[0.1, 0.05, 0.01]), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, 0])
    link_c2, link_v2 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])
    link_c3, link_v3 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])
    link_c4, link_v4 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])

    link_c5, link_v5 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])
    link_c6, link_v6 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])
    link_c7, link_v7 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])

    link_c8, link_v8 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])
    link_c9, link_v9 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])
    link_c10, link_v10 = Shape(Capsule(radius=0.01, length=length), visual=True, collision=True, rgba=[1, 1, 1, 1], return_collision_visual=True, position_offset=[0, 0, length/2])

    link_p1, link_o1 = [0, 0, 0], get_quaternion([0, 0, 0])
    link_p2, link_o2 = [-0.08, 0.02, 0.01], get_quaternion([0, 0, 0])
    link_p3, link_o3 = [0, 0, length], get_quaternion([0, 0, np.pi])
    link_p4, link_o4 = [0, 0, length], get_quaternion([0, 0, 0])

    link_p5, link_o5 = [0, -0.02, 0.01], get_quaternion([0, 0, 0])
    link_p8, link_o8 = [0.08, 0.02, 0.01], get_quaternion([0, 0, 0])

    linkMasses = [1]*9
    linkCollisionShapeIndices = [link_c2, link_c3, link_c4, link_c5, link_c6, link_c7, link_c8, link_c9, link_c10]
    linkVisualShapeIndices = [link_v2, link_v3, link_v4, link_v5, link_v6, link_v7, link_v8, link_v9, link_v10]
    linkPositions = [link_p2, link_p3, link_p4, link_p5, link_p3, link_p4, link_p8, link_p3, link_p4]
    linkOrientations = [link_o2, link_o3, link_o4, link_o5, link_o3, link_o4, link_o8, link_o3, link_o4]
    linkInertialFramePositions = [[0, 0, 0]]*9
    linkInertialFrameOrientations = [[0, 0, 0, 1]]*9
    linkParentIndices = [0, 1, 2, 0, 4, 5, 0, 7, 8]
    linkJointTypes = [p.JOINT_REVOLUTE]*9
    linkJointAxis =[[0, 0, 1], [1, 0, 0], [1, 0, 0]]*3

    multibody = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=link_c1, baseVisualShapeIndex=link_v1, basePosition=link_p1, baseOrientation=link_o1, linkMasses=linkMasses, linkCollisionShapeIndices=linkCollisionShapeIndices, linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions, linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions, linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=linkParentIndices, linkJointTypes=linkJointTypes, linkJointAxis=linkJointAxis, physicsClientId=env.id)
    body = Body(multibody, env)
    body.controllable_joints = list(range(9))
    return body
