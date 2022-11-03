import os, time
import numpy as np
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
        p.stepSimulation(physicsClientId=env.id)
        if realtime and env.render:
            env.slow_time()


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
        self.view_matrix = p.computeViewMatrix(camera_pos, look_at_pos, [0, 0, 1], physicsClientId=env.id)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.camera_width / self.camera_height, 0.01, 100, physicsClientId=env.id)

    def set_camera_rpy(self, look_at_pos=[0, 0, 0.75], distance=1.5, rpy=[0, -35, 40]):
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(look_at_pos, distance, rpy[2], rpy[1], rpy[0], 2, physicsClientId=env.id)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.camera_width / self.camera_height, 0.01, 100, physicsClientId=env.id)

    def get_rgba_depth(self, light_pos=[0, -3, 1], shadow=False, ambient=0.8, diffuse=0.3, specular=0.1):
        w, h, img, depth, _ = p.getCameraImage(self.camera_width, self.camera_height, self.view_matrix, self.projection_matrix, lightDirection=light_pos, shadow=shadow, lightAmbientCoeff=ambient, lightDiffuseCoeff=diffuse, lightSpecularCoeff=specular, renderer=p.ER_BULLET_HARDWARE_OPENGL, physicsClientId=env.id)
        img = np.reshape(img, (h, w, 4))
        depth = np.reshape(depth, (h, w))
        return img, depth

def get_rgba_depth(light_pos=[0, -3, 1], shadow=False, ambient=0.8, diffuse=0.3, specular=0.1, env=None):
    env = env if env is not None else envir
    return env.camera.get_rgba_depth(light_pos=light_pos, shadow=shadow, ambient=ambient, diffuse=diffuse, specular=specular)

def get_euler(quaternion, env=None):
    env = env if env is not None else envir
    return np.array(quaternion) if len(quaternion) == 3 else np.array(p.getEulerFromQuaternion(np.array(quaternion), physicsClientId=env.id))

def get_quaternion(euler, env=None):
    env = env if env is not None else envir
    return np.array(euler) if len(euler) == 4 else np.array(p.getQuaternionFromEuler(np.array(euler), physicsClientId=env.id))


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

def Shape(shape, static=False, mass=1.0, position=[0, 0, 0], orientation=[0, 0, 0, 1], visual=True, collision=True, rgba=[0, 1, 1, 1], maximal_coordinates=False, return_collision_visual=False, env=None):
    env = env if env is not None else envir
    collision = p.createCollisionShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, height=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, physicsClientId=env.id) if collision else -1
    visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, rgbaColor=rgba, physicsClientId=env.id) if visual else -1
    if return_collision_visual:
        return collision, visual
    body = p.createMultiBody(baseMass=0 if static else mass, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=position, baseOrientation=get_quaternion(orientation), useMaximalCoordinates=maximal_coordinates, physicsClientId=env.id)
    return Body(body, env)

def Shapes(shape, static=False, mass=1.0, positions=[[0, 0, 0]], orientation=[0, 0, 0, 1], visual=True, collision=True, rgba=[0, 1, 1, 1], maximal_coordinates=False, return_collision_visual=False, env=None):
    env = env if env is not None else envir
    collision = p.createCollisionShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, height=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, physicsClientId=env.id) if collision else -1
    visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, rgbaColor=rgba, physicsClientId=env.id) if visual else -1
    if return_collision_visual:
        return collision, visual
    shape_ids = p.createMultiBody(baseMass=0 if static else mass, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=positions[0], baseOrientation=get_quaternion(orientation), batchPositions=positions, useMaximalCoordinates=maximal_coordinates, physicsClientId=env.id)
    shapes = []
    for body in shape_ids:
        shapes.append(Body(body, env))
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


def Line(start, end, rgb=[1, 0, 0], width=0.01, replace_line=None, env=None):
    env = env if env is not None else envir
    line = -1
    while line < 0:
        if replace_line is None:
            line = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=end, lineColorRGB=rgb, lineWidth=width, lifeTime=0, physicsClientId=env.id)
        else:
            line = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=end, lineColorRGB=rgb, lineWidth=width, lifeTime=0, replaceItemUniqueId=replace_line, physicsClientId=env.id)
    return line

def Points(point_positions, points_rgb=[1, 0, 0], size=0.5, replace_points=None, env=None):
    env = env if env is not None else envir
    if type(point_positions[0]) not in (list, tuple):
        point_positions = [point_positions]
    if type(points_rgb[0]) not in (list, tuple):
        points_rgb = [points_rgb]*len(point_positions)
    points = -1
    while points < 0:
        if replace_points is None:
            points = p.addUserDebugPoints(pointPositions=point_positions, pointColorsRGB=points_rgb, pointSize=size, lifeTime=0, physicsClientId=env.id)
        else:
            points = p.addUserDebugPoints(pointPositions=point_positions, pointColorsRGB=points_rgb, pointSize=size, lifeTime=0, replaceItemUniqueId=replace_points, physicsClientId=env.id)
    return points

def visualize_coordinate_frame(position=[0, 0, 0], orientation=[0, 0, 0, 1], replace_old_cf=None, env=None):
    env = env if env is not None else envir
    if position[-1] == 0:
        position[-1] = 0.01
    # if replace_old_cf is not None:
    #     clear_visual_item(replace_old_cf)
    transform = lambda pos: p.multiplyTransforms(position, get_quaternion(orientation), pos, [0, 0, 0, 1], physicsClientId=env.id)[0]
    x = Line(start=transform([0, 0, 0]), end=transform([0.2, 0, 0]), rgb=[1, 0, 0], replace_line=None if replace_old_cf is None else replace_old_cf[0])
    y = Line(start=transform([0, 0, 0]), end=transform([0, 0.2, 0]), rgb=[0, 1, 0], replace_line=None if replace_old_cf is None else replace_old_cf[1])
    z = Line(start=transform([0, 0, 0]), end=transform([0, 0, 0.2]), rgb=[0, 0, 1], replace_line=None if replace_old_cf is None else replace_old_cf[2])
    return x, y, z

def clear_visual_item(items, env=None):
    if items is None:
        return
    env = env if env is not None else envir
    if type(items) in (list, tuple):
        for item in items:
            p.removeUserDebugItem(item, physicsClientId=env.id)
    else:
        p.removeUserDebugItem(items, physicsClientId=env.id)

def clear_all_visual_items(env=None):
    env = env if env is not None else envir
    p.removeAllUserDebugItems(physicsClientId=env.id)

