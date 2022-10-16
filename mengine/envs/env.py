import os, time
import numpy as np
from screeninfo import get_monitors
import pybullet as p

from .util import Util
from .agents import agent
from .agents.agent import Agent

# from .human_creation import HumanCreation
# from .agents import agent, human, robot, panda, tool, furniture
# from .agents.agent import Agent
# from .agents.human import Human
# from .agents.robot import Robot
# from .agents.panda import Panda
# from .agents.tool import Tool
# from .agents.furniture import Furniture

e = None

class Env(gym.Env):
    def __init__(self, build_ground=True, time_step=0.02, frame_skip=5, gravity=[0, 0, -9.81], render=True, gpu_rendering=False, seed=1001, deformable=False):
        global e
        e = self
        self.time_step = time_step
        self.frame_skip = frame_skip
        self.gravity = gravity
        self.id = None
        self.gui = False
        self.gpu_rendering = gpu_rendering
        self.view_matrix = None
        self.deformable = deformable
        self.seed(seed)
        self.directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'assets')
        if render:
            self.gui = True
            try:
                self.width = get_monitors()[0].width
                self.height = get_monitors()[0].height
            except Exception as e:
                self.width = 1920
                self.height = 1080
            self.id = p.connect(p.GUI, options='--background_color_red=0.8 --background_color_green=0.9 --background_color_blue=1.0 --width=%d --height=%d' % (self.width, self.height))
        else:
            self.id = p.connect(p.DIRECT)
        self.util = Util(self.id, self.np_random)

        self.agents = []
        self.ground = Agent()
        self.camera = None

        self.reset(build_ground)

    def seed(self, seed=1001):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def disconnect(self):
        p.disconnect(self.id)

    def reset(self, build_ground=True):
        if self.deformable:
            p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD, physicsClientId=self.id)
        else:
            p.resetSimulation(physicsClientId=self.id)
        if self.gpu_rendering:
            self.util.enable_gpu()
        self.camera = Camera()
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=self.id)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.id)
        p.setTimeStep(self.time_step, physicsClientId=self.id)
        # Disable real time simulation so that the simulation only advances when we call stepSimulation
        p.setRealTimeSimulation(0, physicsClientId=self.id)
        p.setGravity(self.gravity[0], self.gravity[1], self.gravity[2], physicsClientId=self.id)
        self.agents = []
        self.last_sim_time = None
        self.iteration = 0
        if build_ground:
            # Load the ground plane
            plane = p.loadURDF(os.path.join(self.directory, 'plane', 'plane.urdf'), physicsClientId=self.id)
            self.ground.init(plane, self.id, self.np_random, indices=-1)
            # Randomly set friction of the ground
            # self.ground.set_frictions(self.ground.base, lateral_friction=self.np_random.uniform(0.025, 0.5), spinning_friction=0, rolling_friction=0)

    def create_robot(self, robot_class, controllable_joints='right', fixed_base=True):
        self.robot = robot_class(controllable_joints)
        self.robot.init(self.directory, self.id, self.np_random, fixed_base=fixed_base)
        self.agents.append(self.robot)
        return self.robot

    def take_step(self, actions, gains=None, forces=None, action_multiplier=0.05, step_sim=True, ik=False):
        if gains is None:
            gains = [a.motor_gains for a in self.agents]
        elif type(gains) not in (list, tuple):
            gains = [gains]*len(self.agents)
        if forces is None:
            forces = [a.motor_forces for a in self.agents]
        elif type(forces) not in (list, tuple):
            forces = [forces]*len(self.agents)

        if self.last_sim_time is None:
            self.last_sim_time = time.time()
        self.iteration += 1
        actions = np.clip(actions, a_min=self.action_space.low, a_max=self.action_space.high)
        actions *= action_multiplier
        action_index = 0
        for i, agent in enumerate(self.agents):
            needs_action = not isinstance(agent, Human) or agent.controllable
            if needs_action:
                if not ik or isinstance(agent, Human):
                    agent_action_len = len(agent.controllable_joint_indices)
                else:
                    agent_action_len = 3 + 4 # 3 positon, 4 quaternion
                action = np.copy(actions[action_index:action_index+agent_action_len])
                action_index += agent_action_len
                if isinstance(agent, Robot):
                    action *= agent.action_multiplier
                if len(action) != agent_action_len:
                    print('Received agent actions of length %d does not match expected action length of %d' % (len(action), agent_action_len))
                    exit()
            # Append the new action to the current measured joint angles
            agent_joint_angles = agent.get_joint_angles(agent.controllable_joint_indices)
            if not ik or isinstance(agent, Human):
                # Update the target robot/human joint angles based on the proposed action and joint limits
                for _ in range(self.frame_skip):
                    if needs_action:
                        below_lower_limits = agent_joint_angles + action < agent.controllable_joint_lower_limits
                        above_upper_limits = agent_joint_angles + action > agent.controllable_joint_upper_limits
                        action[below_lower_limits] = 0
                        action[above_upper_limits] = 0
                        agent_joint_angles[below_lower_limits] = agent.controllable_joint_lower_limits[below_lower_limits]
                        agent_joint_angles[above_upper_limits] = agent.controllable_joint_upper_limits[above_upper_limits]
                    if isinstance(agent, Human) and agent.impairment == 'tremor':
                        if needs_action:
                            agent.target_joint_angles += action
                        agent_joint_angles = agent.target_joint_angles + agent.tremors * (1 if self.iteration % 2 == 0 else -1)
                    else:
                        agent_joint_angles += action
            else:
                joint = agent.right_end_effector if 'right' in agent.controllable_joints else agent.left_end_effector
                ik_indices = agent.right_arm_ik_indices if 'right' in agent.controllable_joints else agent.left_arm_ik_indices
                # NOTE: Adding action to current pose can cause drift over time
                pos, orient = agent.get_pos_orient(joint)
                # NOTE: Adding action to target pose can cause large targets far outside of the robot's work space that take a long time to come back from
                # pos, orient = np.copy(agent.target_ee_position), np.copy(agent.target_ee_orientation)
                # print('Reached pos:', pos, 'Reached orient:', orient)
                # print('Reached pos:', pos, 'Reached orient:', self.get_euler(orient))
                pos += action[:len(pos)]
                orient += action[len(pos):]
                # orient = self.get_quaternion(self.get_euler(orient) + action[len(pos):len(pos)+3]) # NOTE: RPY
                # print('Target pos:', pos, 'Target orient:', orient)
                # print('Target pos:', pos, 'Target orient:', self.get_euler(orient) + action[len(pos):len(pos)+3])
                agent_joint_angles = agent.ik(joint, pos, orient, ik_indices, max_iterations=200, use_current_as_rest=True)
            if isinstance(agent, Robot) and agent.action_duplication is not None:
                agent_joint_angles = np.concatenate([[a]*d for a, d in zip(agent_joint_angles, self.robot.action_duplication)])
                agent.control(agent.all_controllable_joints, agent_joint_angles, agent.gains, agent.forces)
            else:
                agent.control(agent.controllable_joint_indices, agent_joint_angles, gains[i], forces[i])
        if step_sim:
            # Update all agent positions
            for _ in range(self.frame_skip):
                p.stepSimulation(physicsClientId=self.id)
                for agent in self.agents:
                    if isinstance(agent, Human):
                        agent.enforce_joint_limits()
                        if agent.controllable:
                            agent.enforce_realistic_joint_limits()
                self.update_targets()
                if self.gui:
                    # Slow down time so that the simulation matches real time
                    self.slow_time()

    def slow_time(self):
        # Slow down time so that the simulation matches real time
        t = time.time() - self.last_sim_time
        if t < self.time_step:
            time.sleep(self.time_step - t)
        self.last_sim_time = time.time()


class Camera:
    def __init__(self, camera_pos=[0.5, -0.5, 1.5], look_at_pos=[0, 0, 0.75], fov=60, camera_width=1920//4, camera_height=1080//4, env=None):
        self.env = env if env is not None else e
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
    env = env if env is not None else e
    return env.camera.get_rgba_depth(light_pos=light_pos, shadow=shadow, ambient=ambient, diffuse=diffuse, specular=specular)

def get_euler(quaternion, env=None):
    env = env if env is not None else e
    return np.array(p.getEulerFromQuaternion(np.array(quaternion), physicsClientId=env.id))

def get_quaternion(euler, env=None):
    env = env if env is not None else e
    return np.array(p.getQuaternionFromEuler(np.array(euler), physicsClientId=env.id))


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

class Box:
    def __init__(self, half_extents=[1, 1, 1]):
        super().__init__(type=p.GEOM_BOX, half_extents=half_extents)

class Capsule:
    def __init__(self, radius=1, length=1):
        super().__init__(type=p.GEOM_CAPSULE, radius=radius, length=length)

class Cylinder:
    def __init__(self, radius=1, length=1):
        super().__init__(type=p.GEOM_CYLINDER, radius=radius, length=length)

class Plane:
    def __init__(self, normal=[0, 0, 1]):
        super().__init__(type=p.GEOM_PLANE, normal=normal)

class Mesh:
    def __init__(self, filename='', scale=[1, 1, 1]):
        super().__init__(type=p.GEOM_MESH, filename=filename, scale=scale)

def Shape(shape, static=False, mass=1.0, position=[0, 0, 0], orientation=[0, 0, 0, 1], visual=True, collision=True, rgba=[0, 1, 1, 1], maximal_coordinates=False, return_collision_visual=False, env=None):
    env = env if env is not None else e
    collision = p.createCollisionShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, height=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, physicsClientId=env.id) if collision else -1
    visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, rgbaColor=rgba, physicsClientId=env.id) if visual else -1
    if return_collision_visual:
        return collision, visual
    body = p.createMultiBody(baseMass=0 if static else mass, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=position, baseOrientation=orientation, useMaximalCoordinates=maximal_coordinates, physicsClientId=env.id)
    s = Agent()
    s.init(body, env.id, env.np_random, indices=-1)
    return s

def Shapes(shape, static=False, mass=1.0, positions=[[0, 0, 0]], orientation=[0, 0, 0, 1], visual=True, collision=True, rgba=[0, 1, 1, 1], maximal_coordinates=False, return_collision_visual=False, env=None):
    env = env if env is not None else e
    collision = p.createCollisionShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, height=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, physicsClientId=env.id) if collision else -1
    visual = p.createVisualShape(shapeType=shape.type, radius=shape.radius, halfExtents=shape.half_extents, length=shape.length, fileName=shape.filename, meshScale=shape.scale, planeNormal=shape.normal, rgbaColor=rgba, physicsClientId=env.id) if visual else -1
    if return_collision_visual:
        return collision, visual
    shape_ids = p.createMultiBody(baseMass=0 if static else mass, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=positions[0], baseOrientation=orientation, batchPositions=positions, useMaximalCoordinates=maximal_coordinates, physicsClientId=env.id)
    shapes = []
    for body in shape_ids:
        s = Agent()
        s.init(body, env.id, env.np_random, indices=-1)
        shapes.append(s)
    return shapes

