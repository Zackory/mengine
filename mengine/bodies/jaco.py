import os
import pybullet as p
from .robot import Robot

class Jaco(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        controllable_joints = [1, 2, 3, 4, 5, 6, 7] if controllable_joints is None else controllable_joints
        end_effector = 8 # Used to get the pose of the end effector
        gripper_joints = [9, 11, 13] # Gripper actuated joints
        body = p.loadURDF(os.path.join(directory, 'jaco', 'j2s7s300_gym.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, physicsClientId=env.id)
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)

