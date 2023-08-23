import os
import pybullet as p
from .robot import Robot

class Sawyer(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        controllable_joints = [3, 8, 9, 10, 11, 13, 16] if controllable_joints is None else controllable_joints
        end_effector = 19 # Used to get the pose of the end effector
        gripper_joints = [20, 22] # Gripper actuated joints
        body = p.loadURDF(os.path.join(directory, 'sawyer', 'sawyer.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, physicsClientId=env.id)
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)

