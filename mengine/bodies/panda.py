import os
import pybullet as p
from .robot import Robot

class Panda(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        controllable_joints = [0, 1, 2, 3, 4, 5, 6] if controllable_joints is None else controllable_joints
        end_effector = 11 # Used to get the pose of the end effector
        gripper_joints = [9, 10] # Gripper actuated joints
        # body = p.loadURDF(os.path.join(env.directory, 'panda', 'panda.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=env.id)
        body = p.loadURDF(os.path.join(env.directory, 'panda', 'panda.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, physicsClientId=env.id)
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)

        # Close gripper
        self.set_gripper_position([0]*2, set_instantly=True)

