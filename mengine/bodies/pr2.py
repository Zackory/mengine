import os
import numpy as np
import pybullet as p
from .robot import Robot

class PR2(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        controllable_joints = [42, 43, 44, 46, 47, 49, 50] if controllable_joints is None else controllable_joints
        end_effector = 54 # Used to get the pose of the end effector
        gripper_joints = [57, 58, 59, 60] # Gripper actuated joints
        # right_arm_joint_indices = [42, 43, 44, 46, 47, 49, 50] # Controllable arm joints
        # left_arm_joint_indices = [64, 65, 66, 68, 69, 71, 72] # Controllable arm joints
        # right_end_effector = 54 # Used to get the pose of the end effector
        # left_end_effector = 76 # Used to get the pose of the end effector
        # right_gripper_indices = [57, 58, 59, 60] # Gripper actuated joints
        # left_gripper_indices = [79, 80, 81, 82] # Gripper actuated joints

        body = p.loadURDF(os.path.join(env.directory, 'PR2', 'pr2_no_torso_lift_tall.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=env.id)
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)

        # Recolor robot
        for i in [19, 42, 64]:
            p.changeVisualShape(self.body, i, rgbaColor=[1.0, 1.0, 1.0, 1.0], physicsClientId=env.id)
        for i in [43, 46, 49, 58, 60, 65, 68, 71, 80, 82]:
            p.changeVisualShape(self.body, i, rgbaColor=[0.4, 0.4, 0.4, 1.0], physicsClientId=env.id)
        for i in [45, 51, 67, 73]:
            p.changeVisualShape(self.body, i, rgbaColor=[0.7, 0.7, 0.7, 1.0], physicsClientId=env.id)
        p.changeVisualShape(self.body, 20, rgbaColor=[0.8, 0.8, 0.8, 1.0], physicsClientId=env.id)
        p.changeVisualShape(self.body, 40, rgbaColor=[0.6, 0.6, 0.6, 1.0], physicsClientId=env.id)
        # Close gripper
        # self.set_gripper_position([0]*2, set_instantly=True)

