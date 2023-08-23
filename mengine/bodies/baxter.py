import os
import pybullet as p
from .robot import Robot

class Baxter(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        controllable_joints = [12, 13, 14, 15, 16, 18, 19] if controllable_joints is None else controllable_joints
        end_effector = 26 # Used to get the pose of the end effector
        gripper_joints = [27, 29] # Gripper actuated joints
        # right_arm_joint_indices = [12, 13, 14, 15, 16, 18, 19] # Controllable arm joints
        # left_arm_joint_indices = [34, 35, 36, 37, 38, 40, 41] # Controllable arm joints
        # right_end_effector = 26 # Used to get the pose of the end effector
        # left_end_effector = 48 # Used to get the pose of the end effector
        # right_gripper_indices = [27, 29] # Gripper actuated joints
        # left_gripper_indices = [49, 51] # Gripper actuated joints

        body = p.loadURDF(os.path.join(directory, 'baxter', 'baxter_custom.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, physicsClientId=env.id)
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)

        # Recolor robot
        for i in [20, 21, 23, 31, 32, 42, 43, 45, 53, 54]:
            p.changeVisualShape(self.body, i, rgbaColor=[1.0, 1.0, 1.0, 0.0], physicsClientId=id)
        # Close gripper
        # self.set_gripper_position([0]*2, set_instantly=True)

