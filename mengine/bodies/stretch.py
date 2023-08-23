import os
import pybullet as p
from .robot import Robot

class Stretch(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        controllable_joints = [3, 5, 6, 7, 8, 9] if controllable_joints is None else controllable_joints
        end_effector = 15 # Used to get the pose of the end effector
        gripper_joints = [11, 13] # Gripper actuated joints
        body = p.loadURDF(os.path.join(directory, 'stretch', 'stretch_uncalibrated.urdf'), useFixedBase=fixed_base, basePosition=position, baseOrientation=orientation, physicsClientId=env.id)
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)

        # Recolor robot
        white = [1, 1, 1, 1]
        gray = [0.792, 0.82, 0.933, 1]
        dark_gray = [0.4, 0.4, 0.4, 1]
        black = [0.251, 0.251, 0.251, 1]
        for i in [0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 14, 19, 20, 23, 35]:
            p.changeVisualShape(self.body, i, rgbaColor=black, physicsClientId=id)
        for i in [-1, 11, 13, 21, 16, 17, 18, 33, 34]:
            p.changeVisualShape(self.body, i, rgbaColor=gray, physicsClientId=id)
        # for i in [16, 17, 18, 33, 34]:
        #     p.changeVisualShape(self.body, i, rgbaColor=white, physicsClientId=id)
        for i in [3, 32]:
            p.changeVisualShape(self.body, i, rgbaColor=dark_gray, physicsClientId=id)

