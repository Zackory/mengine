import os
import pybullet as p
from .robot import Robot
import numpy as np

class Stretch3(Robot):
    def __init__(self, env, position=[0, 0, 0], orientation=[0, 0, 0, 1], controllable_joints=None, fixed_base=True):
        controllable_joints = [0, 1, 4, 6, 7, 8, 9, 10, 12, 13] if controllable_joints is None else controllable_joints
        end_effector = 33 # Used to get the pose of the end effector
        gripper_joints = [26, 29] # Gripper actuated joints
        body = p.loadURDF(os.path.join(env.directory, 'stretch3', 'stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf'), useFixedBase=False, basePosition=position, baseOrientation=orientation, physicsClientId=env.id)
        super().__init__(body, env, controllable_joints, end_effector, gripper_joints)

        # Fix mass
        for link in self.all_joints:
            if self.get_link_mass(link) > 0:
                self.set_mass(link, 0.01)
        # self.set_mass(self.base, 1000)
        self.set_mass(0, 100)
        self.set_mass(1, 100)
        self.set_mass(2, 100)

        # Increase friction of wheels and decrease friction of the robot base since it touches the ground
        self.set_frictions([0, 1], lateral_friction=10, spinning_friction=0, rolling_friction=0)
        self.set_frictions([self.base], lateral_friction=0.1, spinning_friction=0, rolling_friction=0)

        self.set_joint_angles(angles=[0.1], joints=[4])

        # Recolor robot
        # white = [1, 1, 1, 1]
        # gray = [0.792, 0.82, 0.933, 1]
        # dark_gray = [0.4, 0.4, 0.4, 1]
        # black = [0.251, 0.251, 0.251, 1]
        # for i in [0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 14, 19, 20, 23, 35]:
        #     p.changeVisualShape(self.body, i, rgbaColor=black, physicsClientId=env.id)
        # for i in [-1, 11, 13, 21, 16, 17, 18, 33, 34]:
        #     p.changeVisualShape(self.body, i, rgbaColor=gray, physicsClientId=env.id)
        # # for i in [16, 17, 18, 33, 34]:
        # #     p.changeVisualShape(self.body, i, rgbaColor=white, physicsClientId=id)
        # for i in [3, 32]:
        #     p.changeVisualShape(self.body, i, rgbaColor=dark_gray, physicsClientId=env.id)

