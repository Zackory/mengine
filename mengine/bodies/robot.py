import numpy as np
import pybullet as p
from .body import Body

class Robot(Body):
    def __init__(self, body, env, controllable_joints, end_effector, gripper_joints, action_duplication=None, action_multiplier=1):
        self.end_effector = end_effector # Used to get the pose of the end effector
        self.gripper_joints = gripper_joints # Gripper actuated joints
        self.action_duplication = action_duplication # The Stretch RE1 robot has a telescoping arm. The 4 linear actuators should be treated as a single actuator
        self.action_multiplier = action_multiplier
        # TODO: remove joint limits from wheels and continuous actuators
        # if self.mobile:
        #     self.controllable_joint_lower_limits[:len(self.wheel_joint_indices)] = -np.inf
        #     self.controllable_joint_upper_limits[:len(self.wheel_joint_indices)] = np.inf
        super().__init__(body, env, controllable_joints)

    def set_gripper_position(self, positions, set_instantly=False, force=500):
        self.control(positions, joints=self.gripper_joints, gains=np.array([0.05]*len(self.gripper_joints)), forces=[force]*len(self.gripper_joints), velocity_control=False, set_instantly=set_instantly)

