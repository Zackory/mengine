import numpy as np
import pybullet as p
from .body import Body

class Robot(Body):
    def __init__(self, body, env, controllable_joints, end_effector, gripper_joints, action_duplication=None, action_multiplier=1):
        self.end_effector = end_effector # Used to get the pose of the end effector
        self.gripper_joints = gripper_joints # Gripper actuated joints
        self.action_duplication = action_duplication # The Stretch RE1 robot has a telescoping arm. The 4 linear actuators should be treated as a single actuator
        self.action_multiplier = action_multiplier
        self.motor_forces = 1.0
        self.motor_gains = 0.05
        # TODO: remove joint limits from wheels and continuous actuators
        # if self.mobile:
        #     self.controllable_joint_lower_limits[:len(self.wheel_joint_indices)] = -np.inf
        #     self.controllable_joint_upper_limits[:len(self.wheel_joint_indices)] = np.inf
        super().__init__(body, env, controllable_joints)

    def enable_wheels(self):
        self.mobile = True
        self.controllable_joint_indices = self.wheel_joint_indices + (self.right_arm_joint_indices if 'right' in self.controllable_joints else self.left_arm_joint_indices if 'left' in self.controllable_joints else self.right_arm_joint_indices + self.left_arm_joint_indices)

    def set_gripper_open_position(self, indices, positions, set_instantly=False, force=500):
        p.setJointMotorControlArray(self.body, jointIndices=indices, controlMode=p.POSITION_CONTROL, targetPositions=positions, positionGains=np.array([0.05]*len(indices)), forces=[force]*len(indices), physicsClientId=self.id)
        if set_instantly:
            self.set_joint_angles(indices, positions, use_limits=True)

