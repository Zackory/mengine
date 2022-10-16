import numpy as np
import pybullet as p
from .agent import Agent

body_joints = [0, 1, 2, 3, 4, 5, 6, 7, 8]
waist_joints = [0, 1, 2]
right_arm_joints = [9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
left_arm_joints = [19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
right_leg_joints = [33, 34, 35, 36, 37, 38, 39]
left_leg_joints = [40, 41, 42, 43, 44, 45, 46]
head_joints = [29, 30, 31, 32]
test_joints = [46, 39]

class Human(Agent):
    def __init__(self, controllable_joint_indices, controllable=False):
        super(Human, self).__init__()
        self.controllable_joint_indices = controllable_joint_indices
        self.controllable = controllable
        self.right_arm_joints = right_arm_joints
        self.left_arm_joints = left_arm_joints
        self.right_leg_joints = right_leg_joints
        self.left_leg_joints = left_leg_joints
        self.head_joints = head_joints
        self.waist = 2
        self.chest = 5
        self.upper_chest = 8
        self.right_pecs = 11
        self.right_upperarm = 14
        self.right_forearm = 16
        self.right_hand = 18
        self.left_pecs = 21
        self.left_upperarm = 24
        self.left_forearm = 26
        self.left_hand = 28
        self.neck = 29
        self.head = 32
        self.right_thigh = 35
        self.right_shin = 36
        self.right_foot = 39
        self.left_thigh = 42
        self.left_shin = 43
        self.left_foot = 46

        self.j_waist_x, self.j_waist_y, self.j_waist_z = 0, 1, 2
        self.j_chest_x, self.j_chest_y, self.j_chest_z = 3, 4, 5
        self.j_upper_chest_x, self.j_upper_chest_y, self.j_upper_chest_z = 6, 7, 8
        self.j_right_pecs_x, self.j_right_pecs_y, self.j_right_pecs_z = 9, 10, 11
        self.j_right_shoulder_x, self.j_right_shoulder_y, self.j_right_shoulder_z = 12, 13, 14
        self.j_right_elbow = 15
        self.j_right_forearm = 16
        self.j_right_wrist_x, self.j_right_wrist_y = 17, 18
        self.j_left_pecs_x, self.j_left_pecs_y, self.j_left_pecs_z = 19, 20, 21
        self.j_left_shoulder_x, self.j_left_shoulder_y, self.j_left_shoulder_z = 22, 23, 24
        self.j_left_elbow = 25
        self.j_left_forearm = 26
        self.j_left_wrist_x, self.j_left_wrist_y = 27, 28
        self.j_neck = 29
        self.j_head_x, self.j_head_y, self.j_head_z = 30, 31, 32
        self.j_right_hip_x, self.j_right_hip_y, self.j_right_hip_z = 33, 34, 35
        self.j_right_knee = 36
        self.j_right_ankle_x, self.j_right_ankle_y, self.j_right_ankle_z = 37, 38, 39
        self.j_left_hip_x, self.j_left_hip_y, self.j_left_hip_z = 40, 41, 42
        self.j_left_knee = 43
        self.j_left_ankle_x, self.j_left_ankle_y, self.j_left_ankle_z = 44, 45, 46

        self.impairment = 'random'
        self.limit_scale = 1.0
        self.tremors = np.zeros(10)
        self.target_joint_angles = None
        self.hand_radius = 0.0
        self.elbow_radius = 0.0
        self.shoulder_radius = 0.0
        self.gender = None
        self.num_body_shape = 10
        self.body_shape = np.zeros((1, self.num_body_shape))

        self.upperarm_radius = 0.0
        self.upperarm_length = 0.0
        self.forearm_radius = 0.0
        self.forearm_length = 0.0
        self.pecs_offset = 0.0

        self.motor_forces = 1.0
        self.motor_gains = 0.5

    def init(self, human_creation, limits_model, static_human_base, impairment, gender, config, id, np_random, mass=None, body_shape=None):
        self.limits_model = limits_model
        self.arm_previous_valid_pose = {True: None, False: None}
        # Choose gender
        if gender not in ['male', 'female']:
            gender = np_random.choice(['male', 'female'])
        self.gender = gender
        # Specify human impairments
        if impairment == 'random':
            impairment = np_random.choice(['none', 'limits', 'weakness', 'tremor'])
        elif impairment == 'no_tremor':
            impairment = np_random.choice(['none', 'limits', 'weakness'])
        self.impairment = impairment
        self.limit_scale = 1.0 if impairment != 'limits' else np_random.uniform(0.5, 1.0)
        self.motor_gains = 0.5 if impairment != 'weakness' else np_random.uniform(0.05, 0.5)
        if self.impairment != 'tremor':
            self.tremors = np.zeros(len(self.controllable_joint_indices))
        elif self.head in self.controllable_joint_indices:
            self.tremors = np_random.uniform(np.deg2rad(-2.5), np.deg2rad(2.5), size=len(self.controllable_joint_indices))
        else:
            self.tremors = np_random.uniform(np.deg2rad(-10), np.deg2rad(10), size=len(self.controllable_joint_indices))
        self.body_shape = body_shape
        if body_shape is None:
            # self.body_shape = np_random.uniform(-1, 5, (1, self.num_body_shape))
            self.body_shape = np_random.uniform(0, 4, (1, self.num_body_shape))
        # Initialize human
        self.body, self.head_scale, self.out_mesh, self.vertices, self.joints = human_creation.create_human(static=static_human_base, limit_scale=self.limit_scale, specular_color=[0.1, 0.1, 0.1], gender=gender, body_shape=self.body_shape)
        self.hand_radius = human_creation.hand_radius
        self.elbow_radius = human_creation.elbow_radius
        self.shoulder_radius = human_creation.shoulder_radius

        self.upperarm_radius = human_creation.upperarm_radius
        self.upperarm_length = human_creation.upperarm_length
        self.forearm_radius = human_creation.forearm_radius
        self.forearm_length = human_creation.forearm_length
        self.pecs_offset = human_creation.pecs_offset

        super(Human, self).init(self.body, id, np_random, self.controllable_joint_indices)

        # By default, initialize the person in the wheelchair
        # self.set_base_pos_orient([0, 0.03, 0.64 if self.gender == 'male' else 0.62], [0, 0, 0, 1])
        self.set_base_pos_orient([0, -0.06, 0.62 if self.gender == 'male' else 0.62], [0, 0, 0, 1])

    def setup_joints(self, joints_positions, use_static_joints=True, reactive_force=None, reactive_gain=0.05):
        # Set static joints
        joint_angles = self.get_joint_angles_dict(self.all_joint_indices)
        for j in self.all_joint_indices:
            if use_static_joints and (j not in self.controllable_joint_indices or (self.impairment != 'tremor' and reactive_force is None and not self.controllable)):
                # Make all non controllable joints on the person static by setting mass of each link (joint) to 0
                p.changeDynamics(self.body, j, mass=0, physicsClientId=self.id)
                # Set velocities to 0
                self.set_joint_angles([j], [joint_angles[j]])

        # Set starting joint positions
        self.set_joint_angles([j for j, _ in joints_positions], [np.deg2rad(j_angle) for _, j_angle in joints_positions])

        # By default, all joints have motors enabled by default that prevent free motion. Disable these motors.
        for j in self.all_joint_indices:
            p.setJointMotorControl2(self.body, jointIndex=j, controlMode=p.VELOCITY_CONTROL, force=0, physicsClientId=self.id)

        self.enforce_joint_limits()

        self.target_joint_angles = self.get_joint_angles(self.controllable_joint_indices)
        if reactive_force is not None:
            # NOTE: This runs a Position / Velocity PD controller for each joint motor on the human
            forces = [reactive_force] * len(self.target_joint_angles)
            self.control(self.controllable_joint_indices, self.target_joint_angles, reactive_gain, forces)

    def get_body_params():
        # body_shape = np.zeros(10)
        joint_ranges = np.zeros(21, 2).flatten()
        return np.concatenate([self.body_shape, joint_ranges])

    def enforce_realistic_joint_limits(self):
        # Only enforce limits for the human arm that is moveable (if either arm is even moveable)
        if (self.j_right_shoulder_x not in self.controllable_joint_indices) and (self.j_left_shoulder_x not in self.controllable_joint_indices):
            return
        right = self.j_right_shoulder_x in self.controllable_joint_indices
        indices = [self.j_right_shoulder_x, self.j_right_shoulder_y, self.j_right_shoulder_z, self.j_right_elbow] if right else [self.j_left_shoulder_x, self.j_left_shoulder_y, self.j_left_shoulder_z, self.j_left_elbow]
        tz, tx, ty, qe = self.get_joint_angles(indices)
        # Transform joint angles to match those from the Matlab data
        tz2 = (((-1 if right else 1)*tz) + 2*np.pi) % (2*np.pi)
        tx2 = (tx + 2*np.pi) % (2*np.pi)
        ty2 = (-1 if right else 1)*ty
        qe2 = (-qe + 2*np.pi) % (2*np.pi)
        result = self.limits_model.predict_classes(np.array([[tz2, tx2, ty2, qe2]]))
        if result == 1:
            # This is a valid pose for the person
            self.arm_previous_valid_pose[right] = [tz, tx, ty, qe]
        elif result == 0 and self.arm_previous_valid_pose[right] is not None:
            # The person is in an invalid pose. Move joint angles back to the most recent valid pose.
            self.set_joint_angles(indices, self.arm_previous_valid_pose[right])

