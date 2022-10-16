import os, colorsys
from gym import spaces
import pybullet as p
import numpy as np

from .agents.human_mesh import HumanMesh
from .agents.agent import Agent

import pickle

# -- Joint Legend --
# 0-2 waist x,y,z
# 3-5 chest x,y,z
# 6-8 upper_chest x,y,z
# 9-11 right_pecs x,y,z
# 12-14 right_shoulder x,y,z
# 15 right_elbow
# 16 right_forearm
# 17-18 right_hand x,y
# 19-21 left_pecs x,y,z
# 22-24 left_shoulder_socket x,y,z
# 25 left_elbow
# 26 left_forearm
# 27-28 left_hand x,y
# 29 neck x
# 30-32 head x,y,z
# 33-35 right_hip x,y,z
# 36 right_knee x
# 37-39 right_ankle x,y,z
# 40-42 left_hip x,y,z
# 43 left_knee x
# 44-46 left_ankle x,y,z

# -- Limb (link) Legend --
# 2 waist
# 5 chest
# 8 upper_chest
# 11 right_pecs
# 14 right_upperarm
# 16 right_forearm
# 18 right_hand
# 21 left_pecs
# 24 left_upperarm
# 26 left_elbow
# 28 left_hand
# 29 neck
# 32 head
# 35 right_thigh
# 36 right_shin
# 39 right_foot
# 42 left_thigh
# 43 left_shin
# 46 left_foot

class HumanCreation:
    def __init__(self, pid=None, np_random=None, cloth=False):
        self.directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'assets')
        self.cloth = cloth
        self.np_random = np_random
        self.hand_radius = 0.0
        self.elbow_radius = 0.0
        self.shoulder_radius = 0.0
        self.upperarm_radius = 0.0
        self.upperarm_length = 0.0
        self.forearm_radius = 0.0
        self.forearm_length = 0.0
        self.id = pid
        self.human_mesh = HumanMesh()

    def create_human(self, static=True, limit_scale=1.0, skin_color='random', specular_color=[0.1, 0.1, 0.1], gender='random', body_shape=None):

        if gender not in ['male', 'female']:
            gender = self.np_random.choice(['male', 'female'])
        if skin_color == 'random':
            hsv = list(colorsys.rgb_to_hsv(0.8, 0.6, 0.4))
            hsv[-1] = self.np_random.uniform(0.4, 0.8)
            skin_color = list(colorsys.hsv_to_rgb(*hsv)) + [1.0]
            # print(hsv, skin_color, colorsys.rgb_to_hls(0.8, 0.6, 0.4))
        def create_body(shape=p.GEOM_CAPSULE, radius=0, length=0, position_offset=[0, 0, 0], orientation=[0, 0, 0, 1]):
            visual_shape = p.createVisualShape(shape, radius=radius, length=length, rgbaColor=skin_color, specularColor=specular_color, visualFramePosition=position_offset, visualFrameOrientation=orientation, physicsClientId=self.id)
            collision_shape = p.createCollisionShape(shape, radius=radius, height=length, collisionFramePosition=position_offset, collisionFrameOrientation=orientation, physicsClientId=self.id)
            return collision_shape, visual_shape

        smplx_model_folder = os.path.join(self.directory, 'smpl_models', 'smplx')
        if os.path.isdir(smplx_model_folder):
            # NOTE: creating the human mesh model
            out_mesh, vertices, joints = self.human_mesh.create_smplx_body(self.directory, self.id, self.np_random, gender=gender, body_shape=body_shape, position=[0, 0, 0], orientation=[0, 0, 0])
            # self.human_mesh.init(self.directory, self.id, self.np_random, out_mesh=out_mesh, vertices=vertices, joints=joints)
            # self.human_mesh.set_on_ground()
            # spheres = self.create_spheres(radius=0.02, mass=0, batch_positions=self.human_mesh.get_joint_positions(list(range(22))), visual=True, collision=False, rgba=[0, 1, 0, 1])

            if gender == 'female':
                chest_radius = np.linalg.norm(vertices[5495] - vertices[3852])
                chest_length = np.linalg.norm(vertices[6023] - vertices[3260])
                waist_radius = np.linalg.norm(vertices[5614] - vertices[5942])
                waist_length = np.linalg.norm(vertices[7152] - vertices[4416])
                hips_radius = np.linalg.norm(vertices[5574] - vertices[5949])
                hips_length = np.linalg.norm(vertices[6832] - vertices[4088])
            else:
                chest_radius = np.linalg.norm(vertices[5496] - vertices[5950])
                chest_length = np.linalg.norm(vertices[5417] - vertices[8151])
                waist_radius = np.linalg.norm(vertices[5940] - vertices[5493])
                waist_length = np.linalg.norm(vertices[3307] - vertices[6070])
                hips_radius = np.linalg.norm(vertices[4320] - vertices[5575])
                hips_length = np.linalg.norm(vertices[6832] - vertices[4088])

            head_length = np.linalg.norm(vertices[1912] - vertices[1909])
            upperchest_radius = np.linalg.norm(vertices[5531] - vertices[5486])
            upperchest_length = np.linalg.norm(vertices[8262] - vertices[5549])
            neck_radius = np.linalg.norm(vertices[5969] - vertices[3206])/2.0
            neck_length = np.linalg.norm(joints[12] - joints[15])
            pecs_radius = np.linalg.norm(vertices[7133] - vertices[7183])/2.0
            pecs_length = np.linalg.norm(joints[17] - joints[14])
            upperarm_radius = np.linalg.norm(vertices[6170] - vertices[6883])/2.0
            upperarm_length = np.linalg.norm(joints[17] - joints[19])
            forearm_radius = np.linalg.norm(vertices[6928] - vertices[6965])/2.0
            forearm_length = np.linalg.norm(joints[19] - joints[21])
            hand_radius = np.linalg.norm(vertices[4872] - vertices[4808])/2.0
            thigh_radius = np.linalg.norm(vertices[6833] - vertices[6296])/2.0
            thigh_length = np.linalg.norm(joints[2] - joints[5])
            shin_radius = np.linalg.norm(vertices[6480] - vertices[6486])/2.0
            shin_length = np.linalg.norm(joints[5] - joints[8])
            foot_radius = np.linalg.norm(vertices[6490] - vertices[6546])/2.0
            foot_length = np.linalg.norm(vertices[8506] - vertices[8637])
            # with open(os.path.join(self.directory, 'smpl_models', 'default_joints_female.pkl'), 'wb') as f:
            #     pickle.dump(joints, f)
        else:
            out_mesh, vertices = None, None
            if gender == 'male':
                chest_radius, chest_length, waist_radius, waist_length, hips_radius, hips_length, head_length, upperchest_radius, upperchest_length, neck_radius, neck_length, pecs_radius, pecs_length, upperarm_radius, upperarm_length, forearm_radius, forearm_length, hand_radius, thigh_radius, thigh_length, shin_radius, shin_length, foot_radius, foot_length = 0.24372830018818026, 0.3235033453568208, 0.2463400061232687, 0.33162781602115443, 0.27371699449023784, 0.37429414696413166, 0.1632957817945111, 0.23463730715627404, 0.35123716305294356, 0.06057179167762721, 0.16395615154939697, 0.07240950997851447, 0.14357629907513994, 0.051181602696581105, 0.2782320035301992, 0.03542881118490844, 0.26364447441769534, 0.04550181243587064, 0.08598184656726247, 0.38346604790240313, 0.06028584649204056, 0.4370784994288006, 0.0492502996371618, 0.2420408997906238
                with open(os.path.join(self.directory, 'smpl_models', 'default_joints_male.pkl'), 'rb') as f:
                    joints = pickle.load(f)
            else:
                chest_radius, chest_length, waist_radius, waist_length, hips_radius, hips_length, head_length, upperchest_radius, upperchest_length, neck_radius, neck_length, pecs_radius, pecs_length, upperarm_radius, upperarm_length, forearm_radius, forearm_length, hand_radius, thigh_radius, thigh_length, shin_radius, shin_length, foot_radius, foot_length = 0.21575878731978226, 0.29160419113709096, 0.26688967970097627, 0.33551044766447347, 0.2581078038530574, 0.380072891719476, 0.1519502600272181, 0.24640944024754965, 0.3172224015126477, 0.053028900221613544, 0.14781758435052458, 0.06744268109291933, 0.10890902341662918, 0.049960543998057085, 0.2681915041992746, 0.030932360520105853, 0.22818899803500603, 0.03999950385300819, 0.0854951616799388, 0.37358902065229527, 0.0582057843549295, 0.3787796249953105, 0.047779593757854046, 0.22005447845611564
                with open(os.path.join(self.directory, 'smpl_models', 'default_joints_female.pkl'), 'rb') as f:
                    joints = pickle.load(f)

        joint_c, joint_v = -1, -1
        if gender == 'male':
            m = self.np_random.uniform(50, 110) # 78.4
            chest_c, chest_v = create_body(shape=p.GEOM_CAPSULE, radius=chest_radius/2.0, length=chest_length-chest_radius if chest_length-chest_radius > 0 else 0, position_offset=[0, -joints[3][1]/0.8, -joints[3][2]*0.35], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            upper_chest_c, upper_chest_v = create_body(shape=p.GEOM_CAPSULE, radius=upperchest_radius/2.0, length=upperchest_length-upperchest_radius if upperchest_length-upperchest_radius > 0 else 0, position_offset=[-joints[6][0], 0, -joints[6][2]/1.5], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            # neck_c, neck_v = create_body(shape=p.GEOM_CAPSULE, radius=neck_radius, length=neck_length/2, position_offset=[0.01, 0, 0.01])
            neck_c, neck_v = create_body(shape=p.GEOM_CAPSULE, radius=neck_radius, length=neck_length, position_offset=[0.01, 0, neck_length/4.0])
            right_shoulders_c, right_shoulders_v = create_body(shape=p.GEOM_CAPSULE, radius=4*pecs_radius/3, length=5*pecs_length/6, position_offset=[-0.0553875, 0.035, 0.031], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            left_shoulders_c, left_shoulders_v = create_body(shape=p.GEOM_CAPSULE, radius=4*pecs_radius/3, length=5*pecs_length/6, position_offset=[0.0553875, 0.035, 0.031], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            # right_upperarm_c, right_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length, position_offset=[-0.1285, 0.02, -0.022], orientation=p.getQuaternionFromEuler([np.pi/20.0, 5.3*np.pi/12.0, 0], physicsClientId=self.id))
            # left_upperarm_c, left_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length, position_offset=[0.1285 - 0.01, 0.03, -0.034], orientation=p.getQuaternionFromEuler([19*np.pi/20.0, 7.1*np.pi/12.0, 0], physicsClientId=self.id))
            # right_forearm_c, right_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[-0.1285, 0.01, 0], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            # left_forearm_c, left_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[0.1285, 0.005, 0.01], orientation=p.getQuaternionFromEuler([0, 5.8*np.pi/12.0, 0], physicsClientId=self.id))
            right_upperarm_c, right_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length-upperarm_radius/2, position_offset=[0, 0, -upperarm_length/2.0])
            left_upperarm_c, left_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length-upperarm_radius/2, position_offset=[0, 0, -upperarm_length/2.0])
            right_forearm_c, right_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[0, 0, -forearm_length/2.0])
            left_forearm_c, left_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[0, 0, -forearm_length/2.0])
            right_hand_c, right_hand_v = create_body(shape=p.GEOM_SPHERE, radius=hand_radius, length=0, position_offset=[0, 0, -hand_radius])
            left_hand_c, left_hand_v = create_body(shape=p.GEOM_SPHERE, radius=hand_radius, length=0, position_offset=[0, 0, -hand_radius])
            self.hand_radius, self.elbow_radius, self.shoulder_radius = hand_radius, (forearm_radius+upperarm_radius)/2.0, upperarm_radius
            waist_c, waist_v = create_body(shape=p.GEOM_CAPSULE, radius=waist_radius/2.0, length=waist_length-waist_radius if waist_length-waist_radius > 0 else 0, position_offset=[0, joints[0][1]/1.1, -joints[0][2]/4], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            hips_c, hips_v = create_body(shape=p.GEOM_CAPSULE, radius=hips_radius/2.0, length=hips_length-hips_radius if hips_length-hips_radius > 0 else 0, position_offset=[0, joints[1][1], joints[1][2]*0.1], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            right_thigh_c, right_thigh_v = create_body(shape=p.GEOM_CAPSULE, radius=thigh_radius, length=thigh_length, position_offset=[-0.035, 0, joints[1][2]/3])
            left_thigh_c, left_thigh_v = create_body(shape=p.GEOM_CAPSULE, radius=thigh_radius, length=thigh_length, position_offset=[0.04, 0, joints[1][2]/3])
            right_shin_c, right_shin_v = create_body(shape=p.GEOM_CAPSULE, radius=shin_radius, length=shin_length-shin_radius, position_offset=[0.01, 0.01, joints[1][2]/2.6], orientation=p.getQuaternionFromEuler([np.pi/30.0, 0, 0], physicsClientId=self.id)) #) #
            left_shin_c, left_shin_v = create_body(shape=p.GEOM_CAPSULE, radius=shin_radius, length=shin_length-shin_radius, position_offset=[-0.025, 0.01, joints[1][2]/2.6], orientation=p.getQuaternionFromEuler([np.pi/30.0, np.pi/60.0, 0], physicsClientId=self.id)) #) #
            right_foot_c, right_foot_v = create_body(shape=p.GEOM_CAPSULE, radius=foot_radius, length=foot_length, position_offset=[-0.01, -0.1, -0.00722003865], orientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id))
            left_foot_c, left_foot_v = create_body(shape=p.GEOM_CAPSULE, radius=foot_radius, length=foot_length, position_offset=[0.01, -0.1, -0.01382674078], orientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id))
            elbow_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=(forearm_radius+upperarm_radius)/2.0, length=0, rgbaColor=skin_color, visualFramePosition=[0, 0.01, 0], physicsClientId=self.id)
            if self.cloth:
                # Cloth penetrates the spheres at the end of each capsule, so we create physical spheres at the joints
                invisible_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, length=0, rgbaColor=[0.8, 0.6, 0.4, 0], physicsClientId=self.id)
                shoulder_cloth_c, _ = create_body(shape=p.GEOM_SPHERE, radius=upperarm_radius, length=0)
                elbow_cloth_c, _ = create_body(shape=p.GEOM_SPHERE, radius=(forearm_radius+upperarm_radius)/2.0, length=0)
                wrist_cloth_c, _ = create_body(shape=p.GEOM_SPHERE, radius=0.033, length=0)

            # head_scale = [0.89*(1 + head_length)]*3
            # head_pos = [0.075 + ((head_length)*0.1), 0.1 + ((head_length)*0.1), -0.13]
            # head_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'BaseHeadMeshes_v5_male_cropped_reduced_compressed_vhacd.obj'), collisionFramePosition=head_pos, collisionFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            # head_v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'BaseHeadMeshes_v5_male_cropped_reduced_compressed.obj'), rgbaColor=skin_color, specularColor=specular_color, visualFramePosition=head_pos, visualFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            head_length_scale = (head_length/0.1633)
            head_scale = [head_length_scale]*3
            head_pos = [-0.01, -0.01*(head_length/0.1633), 0.07*(head_length/0.1633)]
            head_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'head_mesh_male_new_vhacd.obj'), collisionFramePosition=head_pos, collisionFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            head_v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'head_mesh_male_new.obj'), rgbaColor=skin_color, specularColor=specular_color, visualFramePosition=head_pos, visualFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            joint_p, joint_o = [0, 0, 0], [0, 0, 0, 1]
            hips_p = [0, 0, 1.00825]
            waist_p = [joints[0][0], joints[0][1], 0.012856584787368841]
            chest_p = joints[3] - joints[0]
            upper_chest_p = joints[9] - joints[3]
            upper_chest2_p = joints[6] - joints[9]
            right_pecs_p = (joints[14] - joints[6])/2# + np.array([0, 0, (joints[14] - joints[6])[-1]/2])
            left_pecs_p = (joints[13] - joints[6])/2# + np.array([0, 0, (joints[14] - joints[6])[-1]/2])
            neck_p = joints[12] - joints[6]
            head_p = joints[15] - joints[12] + np.array([0, 0, -0.03*(head_length/0.1633)])
            # head_p = [0, 0, joints[15][-1] - joints[12][-1]]
            right_upperarm_p = joints[17] - joints[14]
            # right_forearm_p = joints[19] - joints[17]
            # right_hand_p = joints[21] - joints[19]
            # right_forearm_p = [-upperarm_length, 0, 0]
            # right_hand_p = [-forearm_length-forearm_radius/2, 0, 0]
            right_forearm_p = [0, 0, -upperarm_length]
            right_hand_p = [0, 0, -forearm_length-forearm_radius/2]
            left_upperarm_p = joints[16] - joints[13]
            # left_forearm_p = joints[18] - joints[16]
            # left_hand_p = joints[20] - joints[18]
            # left_forearm_p = [upperarm_length, 0, 0]
            # left_hand_p = [forearm_length+forearm_radius/2, 0, 0]
            left_forearm_p = [0, 0, -upperarm_length]
            left_hand_p = [0, 0, -forearm_length-forearm_radius/2]
            right_thigh_p= [joints[2][0], joints[2][1], -0.08125]
            right_shin_p = joints[5] - joints[2]
            right_shin_p = [right_shin_p[0], right_shin_p[1], right_shin_p[2]-0.007]
            right_foot_p = joints[8] - joints[5]
            right_foot_p = [right_foot_p[0], right_foot_p[1], right_foot_p[2]-0.007]
            left_thigh_p= [joints[1][0], joints[1][1], -0.08125]
            left_shin_p = joints[4] - joints[1]
            left_foot_p = joints[7] - joints[4]

            self.pecs_offset = 0.031+pecs_radius/2
        else:
            m = self.np_random.uniform(50, 110) # 62.5
            chest_c, chest_v = create_body(shape=p.GEOM_CAPSULE, radius=chest_radius/2.0, length=chest_length-chest_radius if chest_length-chest_radius > 0 else 0, position_offset=[0, -0.03, -joints[3][2]/6], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id)) #
            upper_chest_c, upper_chest_v = create_body(shape=p.GEOM_CAPSULE, radius=upperchest_radius/2.0, length=upperchest_length-upperchest_radius if upperchest_length-upperchest_radius > 0 else 0, position_offset=[-joints[6][0], -joints[6][1]/0.77, -joints[6][2]/2], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id)) #
            # neck_c, neck_v = create_body(shape=p.GEOM_CAPSULE, radius=neck_radius, length=neck_length/2, position_offset=[0.01, -0.015, 0.04])# orientation=p.getQuaternionFromEuler([0, np.pi/12.0, 0], physicsClientId=self.id)) # # not position
            neck_c, neck_v = create_body(shape=p.GEOM_CAPSULE, radius=neck_radius, length=neck_length, position_offset=[0.01, 0, neck_length/4.0])
            right_shoulders_c, right_shoulders_v = create_body(shape=p.GEOM_CAPSULE, radius=4*pecs_radius/3, length=pecs_length, position_offset=[-0.0259375, 0.025, 0.02], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id)) #
            left_shoulders_c, left_shoulders_v = create_body(shape=p.GEOM_CAPSULE, radius=4*pecs_radius/3, length=pecs_length, position_offset=[0.0259375, 0.025, 0.02], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id)) #
            # right_upperarm_c, right_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length, position_offset=[-0.132, 0.005, -0.022], orientation=p.getQuaternionFromEuler([np.pi/20.0, 5.4*np.pi/12.0, 0], physicsClientId=self.id))
            # left_upperarm_c, left_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length, position_offset=[0.132, 0.01, -0.024], orientation=p.getQuaternionFromEuler([19*np.pi/20.0, 6.8*np.pi/12.0, 0], physicsClientId=self.id))
            # right_forearm_c, right_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[-0.117, 0.005, 0], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            # left_forearm_c, left_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[0.117, 0.01, 0], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id))
            right_upperarm_c, right_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length-upperarm_radius/2, position_offset=[0, 0, -upperarm_length/2.0])
            left_upperarm_c, left_upperarm_v = create_body(shape=p.GEOM_CAPSULE, radius=upperarm_radius, length=upperarm_length-upperarm_radius/2, position_offset=[0, 0, -upperarm_length/2.0])
            right_forearm_c, right_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[0, 0, -forearm_length/2.0])
            left_forearm_c, left_forearm_v = create_body(shape=p.GEOM_CAPSULE, radius=forearm_radius, length=forearm_length, position_offset=[0, 0, -forearm_length/2.0])
            right_hand_c, right_hand_v = create_body(shape=p.GEOM_SPHERE, radius=hand_radius, length=0, position_offset=[0, 0, -hand_radius])
            left_hand_c, left_hand_v = create_body(shape=p.GEOM_SPHERE, radius=hand_radius, length=0, position_offset=[0, 0, -hand_radius])
            self.hand_radius, self.elbow_radius, self.shoulder_radius = hand_radius, (forearm_radius+upperarm_radius)/2.0, upperarm_radius
            waist_c, waist_v = create_body(shape=p.GEOM_CAPSULE, radius=waist_radius/2.0, length=waist_length-waist_radius if waist_length-waist_radius > 0 else 0, position_offset=[-joints[0][0], joints[0][0]/1.35, -joints[0][2]*0.1], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id)) #
            hips_c, hips_v = create_body(shape=p.GEOM_CAPSULE, radius=hips_radius/2.0, length=hips_length-hips_radius if hips_length-hips_radius > 0 else 0, position_offset=[0, joints[1][1] + 0.01, joints[1][2]*0.1], orientation=p.getQuaternionFromEuler([0, np.pi/2.0, 0], physicsClientId=self.id)) #
            right_thigh_c, right_thigh_v = create_body(shape=p.GEOM_CAPSULE, radius=thigh_radius, length=thigh_length, position_offset=[-0.02, 0.01, -0.1955]) #, orientation=p.getQuaternionFromEuler([0, 1*np.pi/12.0, 0], physicsClientId=self.id)) #
            left_thigh_c, left_thigh_v = create_body(shape=p.GEOM_CAPSULE, radius=thigh_radius, length=thigh_length, position_offset=[0.02, 0.01, -0.1955]) #, orientation=p.getQuaternionFromEuler([0, 11*np.pi/12.0, 0], physicsClientId=self.id)) #
            right_shin_c, right_shin_v = create_body(shape=p.GEOM_CAPSULE, radius=shin_radius, length=shin_length-shin_radius, position_offset=[0, 0.02, -0.1835+0.007-shin_radius/2], orientation=p.getQuaternionFromEuler([np.pi/30.0, 0, 0], physicsClientId=self.id)) #) #
            left_shin_c, left_shin_v = create_body(shape=p.GEOM_CAPSULE, radius=shin_radius, length=shin_length-shin_radius, position_offset=[0, 0.02, -0.1835-shin_radius/2], orientation=p.getQuaternionFromEuler([np.pi/30.0, 0, 0], physicsClientId=self.id)) #) #
            right_foot_c, right_foot_v = create_body(shape=p.GEOM_CAPSULE, radius=foot_radius, length=foot_length, position_offset=[-0.01, -0.07, -0.02448752093+0.007], orientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id)) #
            left_foot_c, left_foot_v = create_body(shape=p.GEOM_CAPSULE, radius=foot_radius, length=foot_length, position_offset=[0.01, -0.09, -0.01825895709], orientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id)) #
            elbow_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=(forearm_radius+upperarm_radius)/2.0, length=0, rgbaColor=skin_color, visualFramePosition=[0, 0.01, 0], physicsClientId=self.id)
            if self.cloth:
                # Cloth penetrates the spheres at the end of each capsule, so we create physical spheres at the joints
                invisible_v = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.01, length=0, rgbaColor=[0.8, 0.6, 0.4, 0], physicsClientId=self.id)
                shoulder_cloth_c, _ = create_body(shape=p.GEOM_SPHERE, radius=upperarm_radius, length=0)
                elbow_cloth_c, _ = create_body(shape=p.GEOM_SPHERE, radius=(forearm_radius+upperarm_radius)/2.0, length=0)
                wrist_cloth_c, _ = create_body(shape=p.GEOM_SPHERE, radius=0.027, length=0)

            # head_scale = [0.89*(1 + head_length)]*3
            # head_pos = [-0.089 - ((head_length)*0.1), -0.089 - ((head_length)*0.1), -0.11 - ((head_length)/5)]
            # head_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'BaseHeadMeshes_v5_female_cropped_reduced_compressed_vhacd.obj'), collisionFramePosition=head_pos, collisionFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            # head_v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'BaseHeadMeshes_v5_female_cropped_reduced_compressed.obj'), rgbaColor=skin_color, specularColor=specular_color, visualFramePosition=head_pos, visualFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            head_length_scale = (head_length/0.152)
            head_scale = [1.0*head_length_scale]*3
            head_pos = [0, -0.01*(head_length/0.152), 0.06*(head_length/0.152)]
            head_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'head_mesh_female_new_vhacd.obj'), collisionFramePosition=head_pos, collisionFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            head_v = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=os.path.join(self.directory, 'head_female_male', 'head_mesh_female_new.obj'), rgbaColor=skin_color, specularColor=specular_color, visualFramePosition=head_pos, visualFrameOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, 0], physicsClientId=self.id), meshScale=head_scale, physicsClientId=self.id)
            joint_p, joint_o = [0, 0, 0], [0, 0, 0, 1]
            hips_p = [0, 0, 0.951]
            # hips_p = [0, 0, 0.923]
            waist_p = [joints[0][0], joints[0][1], 0.012856584787368841]
            chest_p = joints[3] - joints[0]
            upper_chest_p = joints[9] - joints[3]
            upper_chest2_p = joints[6] - joints[9]
            right_pecs_p = (joints[14] - joints[6])/2
            left_pecs_p = (joints[13] - joints[6])/2
            neck_p = joints[12] - joints[6]
            head_p = joints[15] - joints[12] + np.array([0, 0, -0.03*(head_length/0.152)])
            # head_p = [0, joints[15][1] - joints[12][1], joints[15][-1] - joints[12][-1]]
            right_upperarm_p = joints[17] - joints[14]
            # right_forearm_p = joints[19] - joints[17]
            # right_hand_p = joints[21] - joints[19]
            right_forearm_p = [0, 0, -upperarm_length]
            right_hand_p = [0, 0, -forearm_length-forearm_radius/2]
            left_upperarm_p = joints[16] - joints[13]
            # left_forearm_p = joints[18] - joints[16]
            # left_hand_p = joints[20] - joints[18]
            left_forearm_p = [0, 0, -upperarm_length]
            left_hand_p = [0, 0, -forearm_length-forearm_radius/2]
            right_thigh_p= [joints[2][0], joints[2][1], -0.15/2]
            right_shin_p = joints[5] - joints[2]
            right_shin_p = [right_shin_p[0], right_shin_p[1], right_shin_p[2]-0.007]
            right_foot_p = joints[8] - joints[5]
            right_foot_p = [right_foot_p[0], right_foot_p[1], right_foot_p[2]-0.007]
            left_thigh_p= [joints[1][0], joints[1][1], -0.15/2]
            left_shin_p = joints[4] - joints[1]
            left_foot_p = joints[7] - joints[4]

            self.pecs_offset = 0.02+pecs_radius/2
        self.upperarm_radius, self.upperarm_length, self.forearm_radius, self.forearm_length = upperarm_radius, upperarm_length, forearm_radius, forearm_length

        linkMasses = []
        linkCollisionShapeIndices = []
        linkVisualShapeIndices = []
        linkPositions = []
        linkOrientations = []
        linkInertialFramePositions = []
        linkInertialFrameOrientations = []
        linkParentIndices = []
        linkJointTypes = []
        linkJointAxis = []
        linkLowerLimits = []
        linkUpperLimits = []

        # NOTE: Waist, chest, and upper chest
        linkMasses.extend(m*np.array([0, 0, 0.13, 0, 0, 0.1, 0, 0, 0.1]))
        linkCollisionShapeIndices.extend([joint_c, joint_c, waist_c, joint_c, joint_c, chest_c, joint_c, joint_c, upper_chest_c])
        linkVisualShapeIndices.extend([joint_v, joint_v, waist_v, joint_v, joint_v, chest_v, joint_v, joint_v, upper_chest_v])
        linkPositions.extend([waist_p, joint_p, joint_p, chest_p, joint_p, joint_p, upper_chest_p, upper_chest2_p, joint_p])
        linkOrientations.extend([joint_o]*9)
        linkInertialFramePositions.extend([[0, 0, 0]]*9)
        linkInertialFrameOrientations.extend([[0, 0, 0, 1]]*9)
        linkParentIndices.extend([0, 1, 2, 3, 4, 5, 6, 7, 8])
        linkJointTypes.extend([p.JOINT_REVOLUTE]*9)
        linkJointAxis.extend([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        linkLowerLimits.extend(np.array([np.deg2rad(-30), np.deg2rad(-30), np.deg2rad(-30), np.deg2rad(-30), np.deg2rad(-30), np.deg2rad(-30), np.deg2rad(-30), np.deg2rad(-30), np.deg2rad(-30)]))
        linkUpperLimits.extend(np.array([np.deg2rad(75), np.deg2rad(30), np.deg2rad(30), np.deg2rad(30), np.deg2rad(30), np.deg2rad(30), np.deg2rad(30), np.deg2rad(30), np.deg2rad(30)]))

        # NOTE: Pecs, neck, and head
        linkMasses.extend(m*np.array([0, 0, 0.05, 0, 0, 0.05, 0.01, 0, 0, 0.07]))
        linkCollisionShapeIndices.extend([joint_c, joint_c, right_shoulders_c, joint_c, joint_c, left_shoulders_c, neck_c, joint_c, joint_c, head_c])
        linkVisualShapeIndices.extend([joint_v, joint_v, right_shoulders_v, joint_v, joint_v, left_shoulders_v, neck_v, joint_v, joint_v, head_v])
        linkPositions.extend([right_pecs_p, right_pecs_p, joint_p, left_pecs_p, left_pecs_p, joint_p, neck_p, head_p, joint_p, joint_p])
        linkOrientations.extend([joint_o]*10)
        linkInertialFramePositions.extend([[0, 0, 0]]*10)
        linkInertialFrameOrientations.extend([[0, 0, 0, 1]]*10)
        linkParentIndices.extend([9, 10, 11, 9, 13, 14, 9, 16, 17, 18])
        linkJointTypes.extend([p.JOINT_REVOLUTE]*10)
        linkJointAxis.extend([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        linkLowerLimits.extend(np.array([np.deg2rad(-20), np.deg2rad(-20), np.deg2rad(-20), np.deg2rad(-20), np.deg2rad(-20), np.deg2rad(-20), np.deg2rad(-10), np.deg2rad(-50), np.deg2rad(-34), np.deg2rad(-70)])*limit_scale)
        linkUpperLimits.extend(np.array([np.deg2rad(20), np.deg2rad(20), np.deg2rad(20), np.deg2rad(20), np.deg2rad(20), np.deg2rad(20), np.deg2rad(20), np.deg2rad(50), np.deg2rad(34), np.deg2rad(70)])*limit_scale)


        # NOTE: Right arm
        linkMasses.extend(m*np.array([0, 0, 0.033, 0, 0.019, 0, 0.0065]))
        if not self.cloth:
            linkCollisionShapeIndices.extend([joint_c, joint_c, right_upperarm_c, joint_c, right_forearm_c, joint_c, right_hand_c])
            linkVisualShapeIndices.extend([joint_v, joint_v, right_upperarm_v, elbow_v, right_forearm_v, joint_v, right_hand_v])
        else:
            linkCollisionShapeIndices.extend([joint_c, shoulder_cloth_c, right_upperarm_c, elbow_cloth_c, right_forearm_c, wrist_cloth_c, right_hand_c])
            linkVisualShapeIndices.extend([joint_v, invisible_v, right_upperarm_v, elbow_v, right_forearm_v, invisible_v, right_hand_v])
        linkPositions.extend([right_upperarm_p, joint_p, joint_p, right_forearm_p, joint_p, right_hand_p, joint_p])
        linkOrientations.extend([joint_o]*7)
        linkInertialFramePositions.extend([[0, 0, 0]]*7)
        linkInertialFrameOrientations.extend([[0, 0, 0, 1]]*7)
        linkParentIndices.extend([12, 20, 21, 22, 23, 24, 25])
        linkJointTypes.extend([p.JOINT_REVOLUTE]*7)
        linkJointAxis.extend([[0, 1, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0]])
        # linkLowerLimits.extend(np.array([np.deg2rad(-90), np.deg2rad(-111), np.deg2rad(-90), np.deg2rad(-128), np.deg2rad(-90), np.deg2rad(-81), np.deg2rad(-27)])*limit_scale)
        # linkUpperLimits.extend(np.array([np.deg2rad(90), np.deg2rad(61), np.deg2rad(90), np.deg2rad(0), np.deg2rad(90), np.deg2rad(90), np.deg2rad(47)])*limit_scale)
        linkLowerLimits.extend(np.array([np.deg2rad(5), np.deg2rad(-188), np.deg2rad(-90), np.deg2rad(-128), np.deg2rad(-90), np.deg2rad(-81), np.deg2rad(-27)])*limit_scale)
        linkUpperLimits.extend(np.array([np.deg2rad(198), np.deg2rad(61), np.deg2rad(90), np.deg2rad(0), np.deg2rad(90), np.deg2rad(90), np.deg2rad(47)])*limit_scale)

        # NOTE: Left arm
        linkMasses.extend(m*np.array([0, 0, 0.033, 0, 0.019, 0, 0.0065]))
        if not self.cloth:
            linkCollisionShapeIndices.extend([joint_c, joint_c, left_upperarm_c, joint_c, left_forearm_c, joint_c, left_hand_c])
            linkVisualShapeIndices.extend([joint_v, joint_v, left_upperarm_v, elbow_v, left_forearm_v, joint_v, left_hand_v])
        else:
            linkCollisionShapeIndices.extend([joint_c, shoulder_cloth_c, left_upperarm_c, elbow_cloth_c, left_forearm_c, wrist_cloth_c, left_hand_c])
            linkVisualShapeIndices.extend([joint_v, invisible_v, left_upperarm_v, elbow_v, left_forearm_v, invisible_v, left_hand_v])
        linkPositions.extend([left_upperarm_p, joint_p, joint_p, left_forearm_p, joint_p, left_hand_p, joint_p])
        linkOrientations.extend([joint_o]*7)
        linkInertialFramePositions.extend([[0, 0, 0]]*7)
        linkInertialFrameOrientations.extend([[0, 0, 0, 1]]*7)
        linkParentIndices.extend([15, 27, 28, 29, 30, 31, 32])
        linkJointTypes.extend([p.JOINT_REVOLUTE]*7)
        linkJointAxis.extend([[0, 1, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0]])
        # linkLowerLimits.extend(np.array([np.deg2rad(-90), np.deg2rad(-111), np.deg2rad(-90), np.deg2rad(-128), np.deg2rad(-90), np.deg2rad(-81), np.deg2rad(-47)])*limit_scale)
        # linkUpperLimits.extend(np.array([np.deg2rad(90), np.deg2rad(61), np.deg2rad(90), np.deg2rad(0), np.deg2rad(90), np.deg2rad(90), np.deg2rad(27)])*limit_scale)
        linkLowerLimits.extend(np.array([np.deg2rad(-198), np.deg2rad(-188), np.deg2rad(-90), np.deg2rad(-128), np.deg2rad(-90), np.deg2rad(-81), np.deg2rad(-47)])*limit_scale)
        linkUpperLimits.extend(np.array([np.deg2rad(-5), np.deg2rad(61), np.deg2rad(90), np.deg2rad(0), np.deg2rad(90), np.deg2rad(90), np.deg2rad(27)])*limit_scale)

        # NOTE: Right leg
        linkMasses.extend(m*np.array([0, 0, 0.105, 0.0475, 0, 0, 0.014]))
        linkCollisionShapeIndices.extend([joint_c, joint_c, right_thigh_c, right_shin_c, joint_c, joint_c, right_foot_c])
        linkVisualShapeIndices.extend([joint_v, joint_v, right_thigh_v, right_shin_v, joint_v, joint_v, right_foot_v])
        linkPositions.extend([right_thigh_p, joint_p, joint_p, right_shin_p, right_foot_p, joint_p, joint_p])
        linkOrientations.extend([joint_o]*7)
        linkInertialFramePositions.extend([[0, 0, 0]]*7)
        linkInertialFrameOrientations.extend([[0, 0, 0, 1]]*7)
        linkParentIndices.extend([0, 34, 35, 36, 37, 38, 39])
        linkJointTypes.extend([p.JOINT_REVOLUTE]*7)
        linkJointAxis.extend([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        linkLowerLimits.extend(np.array([np.deg2rad(-127), np.deg2rad(-40), np.deg2rad(-45), 0, np.deg2rad(-35), np.deg2rad(-23), np.deg2rad(-43)]))
        linkUpperLimits.extend(np.array([np.deg2rad(30), np.deg2rad(45), np.deg2rad(40), np.deg2rad(130), np.deg2rad(38), np.deg2rad(24), np.deg2rad(35)]))

        # NOTE: Left leg
        linkMasses.extend(m*np.array([0, 0, 0.105, 0.0475, 0, 0, 0.014]))
        linkCollisionShapeIndices.extend([joint_c, joint_c, left_thigh_c, left_shin_c, joint_c, joint_c, left_foot_c])
        linkVisualShapeIndices.extend([joint_v, joint_v, left_thigh_v, left_shin_v, joint_v, joint_v, left_foot_v])
        linkPositions.extend([left_thigh_p, joint_p, joint_p, left_shin_p, left_foot_p, joint_p, joint_p])
        linkOrientations.extend([joint_o]*7)
        linkInertialFramePositions.extend([[0, 0, 0]]*7)
        linkInertialFrameOrientations.extend([[0, 0, 0, 1]]*7)
        linkParentIndices.extend([0, 41, 42, 43, 44, 45, 46])
        linkJointTypes.extend([p.JOINT_REVOLUTE]*7)
        linkJointAxis.extend([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        linkLowerLimits.extend(np.array([np.deg2rad(-127), np.deg2rad(-45), np.deg2rad(-40), 0, np.deg2rad(-35), np.deg2rad(-24), np.deg2rad(-35)]))
        linkUpperLimits.extend(np.array([np.deg2rad(30), np.deg2rad(40), np.deg2rad(45), np.deg2rad(130), np.deg2rad(38), np.deg2rad(23), np.deg2rad(43)]))
        human = p.createMultiBody(baseMass=0 if static else m*0.14, baseCollisionShapeIndex=hips_c, baseVisualShapeIndex=hips_v, basePosition=hips_p, baseOrientation=[0, 0, 0, 1], linkMasses=linkMasses, linkCollisionShapeIndices=linkCollisionShapeIndices, linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions, linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions, linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=linkParentIndices, linkJointTypes=linkJointTypes, linkJointAxis=linkJointAxis, linkLowerLimits=linkLowerLimits, linkUpperLimits=linkUpperLimits, useMaximalCoordinates=False, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.id)

        # Self collision has been enabled for the person
        # For stability: Remove all collisions except between the arms/legs and the other body parts
        num_joints = p.getNumJoints(human, physicsClientId=self.id)
        for i in range(-1, num_joints):
            for j in range(-1, num_joints):
                p.setCollisionFilterPair(human, human, i, j, 0, physicsClientId=self.id)
        for i in range(12, 19): # Right arm
            for j in list(range(-1, 3)) + list(range(19, num_joints)):
                p.setCollisionFilterPair(human, human, i, j, 1, physicsClientId=self.id)
        for i in range(22, 29): # Left arm
            for j in list(range(-1, 3)) + list(range(9, 19)) + list(range(29, num_joints)):
                p.setCollisionFilterPair(human, human, i, j, 1, physicsClientId=self.id)
        for i in range(33, 40): # Right leg
            for j in list(range(6, 33)) + list(range(40, num_joints)):
                p.setCollisionFilterPair(human, human, i, j, 1, physicsClientId=self.id)
        for i in range(40, num_joints): # Left leg
            for j in list(range(6, 40)):
                p.setCollisionFilterPair(human, human, i, j, 1, physicsClientId=self.id)

        human_joint_states = p.getJointStates(human, jointIndices=list(range(p.getNumJoints(human, physicsClientId=self.id))), physicsClientId=self.id)
        human_joint_positions = np.array([x[0] for x in human_joint_states])
        for j in range(p.getNumJoints(human, physicsClientId=self.id)):
            p.resetJointState(human, jointIndex=j, targetValue=0, targetVelocity=0, physicsClientId=self.id)

        return human, head_length_scale, out_mesh, vertices, joints
