import numpy as np
import pybullet as p

class Body:
    def __init__(self, body, env, controllable_joints=None, collision_shape=None, visual_shape=None):
        self.base = -1
        self.lower_limits = None
        self.upper_limits = None
        self.ik_lower_limits = None
        self.ik_upper_limits = None
        self.ik_joint_names = None
        self.ik_indices = None
        self.motor_gains = 0.05
        self.motor_forces = 50.0

        self.body = body
        self.id = env.id
        self.all_joints = list(range(p.getNumJoints(body, physicsClientId=self.id)))
        self.controllable_joints = controllable_joints
        self.collision_shape = collision_shape
        self.visual_shape = visual_shape
        if controllable_joints is not None:
            self.update_joint_limits()
            self.enforce_joint_limits()

    def control(self, targets, joints=None, gains=None, forces=None, velocity_control=False, set_instantly=False):
        joints = self.controllable_joints if joints is None else joints
        gains = (self.motor_gains if not velocity_control else 1) if gains is None else gains
        forces = self.motor_forces if forces is None else forces
        if type(gains) in [int, float, np.float64, np.float32]:
            gains = [gains]*len(joints)
        if type(forces) in [int, float, np.float64, np.float32]:
            forces = [forces]*len(joints)
        if not velocity_control:
            p.setJointMotorControlArray(self.body, jointIndices=joints, controlMode=p.POSITION_CONTROL, targetPositions=targets, positionGains=gains, forces=forces, physicsClientId=self.id)
        else:
            p.setJointMotorControlArray(self.body, jointIndices=joints, controlMode=p.VELOCITY_CONTROL, targetVelocities=targets, velocityGains=gains, forces=forces, physicsClientId=self.id)
        if set_instantly:
            self.set_joint_angles(targets, joints=joints, use_limits=True)

    def get_joint_angles(self, joints=None, include_fixed_joints=True):
        if joints is None:
            joints = self.all_joints
        elif not joints:
            return []
        if not include_fixed_joints:
            return np.array(self.get_motor_joint_states(joints)[1])
        else:
            robot_joint_states = p.getJointStates(self.body, jointIndices=joints, physicsClientId=self.id)
            return np.array([x[0] for x in robot_joint_states])

    def get_joint_velocities(self, joints=None, include_fixed_joints=True):
        if joints is None:
            joints = self.all_joints
        elif not joints:
            return []
        if not include_fixed_joints:
            return np.array(self.get_motor_joint_states(joints)[2])
        else:
            robot_joint_states = p.getJointStates(self.body, jointIndices=joints, physicsClientId=self.id)
            return np.array([x[1] for x in robot_joint_states])

    def get_joint_angles_dict(self, joints=None):
        return {j: a for j, a in zip(joints, self.get_joint_angles(joints))}

    def get_link_pos_orient(self, link, center_of_mass=False, local_coordinate_frame=False):
        # Get the 3D position and orientation (4D quaternion) of a specific link on the body
        if link == self.base:
            pos, orient = p.getBasePositionAndOrientation(self.body, physicsClientId=self.id)
        else:
            if not center_of_mass:
                pos, orient = p.getLinkState(self.body, link, computeForwardKinematics=True, physicsClientId=self.id)[4:6]
            else:
                pos, orient = p.getLinkState(self.body, link, computeForwardKinematics=True, physicsClientId=self.id)[:2]
        if local_coordinate_frame:
            return self.global_to_local_coordinate_frame(pos, orient)
        else:
            return np.array(pos), np.array(orient)

    def global_to_local_coordinate_frame(self, pos, orient=[0, 0, 0, 1], link=-1, rotation_only=False):
        link_pos, link_orient = self.get_base_pos_orient() if link == self.base else self.get_link_pos_orient(link)
        link_pos_inv, link_orient_inv = p.invertTransform(link_pos, link_orient, physicsClientId=self.id)
        real_pos, real_orient = p.multiplyTransforms(link_pos_inv if not rotation_only else [0, 0, 0], link_orient_inv, pos, self.get_quaternion(orient), physicsClientId=self.id)
        return np.array(real_pos), np.array(real_orient)

    def local_to_global_coordinate_frame(self, pos, orient=[0, 0, 0, 1], link=-1, rotation_only=False):
        link_pos, link_orient = self.get_base_pos_orient() if link == self.base else self.get_link_pos_orient(link)
        real_pos, real_orient = p.multiplyTransforms(link_pos if not rotation_only else [0, 0, 0], link_orient, pos, self.get_quaternion(orient), physicsClientId=self.id)
        return np.array(real_pos), np.array(real_orient)

    def get_base_pos_orient(self):
        return self.get_link_pos_orient(self.base)

    def get_link_velocity(self, link):
        if link == self.base:
            return np.array(p.getBaseVelocity(self.body, physicsClientId=self.id)[0])
        return np.array(p.getLinkState(self.body, link, computeForwardKinematics=True, computeLinkVelocity=True, physicsClientId=self.id)[6])

    def get_base_linear_velocity(self):
        return np.array(p.getBaseVelocity(self.body, physicsClientId=self.id)[0])

    def get_base_angular_velocity(self):
        return np.array(p.getBaseVelocity(self.body, physicsClientId=self.id)[1])

    def get_euler(self, quaternion):
        return np.array(quaternion) if len(quaternion) == 3 else np.array(p.getEulerFromQuaternion(np.array(quaternion), physicsClientId=self.id))

    def get_quaternion(self, euler):
        return np.array(euler) if len(euler) == 4 else np.array(p.getQuaternionFromEuler(np.array(euler), physicsClientId=self.id))

    def get_link_mass(self, link):
        return p.getDynamicsInfo(self.body, link, physicsClientId=self.id)[0]

    def get_motor_joint_states(self, joints=None):
        # Get the position, velocity, and torque for nonfixed joint motors
        joint_states = p.getJointStates(self.body, self.all_joints if joints is None else joints, physicsClientId=self.id)
        joint_infos = [p.getJointInfo(self.body, i, physicsClientId=self.id) for i in (self.all_joints if joints is None else joints)]
        motor_states = [j for j, i in zip(joint_states, joint_infos) if i[2] != p.JOINT_FIXED]
        motor_indices = [i[0] for j, i in zip(joint_states, joint_infos) if i[2] != p.JOINT_FIXED]
        motor_positions = [state[0] for state in motor_states]
        motor_velocities = [state[1] for state in motor_states]
        motor_torques = [state[3] for state in motor_states]
        return motor_indices, motor_positions, motor_velocities, motor_torques

    def get_joint_max_force(self, joints=None):
        if joints is None:
            joints = self.all_joints
        joint_infos = [p.getJointInfo(self.body, i, physicsClientId=self.id) for i in joints]
        return [j[10] for j in joint_infos]

    def get_joint_stiffness(self, joints=None):
        if joints is None:
            joints = self.all_joints
        joint_infos = [p.getJointInfo(self.body, i, physicsClientId=self.id) for i in joints]
        return [j[6] for j in joint_infos]

    def get_contact_points(self, bodyB=None, linkA=None, linkB=None, average=False):
        args = dict(bodyA=self.body, physicsClientId=self.id)
        if bodyB is not None:
            args['bodyB'] = bodyB.body
        if linkA is not None:
            args['linkIndexA'] = linkA
        if linkB is not None:
            args['linkIndexB'] = linkB
        cp = p.getContactPoints(**args)
        if cp is None or len(cp) == 0:
            return None
        if average and bodyB is not None:
            # Average all contact points together.
            avg = lambda i, absolute=False: np.mean([(abs(c[i]) if absolute else c[i]) for c in cp], axis=0)
            return dict(bodyA=cp[0][1], bodyB=cp[0][2], linkA=cp[0][3], linkB=cp[0][4], posA=avg(5), posB=avg(6), contact_normal=avg(7), contact_distance=avg(8), normal_force=avg(9, True), lateral_friction_1=avg(10, True), lateral_friction_dir_1=avg(11), lateral_friction_2=avg(12, True), lateral_friction_dir_2=avg(13))
        return [dict(bodyA=c[1], bodyB=c[2], linkA=c[3], linkB=c[4], posA=c[5], posB=c[6], contact_normal=c[7], contact_distance=c[8], normal_force=abs(c[9]), lateral_friction_1=abs(c[10]), lateral_friction_dir_1=c[11], lateral_friction_2=abs(c[12]), lateral_friction_dir_2=c[13]) for c in cp]

    def get_resultant_contact_forces(self, bodyB=None, linkA=None, linkB=None):
        # Returns the resultant forces between the two bodies
        cp = self.get_contact_points(bodyB, linkA, linkB)
        normal = np.zeros(3)
        friction_1 = np.zeros(3)
        friction_2 = np.zeros(3)
        if len(cp) > 0:
            for c in cp:
                normal += np.array(c['contact_normal']) * c['normal_force']
                friction_1 += np.array(c['lateral_friction_dir_1']) * c['lateral_friction_1']
                friction_2 += np.array(c['lateral_friction_dir_2']) * c['lateral_friction_2']
        return normal, friction_1, friction_2

    def get_closest_points(self, bodyB, distance=4.0, linkA=None, linkB=None):
        args = dict(bodyA=self.body, bodyB=bodyB.body, distance=distance, physicsClientId=self.id)
        if linkA is not None:
            args['linkIndexA'] = linkA
        if linkB is not None:
            args['linkIndexB'] = linkB
        cp = p.getClosestPoints(**args)
        linkA = [c[3] for c in cp]
        linkB = [c[4] for c in cp]
        posA = [c[5] for c in cp]
        posB = [c[6] for c in cp]
        contact_distance = [c[8] for c in cp]
        return linkA, linkB, posA, posB, contact_distance

    def get_heights(self, set_on_ground=False):
        min_z = np.inf
        max_z = -np.inf
        for i in self.all_joints + [self.base]:
            min_pos, max_pos = p.getAABB(self.body, i, physicsClientId=self.id)
            min_z = min(min_z, min_pos[-1])
            max_z = max(max_z, max_pos[-1])
        height = max_z - min_z
        base_height = self.get_link_pos_orient(self.base)[0][-1] - min_z
        if set_on_ground:
            self.set_on_ground(base_height)
        return height, base_height

    def get_AABB(self, link=-1):
        min_pos, max_pos = p.getAABB(self.body, link, physicsClientId=self.id)
        return min_pos, max_pos

    def get_force_torque_sensor(self, joint):
        return np.array(p.getJointState(self.body, joint, physicsClientId=self.id)[2])

    def get_linear_jacobian(self, link, target_joint_accelerations=None):
        return self.get_jacobian(link, target_joint_accelerations)[0]

    def get_angular_jacobian(self, link, target_joint_accelerations=None):
        return self.get_jacobian(link, target_joint_accelerations)[1]

    def get_jacobian(self, link, target_joint_accelerations=None):
        # linear_jacobian: the translational jacobian, x_dot = J_t * q_dot
        # angular_jacobian: the rotational jacobian, r_dot = J_r * q_dot
        joint_states = self.get_motor_joint_states()
        joint_positions = joint_states[1]
        joint_velocities = joint_states[2]
        target_joint_accelerations = [0]*len(joint_positions) if target_joint_accelerations is None else list(target_joint_accelerations)

        linear_jacobian, angular_jacobian = p.calculateJacobian(self.body, link, localPosition=[0]*3, objPositions=joint_positions, objVelocities=joint_velocities, objAccelerations=target_joint_accelerations, physicsClientId=self.id)
        return np.array(linear_jacobian), np.array(angular_jacobian)

    def set_base_pos_orient(self, pos=None, orient=None):
        if pos is None:
            pos, _ = self.get_base_pos_orient()
        if orient is None:
            _, orient = self.get_base_pos_orient()
        p.resetBasePositionAndOrientation(self.body, pos, self.get_quaternion(orient), physicsClientId=self.id)

    def set_base_velocity(self, linear_velocity, angular_velocity=[0, 0, 0]):
        p.resetBaseVelocity(self.body, linearVelocity=linear_velocity, angularVelocity=angular_velocity, physicsClientId=self.id)

    def set_joint_angles(self, angles, joints=None, use_limits=True, velocities=0):
        joints = self.controllable_joints if joints is None else joints
        for i, (j, a) in enumerate(zip(joints, angles)):
            p.resetJointState(self.body, jointIndex=j, targetValue=min(max(a, self.lower_limits[j]), self.upper_limits[j]) if use_limits else a, targetVelocity=velocities if type(velocities) in [int, float] else velocities[i], physicsClientId=self.id)

    def set_on_ground(self, base_height=None):
        if base_height is None:
            _, base_height = self.get_heights()
        pos, orient = self.get_base_pos_orient()
        self.set_base_pos_orient([pos[0], pos[1], base_height], orient)

    def reset_joints(self):
        # Reset all joints to 0 position, 0 velocity
        self.set_joint_angles(self.all_joints, [0]*len(self.all_joints))

    def set_whole_body_frictions(self, lateral_friction=None, spinning_friction=None, rolling_friction=None):
        self.set_frictions(self.all_joints + [self.base], lateral_friction, spinning_friction, rolling_friction)

    def set_frictions(self, links, lateral_friction=None, spinning_friction=None, rolling_friction=None):
        if type(links) == int:
            links = [links]
        for link in links:
            if lateral_friction is not None:
                p.changeDynamics(self.body, link, lateralFriction=lateral_friction, physicsClientId=self.id)
            if spinning_friction is not None:
                p.changeDynamics(self.body, link, spinningFriction=spinning_friction, physicsClientId=self.id)
            if rolling_friction is not None:
                p.changeDynamics(self.body, link, rollingFriction=rolling_friction, physicsClientId=self.id)

    def set_friction(self, links, friction):
        self.set_frictions(links, lateral_friction=friction, spinning_friction=friction, rolling_friction=friction)

    def set_mass(self, link, mass):
        p.changeDynamics(self.body, link, mass=mass, physicsClientId=self.id)

    def set_restitution(self, restitution):
        for link in self.all_joints + [self.base]:
            p.changeDynamics(self.body, link, restitution=restitution, physicsClientId=self.id)

    def set_all_joints_stiffness(self, stiffness):
        for joint in self.all_joints:
            self.set_joint_stiffness(joint, stiffness)

    def set_joint_stiffness(self, joint, stiffness):
        p.changeDynamics(self.body, joint, jointDamping=stiffness, physicsClientId=self.id)
        # p.changeDynamics(self.body, joint, contactStiffness=stiffness, contactDamping=stiffness, physicsClientId=self.id)

    # def set_gravity(self, ax=0.0, ay=0.0, az=-9.81):
    #     p.setGravity(ax, ay, az, body=self.body, physicsClientId=self.id)

    def enable_force_torque_sensor(self, joint):
        p.enableJointForceTorqueSensor(self.body, joint, enableSensor=True, physicsClientId=self.id)

    def apply_external_force(self, link=-1, force=[0,0,0], pos=[0,0,0], local_coordinate_frame=False):
        p.applyExternalForce(self.body, link, force, pos, p.LINK_FRAME if local_coordinate_frame else p.WORLD_FRAME, physicsClientId=self.id)

    def apply_external_torque(self, link=-1, torque=[0,0,0], pos=[0,0,0], local_coordinate_frame=False):
        p.applyExternalTorque(self.body, link, torque, pos, p.LINK_FRAME if local_coordinate_frame else p.WORLD_FRAME, physicsClientId=self.id)

    def create_constraint(self, parent_link, child, child_link, joint_type=p.JOINT_FIXED, joint_axis=[0, 0, 0], parent_pos=[0, 0, 0], child_pos=[0, 0, 0], parent_orient=[0, 0, 0], child_orient=[0, 0, 0], force=None):
        constraint = p.createConstraint(self.body, parent_link, child.body, child_link, joint_type, joint_axis, parent_pos, child_pos, self.get_quaternion(parent_orient), self.get_quaternion(child_orient), physicsClientId=self.id)
        if force is not None:
            p.changeConstraint(constraint, maxForce=force)
        return constraint

    def change_visual(self, link=-1, rgba=None, specular_color=None):
        if rgba is not None:
            p.changeVisualShape(self.body, link, rgbaColor=rgba, physicsClientId=self.id)
        if specular_color is not None:
            p.changeVisualShape(self.body, link, specularColor=specular_color, physicsClientId=self.id)

    def update_joint_limits(self, joints=None):
        if joints is None:
            joints = self.all_joints
        self.lower_limits = dict()
        self.upper_limits = dict()
        self.ik_lower_limits = []
        self.ik_upper_limits = []
        self.ik_joint_names = []
        for j in joints:
            joint_info = p.getJointInfo(self.body, j, physicsClientId=self.id)
            joint_name = joint_info[1]
            joint_type = joint_info[2]
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            if lower_limit == 0 and upper_limit == -1:
                lower_limit = -1e10
                upper_limit = 1e10
                if joint_type != p.JOINT_FIXED:
                    # NOTE: IK only works on non fixed joints, so we build special joint limit lists for IK
                    self.ik_lower_limits.append(-2*np.pi)
                    self.ik_upper_limits.append(2*np.pi)
                    self.ik_joint_names.append([len(self.ik_joint_names)] + list(joint_info[:2]))
            elif joint_type != p.JOINT_FIXED:
                self.ik_lower_limits.append(lower_limit)
                self.ik_upper_limits.append(upper_limit)
                self.ik_joint_names.append([len(self.ik_joint_names)] + list(joint_info[:2]))
            self.lower_limits[j] = lower_limit
            self.upper_limits[j] = upper_limit
        self.ik_lower_limits = np.array(self.ik_lower_limits)
        self.ik_upper_limits = np.array(self.ik_upper_limits)
        # Determine ik indices for the controllable joints (indices differ since fixed joints are not counted)
        self.ik_indices = []
        for i in self.controllable_joints:
            counter = 0
            for j in self.all_joints:
                if i == j:
                    self.ik_indices.append(counter)
                joint_type = p.getJointInfo(self.body, j, physicsClientId=self.id)[2]
                if joint_type != p.JOINT_FIXED:
                    counter += 1

    def enforce_joint_limits(self, joints=None):
        if joints is None:
            joints = self.all_joints
        joint_angles = self.get_joint_angles_dict(joints)
        if self.lower_limits is None or len(joints) > len(self.lower_limits):
            self.update_joint_limits()
        for j in joints:
            if joint_angles[j] < self.lower_limits[j]:
                p.resetJointState(self.body, jointIndex=j, targetValue=self.lower_limits[j], targetVelocity=0, physicsClientId=self.id)
            elif joint_angles[j] > self.upper_limits[j]:
                p.resetJointState(self.body, jointIndex=j, targetValue=self.upper_limits[j], targetVelocity=0, physicsClientId=self.id)

    def ik(self, target_joint, target_pos, target_orient=None, max_iterations=1000, use_current_joint_angles=False, randomize_limits=False):
        ik_lower_limits = self.ik_lower_limits if not randomize_limits else np.random.uniform(0, self.ik_lower_limits)
        ik_upper_limits = self.ik_upper_limits if not randomize_limits else np.random.uniform(0, self.ik_upper_limits)
        ik_joint_ranges = ik_upper_limits - ik_lower_limits
        if use_current_joint_angles:
            ik_rest_poses = np.array(self.get_motor_joint_states()[1])
        else:
            ik_rest_poses = np.random.uniform(ik_lower_limits, ik_upper_limits)

        # print('JPO:', target_joint, target_pos, target_orient)
        # print('Lower:', self.ik_lower_limits)
        # print('Upper:', self.ik_upper_limits)
        # print('Range:', ik_joint_ranges)
        # print('Rest:', ik_rest_poses)

        if target_orient is not None:
            ik_joint_poses = np.array(p.calculateInverseKinematics(self.body, target_joint, targetPosition=target_pos, targetOrientation=self.get_quaternion(target_orient), lowerLimits=ik_lower_limits.tolist(), upperLimits=ik_upper_limits.tolist(), jointRanges=ik_joint_ranges.tolist(), restPoses=ik_rest_poses.tolist(), maxNumIterations=max_iterations, physicsClientId=self.id))
        else:
            ik_joint_poses = np.array(p.calculateInverseKinematics(self.body, target_joint, targetPosition=target_pos, lowerLimits=ik_lower_limits.tolist(), upperLimits=ik_upper_limits.tolist(), jointRanges=ik_joint_ranges.tolist(), restPoses=ik_rest_poses.tolist(), maxNumIterations=max_iterations, physicsClientId=self.id))

        return ik_joint_poses[self.ik_indices]

    def print_joint_info(self, show_fixed=True):
        joint_names = []
        for j in self.all_joints:
            info = p.getJointInfo(self.body, j, physicsClientId=self.id)
            if show_fixed or info[2] != p.JOINT_FIXED:
                print(info)
                joint_names.append((j, info[1]))
        print(joint_names)

