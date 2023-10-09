import os
import numpy as np
import mengine as m

class Node:
    def __init__(self, joint_angles, parent=None):
        self.angles = joint_angles
        self.parent = parent

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1] # Reverse the order of sequence


def links_in_collision(body1, link1, body2, link2, max_distance=0.0):
    return len(p.getClosestPoints(body1, body2, max_distance, linkIndexA=link1, linkIndexB=link2)) != 0


def robot_in_collision(q):
    """Returns True if the robot is in collision at the given joint angles (q).
    For simplicity, we only consider:
     1) robot joint angle limits
     2) robot collision with table and wall
    Robot self collision or collision with cubes is optional.
    """
    # set robot to joint angles
    prev_joint_angles = robot.get_joint_angles(robot.controllable_joints)
    robot.control(q, set_instantly=True)

    # robot-obstacle collision
    for obstacle in obstacles:
        if len(robot.get_closest_points(obstacle, distance=0)[-1]) != 0:
            robot.control(prev_joint_angles, set_instantly=True)
            return True

    robot.control(prev_joint_angles, set_instantly=True)
    return False


def generate_path(q1, q2, step_size=0.05):
    """Returns a list of robot joint angles from q1 to q2."""
    direction = q2 - q1
    distance = np.linalg.norm(direction)
    num_steps = int(distance / step_size)
    q = q1
    yield q1
    for i in range(num_steps):
        q = (1. / (num_steps - i)) * np.array(q2 - q) + q
        yield q


def extend(tree, target):
    """Takes current tree and extend it towards a new node (`target`).
    """
    closest_node = min(tree, key=lambda n: np.linalg.norm(n.angles - target))
    for q in generate_path(closest_node.angles, target):
        if robot_in_collision(q):
            return closest_node, False
        closest_node = Node(q, parent=closest_node)
        tree.append(closest_node)
    return closest_node, True


def random_sample_config():
    return np.random.uniform(robot.ik_lower_limits[:7], robot.ik_upper_limits[:7])


def rrt_connect(init, goal, max_iterations=100):
    """Returns a path from init to goal, using rrt_connect method.
    reference: http://www.kuffner.org/james/papers/kuffner_icra2000.pdf
    """
    """TODO: Your Answer HERE"""
    raise NotImplementedError
    return None
    """TODO: Your Answer END"""


def is_ee_close(robot, joint_angles, pos, orient):
    """Returns True if the end effector is close to the given position and orientation."""
    prev_joint_angles = robot.get_joint_angles(robot.controllable_joints)
    robot.control(joint_angles, set_instantly=True)
    ee_pos, ee_orient = robot.get_link_pos_orient(robot.end_effector)
    robot.control(prev_joint_angles, set_instantly=True)
    return np.linalg.norm(ee_pos - pos) < 0.01 and (
                np.linalg.norm(ee_orient - orient) < 0.01 or np.linalg.norm(ee_orient + orient) < 0.01)


def moveto(robot, pos, orient, avoid_collision=False, max_iter=100, max_path_length=150):
    if not avoid_collision:
        print('Using simple move')
        robot.motor_gains = 0.05
        target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient,
                                       use_current_joint_angles=True)
        robot.control(target_joint_angles)
        while np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - target_joint_angles) > 0.03:
            m.step_simulation(realtime=True)
        return

    print('Using RRT connect')
    for i in range(max_iter):
        print('moveto iteration %d' % i)
        target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
        if not is_ee_close(robot, target_joint_angles, pos, m.get_quaternion(orient)):
            print('ik solution too far, try next')
            continue

        current_joint_angles = robot.get_joint_angles(robot.controllable_joints)
        path = rrt_connect(current_joint_angles, target_joint_angles)

        if path is not None and len(path) < max_path_length:
            print('found rrt path')
            robot.motor_gains = 0.2
            color = np.random.uniform(0, 1, 3).tolist() + [1]
            for joint_angles in path:
                robot.control(joint_angles)
                # Enable for Debug
                # robot.control(joint_angles, set_instantly=True)
                # ee_pos, ee_orient = robot.get_link_pos_orient(robot.end_effector)
                # m.Shape(m.Sphere(.005), static=True, mass=0, position=ee_pos, orientation=ee_orient,
                #         rgba=color, collision=False)
                step = 0
                while np.linalg.norm(robot.get_joint_angles(robot.controllable_joints) - joint_angles) > 0.02:
                    m.step_simulation(realtime=True)
                    if step % 10 == 0:
                        ee_pos, ee_orient = robot.get_link_pos_orient(robot.end_effector)
                        m.Shape(m.Sphere(.005), static=True, mass=0, position=ee_pos, orientation=ee_orient,
                                rgba=color, collision=False)
                    step += 1
            return
        print('no rrt path found')


# Create environment and ground plane
env = m.Env(seed=1000)
ground = m.Ground()
m.visualize_coordinate_frame()

# Create table and wall
table = m.URDF(filename=os.path.join(m.directory, 'table', 'table.urdf'), static=True, position=[0, 0, 0],
               orientation=[0, 0, 0, 1])
wall = m.Shape(m.Box(half_extents=[0.02, 0.2, 0.4]), static=True, position=[-0.2, 0, 0.85],
               orientation=[0, 0, 1, 1],
               rgba=[0, 1, 1, 0.75])
obstacles = [table, wall]

# Create cubes to grasp
cubes = []
for i in range(3):
    # size = 0.025 - i * 0.0025
    size = 0.025
    position = [0.2 - i * 0.1, -0.2, 1]
    yaw = -np.pi / 4 * i
    cubes.append(m.Shape(m.Box(half_extents=[size] * 3), static=False, mass=1, position=position,
                         orientation=m.get_quaternion([0, 0, yaw]), rgba=[0, (i + 1) / 5.0, (i + 1) / 5.0, 0.75]))
    cubes[-1].set_whole_body_frictions(lateral_friction=2000, spinning_friction=2000, rolling_friction=0)

# Let the cube drop onto the table
m.step_simulation(steps=50)

# Create Panda manipulator
robot = m.Robot.Panda(position=[0.5, 0, 0.76])
robot.motor_forces = 100

# Move end effector to a starting position using IK
pos = [-0.2, 0.2, 1.2]
default_euler = np.array([np.pi, 0, 0])
orient = m.get_quaternion(default_euler)
target_joint_angles = robot.ik(robot.end_effector, target_pos=pos, target_orient=orient)
robot.control(target_joint_angles, set_instantly=True)
robot.set_gripper_position([1] * 2, set_instantly=True)  # Open gripper

for i, current_cube in enumerate(range(len(cubes))):
    # MOVETO cube
    cube_pos, cube_orient = cubes[current_cube].get_base_pos_orient()
    gripper_orient = default_euler + [0, 0, m.get_euler(cube_orient)[-1]]
    moveto(robot, cube_pos + [0, 0, 0.2], gripper_orient, avoid_collision=True)
    moveto(robot, cube_pos, gripper_orient)

    # CLOSE
    robot.set_gripper_position([0]*2, force=5000)
    m.step_simulation(steps=100, realtime=True)

    # MOVETO goal
    pos, ori = robot.get_link_pos_orient(robot.end_effector)
    moveto(robot, pos + [0, 0, 0.2], ori)
    moveto(robot, [-0.1, 0.3, 0.8 + i * 0.05], default_euler, avoid_collision=True)

    # OPEN
    robot.set_gripper_position([1]*2)
    m.step_simulation(steps=50, realtime=True)
    pos, ori = robot.get_link_pos_orient(robot.end_effector)
    moveto(robot, pos + [0, 0, 0.1], ori)

print('Done')
m.step_simulation(steps=10000, realtime=True)
