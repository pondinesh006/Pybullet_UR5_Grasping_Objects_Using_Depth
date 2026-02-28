import math
import numpy as np
from collections import namedtuple

class UR5ROBOT():
    """
    Represents a UR5 robot with a Robotiq 85 gripper
    and Intel Realsense depth camera.
    """
    def __init__(self, initial_angles, pybullet_id):
        self.p = pybullet_id
        self.initial_angles = initial_angles
        self.current_angles = initial_angles
        self.eef_id = 7  # Define the end-effector link ID
        self.arm_num_dofs = 6  # Number of degrees of freedom of the UR5 robot arm
        self.gripper_range = [0, 0.085]  # Define the range of gripper opening and closing (minimum and maximum)
        self.max_velocity = 3
        self.camera_link_id = 11 # camera_depth_optical_frame

        # =========================================================
        # UR5 DH PARAMETERS
        # =========================================================
        self.d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
        self.a = [0, -0.425, -0.39225, 0, 0, 0]
        self.alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

    def load_robot(self):
        """
        Load the UR5 robot.
        """
        self.robot_id = self.p.loadURDF("Model/urdf/ur5_robotiq_85_d435.urdf", [0, 0, 0.625], useFixedBase=True)
        self.__parse_joint_info__()  # Get joint information of the robot arm
        self.__setup_mimic_joints__()  # Set up mimic joints for the gripper
        self.move_arm(self.initial_angles)

    def __parse_joint_info__(self):
        """
        Get joint information of the robot arm, including controllable joints and degree of freedom ranges.
        """
        jointInfo = namedtuple('jointInfo',
                               ['id', 'name', 'type', 'lowerLimit', 'upperLimit', 'maxForce', 'maxVelocity', 'controllable'])
        self.joints = []
        self.controllable_joints = []

        for i in range(self.p.getNumJoints(self.robot_id)):
            info = self.p.getJointInfo(self.robot_id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = jointType != self.p.JOINT_FIXED
            if controllable:
                self.controllable_joints.append(jointID)
            self.joints.append(
                jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
            )

        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]
        self.arm_lower_limits = [j.lowerLimit for j in self.joints if j.controllable][:self.arm_num_dofs]
        self.arm_upper_limits = [j.upperLimit for j in self.joints if j.controllable][:self.arm_num_dofs]
        self.arm_joint_ranges = [ul - ll for ul, ll in zip(self.arm_upper_limits, self.arm_lower_limits)]

    def __setup_mimic_joints__(self):
        """
        Set up mimic joints for the gripper to enable synchronized motion.
        """
        mimic_parent_name = 'finger_joint'
        mimic_children_names = {
            'right_outer_knuckle_joint': 1,
            'left_inner_knuckle_joint': 1,
            'right_inner_knuckle_joint': 1,
            'left_inner_finger_joint': -1,
            'right_inner_finger_joint': -1
        }
        self.mimic_parent_id = [joint.id for joint in self.joints if joint.name == mimic_parent_name][0]
        self.mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in self.joints if joint.name in mimic_children_names}

        for joint_id, multiplier in self.mimic_child_multiplier.items():
            c = self.p.createConstraint(self.robot_id, self.mimic_parent_id, self.robot_id, joint_id,
                                   jointType=self.p.JOINT_GEAR, jointAxis=[0, 1, 0],
                                   parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            self.p.changeConstraint(c, gearRatio=-multiplier, maxForce=100, erp=1)

    def move_gripper(self, open_length):
        """
        Control the gripper to open or close.
        :param open_length: Target width for gripper opening (0 ~ 0.085m)
        """
        open_length = max(self.gripper_range[0], min(open_length, self.gripper_range[1]))
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)
        self.p.setJointMotorControl2(self.robot_id, self.mimic_parent_id, self.p.POSITION_CONTROL, targetPosition=open_angle)

    # =========================================================
    # DH Transformation
    # =========================================================
    def dh(self, a, alpha, d, theta):
        """
        Calculate the DH transformation matrix.
        :param a: Link length
        :param alpha: Link twist
        :param d: Link offset
        :param theta: Link angle
        :return: DH transformation matrix
        """
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    def get_current_ee_position(self):
        """
        Get the current position of the end-effector.
        """
        eef_state = self.p.getLinkState(self.robot_id, self.eef_id)
        return eef_state

    def get_current_joint_positions(self):
        """
        Get the current joint positions of the robot arm.
        """
        num_joints = self.p.getNumJoints(self.robot_id)

        joint_states = self.p.getJointStates(self.robot_id, range(num_joints))

        # Extract only joint positions (ignore velocity, torque, etc.)
        joint_positions = [state[0] for state in joint_states]

        return joint_positions

    # =========================================================
    # Rotation matrix to orientation error (axis-angle)
    # =========================================================
    def rotation_error(self, R_des, R_cur):
        """
        Calculate the orientation error between two rotation matrices.
        :param R_des: Desired rotation matrix
        :param R_cur: Current rotation matrix
        :return: Tuple of (position, orientation)
        """
        R_err = R_des @ R_cur.T
        angle = np.arccos((np.trace(R_err) - 1) / 2.0)
        if abs(angle) < 1e-6:
            return np.zeros(3)
        axis = np.array([
            R_err[2,1] - R_err[1,2],
            R_err[0,2] - R_err[2,0],
            R_err[1,0] - R_err[0,1]
        ]) / (2*np.sin(angle))
        return axis * angle

    # =========================================================
    # Numerical Jacobian (6x6)
    # =========================================================
    def numerical_jacobian(self, q, eps=1e-6):
        """
        Calculate the numerical Jacobian matrix for the robot arm.
        :param q: Current joint angles
        :param eps: Small perturbation value
        :return: Jacobian matrix
        """
        J = np.zeros((6,6))
        T0 = self.calculate_fk(q)
        p0 = T0[:3,3]
        R0 = T0[:3,:3]

        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps

            T1 = self.calculate_fk(q + dq)
            p1 = T1[:3,3]
            R1 = T1[:3,:3]

            dp = (p1 - p0) / eps
            dR = self.rotation_error(R1, R0) / eps

            J[:3,i] = dp
            J[3:,i] = dR

        return J

    def calculate_fk(self, joint_angles):
        """
        Calculate the forward kinematics for the robot arm.
        :param joint_angles: List of joint angles in radians
        :return: Tuple of (position, orientation)
        """
        T = np.eye(4)
        for i in range(6):
            T = T @ self.dh(self.a[i], self.alpha[i], self.d[i], joint_angles[i])
        return T

    def calculate_ik(self, target_angles, max_iters=200, tol=1e-4):
        """
        Calculate the inverse kinematics for the robot arm.
        :param target_pos: Target position of the end-effector
        :param target_orientation: Target orientation of the end-effector
        :return: List of joint angles in radians
        """
        current_joint_angles = [self.p.getJointState(self.robot_id, joint_id)[0] for joint_id in self.arm_controllable_joints]

        q = np.array(self.current_angles, dtype=float)

        for i in range(max_iters):

            T_cur = self.calculate_fk(q)

            p_cur = T_cur[:3,3]
            R_cur = T_cur[:3,:3]

            p_des = target_angles[:3,3]
            R_des = target_angles[:3,:3]

            pos_err = p_des - p_cur
            rot_err = self.rotation_error(R_des, R_cur)

            error = np.hstack((pos_err, rot_err))

            if np.linalg.norm(error) < tol:
                print("Converged in", i, "iterations")
                return q

            J = self.numerical_jacobian(q)

            # Damped Least Squares
            lam = 0.01
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam*np.eye(6))

            dq = J_pinv @ error
            q += dq

        print("Did not fully converge")
        return q

    def rpy_to_rot(self, roll, pitch, yaw):
        """
        Convert Roll, Pitch, Yaw angles to a rotation matrix.
        :param roll: Roll angle in radians
        :param pitch: Pitch angle in radians
        :param yaw: Yaw angle in radians
        :return: Rotation matrix
        """
        Rx = np.array([
            [1,0,0],
            [0,np.cos(roll),-np.sin(roll)],
            [0,np.sin(roll), np.cos(roll)]
        ])

        Ry = np.array([
            [np.cos(pitch),0,np.sin(pitch)],
            [0,1,0],
            [-np.sin(pitch),0,np.cos(pitch)]
        ])

        Rz = np.array([
            [np.cos(yaw),-np.sin(yaw),0],
            [np.sin(yaw), np.cos(yaw),0],
            [0,0,1]
        ])

        return Rz @ Ry @ Rx   # ZYX convention

    def pose_xyz_rpy(self, x, y, z, roll, pitch, yaw):
        """
        Create a homogeneous transformation matrix from position and orientation.
        :param x: X coordinate
        :param y: Y coordinate
        :param z: Z coordinate
        :param roll: Roll angle in radians
        :param pitch: Pitch angle in radians
        :param yaw: Yaw angle in radians
        :return: Homogeneous transformation matrix
        """
        T = np.eye(4)
        T[:3,:3] = self.rpy_to_rot(roll, pitch, yaw)
        T[:3,3] = [x, y, z]

        return T

    def move_arm(self, joint_angles):
        """
        Move the robot arm to the specified joint angles.
        :param joint_angles: List of joint angles in radians
        """
        for i, joint_id in enumerate(self.arm_controllable_joints):
            self.p.setJointMotorControl2(self.robot_id, joint_id, self.p.POSITION_CONTROL, targetPosition=joint_angles[i])
        self.current_angles = joint_angles

    def get_camera_view(self):
        """
        Get the camera view from the robot.
        :return: Camera view
        """
        state = self.p.getLinkState(
            self.robot_id,
            self.camera_link_id,
            computeForwardKinematics=True
        )

        cam_pos = np.array(state[4])
        cam_orn = state[5]

        R = np.array(self.p.getMatrixFromQuaternion(cam_orn)).reshape(3,3)

        # ROS optical → OpenGL
        R_optical_to_gl = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])

        R_gl = R @ R_optical_to_gl

        # OpenGL camera looks along -Z
        forward = -R_gl[:,2]
        up = R_gl[:,1]

        target = cam_pos + forward

        view_matrix = self.p.computeViewMatrix(
            cam_pos.tolist(),
            target.tolist(),
            up.tolist()
        )

        return view_matrix

    def compute_extrinsics(self):
        """
        Compute the extrinsic matrix for the robot-mounted camera.
        """
        state = self.p.getLinkState(
            self.robot_id,
            self.camera_link_id,
            computeForwardKinematics=True
        )

        pos = np.array(state[4])
        orn = state[5]

        R = np.array(self.p.getMatrixFromQuaternion(orn)).reshape(3,3)

        T = np.eye(4)
        T[:3,:3] = R
        T[:3,3] = pos

        return T
