#! /usr/bin/env python3

import rospy
import tf2_ros
import argparse
import numpy as np
import math
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from urdf_parser_py.urdf import URDF


class Kinematics:
    """
    UR3e kinematics for pick and place.
    
    Provides forward kinematics (joint angles to end-effector pose) and inverse
    kinematics (target pose to joint angles), and derives all motion-sequence
    waypoints with their joint angles from the pick and place coordinates.
    """

    # Home pose upright, The sequence starts and ends here.
    HOME_RAD = np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])

    # Safe start pose — verified teach-pendant pose with the gripper pointing
    # straight down (degrees). The robot moves here right after home, and the
    # grasp orientation for all pick/place waypoints is derived from it.
    SAFE_START_DEG = np.array([79.94, -88.37, 0.13, -2.21, -89.37, 351.79])

    # --- Height calibration (measured on the real robot) ---
    # Single measured height offset between the code's zero and the desired grasp height
    Z_OFFSET = 0.215 # m

    # Approach offset above the object top
    APPROACH_OFFSET = 0.05  # m

    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('kinematics_unified')
        self.joint_state_publisher = rospy.Publisher('my_joint_states', JointState, queue_size=10)
        self.pose_publisher        = rospy.Publisher('my_pose', PoseStamped, queue_size=10)
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(10)
        rospy.sleep(1.0)

    def run(self, pick_x, pick_y, place_x, place_y, object_height, gripper_width_mm):
        """
        Compute all waypoint joint angles for a pick-and-place sequence.

        Parameters
        ----------
        pick_x / pick_y     : float  – pick position in meters (robot base frame)
        place_x / place_y   : float  – place position in meters
        object_height       : float  – object height in meters
        gripper_width_mm    : float  – gripper opening needed to grasp (mm)

        Returns
        -------
        dict with keys: home, start, pick_approach, pick, pick_retreat,
                        place_approach, place, place_retreat
        """
        robot = URDF.from_parameter_server()
        root = robot.get_root()
        joint_names = robot.get_chain(root, "tool0", joints=True, links=False, fixed=False)

        # Home pose (very start & very end of the sequence)
        q_home = self.HOME_RAD.copy()

        # Safe start pose (gripper straight down): reached right after home
        # The grasp orientation for every pick/place waypoint is taken from here.
        q_start = np.deg2rad(self.SAFE_START_DEG)
        
        # Unwrap each joint to the shortest path from home
        q_start = self._unwrap_to_seed(q_start, q_home)
        T_start = self.calculate_forward_kinematics(q_start, joint_names, robot)
        grasp_rot = Rotation.from_matrix(T_start[:3, :3])
        rospy.loginfo("Home (upright): %s", np.round(np.rad2deg(q_home), 1))
        rospy.loginfo("Safe start (unwrapped): %s", np.round(np.rad2deg(q_start), 1))
        rospy.loginfo("Safe start FK: pos=(%.1f, %.1f, %.1f) mm",
                      T_start[0,3]*1000, T_start[1,3]*1000, T_start[2,3]*1000)

        # Derive waypoint Cartesian poses 
        # grasp_z = Z_OFFSET + object_height/2
        # above_z = Z_OFFSET + object_height + APPROACH_OFFSET
        grasp_z  = self.Z_OFFSET + object_height / 2.0
        above_z  = self.Z_OFFSET + object_height + self.APPROACH_OFFSET

        # Pick poses (orientation = same as start → gripper pointing down)
        pick_approach_pos = np.array([pick_x,  pick_y,  above_z])
        pick_pos = np.array([pick_x,  pick_y,  grasp_z])

        # Place poses
        place_approach_pos = np.array([place_x, place_y, above_z])
        place_pos = np.array([place_x, place_y, grasp_z])

        # Place retreat: slightly toward the safe-start pose in XY, same height as approach
        dir_xy = np.array([T_start[0,3] - place_x, T_start[1,3] - place_y])
        dist = np.linalg.norm(dir_xy)
        retreat_offset = (dir_xy / dist * 0.03) if dist > 0.001 else np.zeros(2)
        place_retreat_pos = np.array([place_x + retreat_offset[0],
                                      place_y + retreat_offset[1],
                                      above_z])

        # Solve IK for each waypoint (chained seeds)
        waypoints = {}
        waypoints['home']  = q_home  # upright — very first and very last move
        waypoints['start'] = q_start # safe gripper-down pose

        configs = [
            ('pick_approach',  pick_approach_pos,  grasp_rot,   q_start),
            ('pick',           pick_pos,            grasp_rot,   None),     
            ('pick_retreat',   pick_approach_pos,   grasp_rot,   None),
            ('place_approach', place_approach_pos,  grasp_rot,   None),
            ('place',          place_pos,           grasp_rot,   None),
            ('place_retreat',  place_retreat_pos,   grasp_rot,   None),
        ]

        prev_q = q_start
        for name, pos, rot, seed in configs:
            q_seed = seed if seed is not None else prev_q
            rospy.loginfo("Solving IK: %s  target=(%.1f, %.1f, %.1f) mm",
                          name, pos[0]*1000, pos[1]*1000, pos[2]*1000)
            q, success = self.solve_ik_natural(pos, rot, q_seed, joint_names, robot)
            if not success:
                rospy.logerr("IK failed for waypoint: %s", name)
                return None
            # Unwrap angles to take shortest path from previous configuration.
            q = self._unwrap_to_seed(q, q_seed)
            waypoints[name] = q
            prev_q = q

        rospy.loginfo("All waypoints computed successfully.")
        return waypoints

    # Forward kinematics
    def _fk_raw(self, joint_positions, joint_names, robot):
        """Forward kinematics in the raw URDF (base_link) frame."""
        root = robot.get_root()
        all_joint_names = robot.get_chain(root, "tool0", joints=True, links=False, fixed=True)

        T = np.eye(4)
        revolute_idx = 0

        for jname in all_joint_names:
            joint = robot.joint_map[jname]
            xyz = joint.origin.xyz if joint.origin else [0, 0, 0]
            rpy = joint.origin.rpy if joint.origin else [0, 0, 0]
            T = T @ self._transformation_matrix(xyz, rpy)

            if joint.type in ('revolute', 'continuous'):
                axis  = joint.axis if joint.axis is not None else [0, 0, 1]
                theta = joint_positions[revolute_idx]
                T     = T @ self._rotation_matrix_axis_angle(axis, theta)
                revolute_idx += 1

        return T

    def calculate_forward_kinematics(self, joint_positions, joint_names, robot):
        """
        FK in the UR base frame: the URDF base_link -> tool0 chain product.
        """
        return self._fk_raw(joint_positions, joint_names, robot)

    # Inverse kinematics  (Newton with scipy Rotation error + adaptive step)
    def inverse_kinematics(self, target_pos, target_rot, q_init, joint_names, robot,
                           max_iterations=500, tolerance=5e-3):
        """
        Newton-Raphson IK with:
        - Correct 6D error: position + rotation vector (scipy, singularity-free)
        - Damped least-squares (pseudo-inverse with regularisation)
        - Adaptive step size to avoid overshooting
        - Stall detection
        """
        q = q_init.copy()
        damping = 0.05
        prev_error = float('inf')
        stall_cnt = 0

        for i in range(max_iterations):
            T = self.calculate_forward_kinematics(q, joint_names, robot)
            error = self._compute_error(T, target_pos, target_rot)
            pos_error  = np.linalg.norm(error[:3])

            if i % 50 == 0:
                rospy.loginfo("  iter %d: pos_err=%.2f mm", i, pos_error * 1000)

            if pos_error < tolerance:
                rospy.loginfo("  Converged in %d iterations (%.2f mm)", i, pos_error * 1000)
                return q, True

            # Stall detection
            if abs(prev_error - pos_error) < 1e-7:
                stall_cnt += 1
                if stall_cnt > 30:
                    rospy.logwarn("  Stalled at iter %d (%.2f mm)", i, pos_error * 1000)
                    break
            else:
                stall_cnt = 0
            prev_error = pos_error

            # Numerical Jacobian (6×6)
            J = self._numerical_jacobian(q, target_pos, target_rot, joint_names, robot)

            # Damped least squares
            JtJ = J.T @ J
            delta_q = np.linalg.solve(JtJ + damping * np.eye(6), J.T @ error)

            # Adaptive step size
            max_step = 0.3 if pos_error > 0.1 else (0.2 if pos_error > 0.05 else 0.1)
            norm = np.linalg.norm(delta_q)
            if norm > max_step:
                delta_q *= max_step / norm

            q = q - delta_q  # Newton step: minimise error toward zero
            # Normalise to avoid numerical drift; proper unwrapping happens
            # in _unwrap_to_seed() after convergence (see run()).
            q = (q + np.pi) % (2 * np.pi) - np.pi

        success = pos_error < 0.02
        rospy.logwarn("  Did not converge. Final error: %.2f mm", pos_error * 1000)
        return q, success

    # Natural-posture IK (multi-seed + plausibility check)
    def _shoulder_is_natural(self, q):
        """
        True if the shoulder (joint 2) is in an upright, non-folded range.
        """
        s = (q[1] + np.pi) % (2 * np.pi) - np.pi      # wrap to [-pi, pi]
        return np.deg2rad(-150) < s < np.deg2rad(-30)

    def _wrist1_is_free(self, q):
        """
        True if wrist_1 is on the collision-free side. Confirmed in simulation
        """
        w1 = (q[3] + np.pi) % (2 * np.pi) - np.pi     # wrap to [-pi, pi]
        return np.deg2rad(-160) < w1 < np.deg2rad(-10)

    def _flip_wrist(self, q):
        """Other wrist branch with the SAME tool orientation:
        wrist1 += 180, wrist2 = -wrist2, wrist3 += 180. Used only as an IK seed
        (then re-solved) so the small UR wrist offsets can't move the TCP."""
        q2 = np.array(q, dtype=float)
        q2[3] += np.pi
        q2[4]  = -q2[4]
        q2[5] += np.pi
        return (q2 + np.pi) % (2 * np.pi) - np.pi

    def _solution_ok(self, q):
        return self._shoulder_is_natural(q) and self._wrist1_is_free(q)

    def solve_ik_natural(self, target_pos, target_rot, primary_seed,
                         joint_names, robot, max_iterations=500, tolerance=5e-3):
        """
        Solve IK preferring a natural, upright arm posture with the wrist on the
        collision-free side. Returns the first solution that is both shoulder-natural and wrist-free; 
        otherwise the best converged solution so the sequence still completes.
        """
        b  = primary_seed[0]   # keep base (target direction) from the chained seed
        w2 = primary_seed[4]   # keep wrist2 (gripper pointing down)
        w3 = primary_seed[5]   # keep wrist3

        # Reference seeds in the upright basin (shoulder/elbow/wrist1 varied).
        seeds = [
            primary_seed,
            np.array([b, -1.57, -1.20, -1.40, w2, w3]),
            np.array([b, -1.20, -1.60, -1.00, w2, w3]),
            np.array([b, -1.00,  1.20, -1.30, w2, w3]),
            np.array([b, -1.80,  1.00, -0.80, w2, w3]),
            np.array([b, -2.00, -1.00, -1.20, w2, w3]),
        ]

        fallback = None
        for seed in seeds:
            q, success = self.inverse_kinematics(target_pos, target_rot, seed,
                                                 joint_names, robot,
                                                 max_iterations, tolerance)
            if not success:
                continue

            if self._solution_ok(q):
                return q, True

            # Shoulder natural but wrist on the wrong side: Re-solve from the
            # flipped wrist branch so the flange clears the arm.
            if self._shoulder_is_natural(q) and not self._wrist1_is_free(q):
                q2, ok2 = self.inverse_kinematics(target_pos, target_rot,
                                                  self._flip_wrist(q),
                                                  joint_names, robot,
                                                  max_iterations, tolerance)
                if ok2 and self._solution_ok(q2):
                    return q2, True

            if fallback is None:
                fallback = q   # remember a converged but not ideal solution

        if fallback is not None:
            rospy.logwarn("  No natural+wrist-free solution found — using best "
                          "converged solution. Check it in simulation.")
            return fallback, True
        return q, False

    # Angle unwrapping — shortest path from seed
    def _unwrap_to_seed(self, q, q_seed):
        """
        For each joint, shift by the nearest multiple of 2pi so the result
        stays as close as possible to q_seed.
        """
        q_out = q.copy()
        for i in range(len(q)):
            diff     = q[i] - q_seed[i]
            q_out[i] = q[i] - np.round(diff / (2 * np.pi)) * (2 * np.pi)
        return q_out

    # Error function (6D: position + rotation vector)
    def _compute_error(self, T, target_pos, target_rot):
        pos_err = T[:3, 3] - target_pos
        ist_rot = Rotation.from_matrix(T[:3, :3])
        rot_err = (target_rot * ist_rot.inv()).as_rotvec()
        return np.concatenate([pos_err, rot_err])

    # Numerical Jacobian
    def _numerical_jacobian(self, q, target_pos, target_rot, joint_names, robot, eps=1e-4):
        J = np.zeros((6, 6))
        T0 = self.calculate_forward_kinematics(q, joint_names, robot)
        error0  = self._compute_error(T0, target_pos, target_rot)

        for i in range(6):
            q_d = q.copy()
            q_d[i] += eps
            T_d = self.calculate_forward_kinematics(q_d, joint_names, robot)
            error_d = self._compute_error(T_d, target_pos, target_rot)
            J[:, i] = (error_d - error0) / eps

        return J

    
    # Matrix helpers

    def _transformation_matrix(self, xyz, rpy):
        roll, pitch, yaw = rpy
        Rx = np.array([[1, 0,              0             ],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll),  math.cos(roll)]])
        Ry = np.array([[ math.cos(pitch), 0, math.sin(pitch)],
                       [0,                1, 0              ],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw),  math.cos(yaw), 0],
                       [0,              0,             1]])
        T = np.eye(4)
        T[:3, :3] = Rz @ Ry @ Rx
        T[:3,  3] = xyz
        return T

    def _rotation_matrix_axis_angle(self, axis, angle):
        ax = np.array(axis, dtype=float)
        ax /= np.linalg.norm(ax)
        x, y, z = ax
        c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
        T = np.eye(4)
        T[:3, :3] = np.array([[t*x*x+c,   t*x*y-s*z, t*x*z+s*y],
                               [t*x*y+s*z, t*y*y+c,   t*y*z-s*x],
                               [t*x*z-s*y, t*y*z+s*x, t*z*z+c  ]])
        return T

    # ROS message helpers

    def get_pose_message_from_matrix(self, matrix):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.header.stamp    = rospy.Time.now()
        ps.pose.position.x = matrix[0][3]
        ps.pose.position.y = matrix[1][3]
        ps.pose.position.z = matrix[2][3]
        q = self.get_quaternion_from_matrix(matrix)
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        return ps

    def get_frame_from_matrix(self, matrix):
        tf = TransformStamped()
        tf.header.frame_id  = "base_link"
        tf.header.stamp     = rospy.Time.now()
        tf.child_frame_id   = "target_frame"
        tf.transform.translation.x = matrix[0][3]
        tf.transform.translation.y = matrix[1][3]
        tf.transform.translation.z = matrix[2][3]
        q = self.get_quaternion_from_matrix(matrix)
        tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
        tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
        return tf

    def get_quaternion_from_matrix(self, matrix):
        q = np.empty((4,), dtype=np.float64)
        M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
        t = np.trace(M)
        if t > M[3, 3]:
            q[3] = t
            q[2] = M[1, 0] - M[0, 1]
            q[1] = M[0, 2] - M[2, 0]
            q[0] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]: i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]: i, j, k = 2, 0, 1
            t    = M[i,i] - (M[j,j] + M[k,k]) + M[3,3]
            q[i] = t
            q[j] = M[i,j] + M[j,i]
            q[k] = M[k,i] + M[i,k]
            q[3] = M[k,j] - M[j,k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
        return q


# Standalone entry point

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Unified Pick and Place Kinematics')
    parser.add_argument('--pick_x',  type=float, required=True,  help='Pick X in meters')
    parser.add_argument('--pick_y',  type=float, required=True,  help='Pick Y in meters')
    parser.add_argument('--place_x', type=float, required=True,  help='Place X in meters')
    parser.add_argument('--place_y', type=float, required=True,  help='Place Y in meters')
    parser.add_argument('--height',  type=float, required=True,  help='Object height in meters')
    parser.add_argument('--width',   type=float, required=True,  help='Gripper opening in mm')
    args = parser.parse_args()

    kin = Kinematics()
    waypoints = kin.run(
        pick_x=args.pick_x,
        pick_y=args.pick_y,
        place_x=args.place_x,
        place_y=args.place_y,
        object_height=args.height,
        gripper_width_mm=args.width,
    )

    if waypoints:
        for name, angles in waypoints.items():
            rospy.loginfo("%s: %s", name, np.round(np.rad2deg(angles), 2))
