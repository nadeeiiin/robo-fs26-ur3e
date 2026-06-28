#! /usr/bin/env python3
# Version 28.6.26 01

import rospy
import argparse
import numpy as np
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

from kinematics import Kinematics


class PickAndPlaceSim(Kinematics):

    INTERP_STEPS = 50   # steps per segment
    STEP_DELAY   = 0.05  # seconds per step (= ~2.5s per move)

    def __init__(self):
        super().__init__()
        self.rate = rospy.Rate(1.0 / self.STEP_DELAY)

    def run(self, pick_x, pick_y, place_x, place_y, object_height, gripper_width_mm):
        robot       = URDF.from_parameter_server()
        root        = robot.get_root()
        joint_names = robot.get_chain(root, "tool0", joints=True, links=False, fixed=False)

        rospy.loginfo("Computing waypoints...")
        waypoints = super().run(
            pick_x=pick_x, pick_y=pick_y,
            place_x=place_x, place_y=place_y,
            object_height=object_height,
            gripper_width_mm=gripper_width_mm,
        )

        if waypoints is None:
            rospy.logerr("IK failed — cannot run simulation")
            return

        sequence = [
            (waypoints['start'],          waypoints['start'],          "Start position"),
            (waypoints['start'],          waypoints['pick_approach'],   "Home → Pick Approach"),
            (waypoints['pick_approach'],  waypoints['pick'],            "Pick Approach → Pick"),
            (waypoints['pick'],           waypoints['pick'],            ">>> GRIPPER CLOSE <<<"),
            (waypoints['pick'],           waypoints['pick_retreat'],    "Pick → Pick Retreat"),
            (waypoints['pick_retreat'],   waypoints['place_approach'],  "Pick Retreat → Place Approach"),
            (waypoints['place_approach'], waypoints['place'],           "Place Approach → Place"),
            (waypoints['place'],          waypoints['place'],           ">>> GRIPPER OPEN <<<"),
            (waypoints['place'],          waypoints['place_retreat'],   "Place → Place Retreat"),
            (waypoints['place_retreat'],  waypoints['start'],           "Place Retreat → Home"),
        ]

        for q_from, q_to, label in sequence:
            rospy.loginfo("=== %s ===", label)
            self._interpolate(q_from, q_to, joint_names, robot)

        rospy.loginfo("Simulation done — holding home.")
        while not rospy.is_shutdown():
            self._publish(waypoints['start'], joint_names, robot)
            self.rate.sleep()

    # ------------------------------------------------------------------

    def _interpolate(self, q_start, q_end, joint_names, robot):
        q_start, q_end = np.array(q_start), np.array(q_end)
        for i in range(self.INTERP_STEPS + 1):
            if rospy.is_shutdown():
                return
            t = i / self.INTERP_STEPS
            self._publish((1 - t) * q_start + t * q_end, joint_names, robot)
            self.rate.sleep()

    def _publish(self, joint_angles, joint_names, robot):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name         = joint_names
        js.position     = list(joint_angles)
        self.joint_state_publisher.publish(js)

        T = self.calculate_forward_kinematics(joint_angles, joint_names, robot)
        self.pose_publisher.publish(self.get_pose_message_from_matrix(T))
        self.transform_broadcaster.sendTransform(self.get_frame_from_matrix(T))


# ----------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Pick and Place Simulation')
    parser.add_argument('--pick_x',  type=float, required=True,  help='Pick X in meters')
    parser.add_argument('--pick_y',  type=float, required=True,  help='Pick Y in meters')
    parser.add_argument('--place_x', type=float, required=True,  help='Place X in meters')
    parser.add_argument('--place_y', type=float, required=True,  help='Place Y in meters')
    parser.add_argument('--height',  type=float, required=True,  help='Object height in meters')
    parser.add_argument('--width',   type=float, required=True,  help='Gripper opening in mm')
    args = parser.parse_args()

    sim = PickAndPlaceSim()
    sim.run(
        pick_x=args.pick_x,
        pick_y=args.pick_y,
        place_x=args.place_x,
        place_y=args.place_y,
        object_height=args.height,
        gripper_width_mm=args.width,
    )