#! /usr/bin/env python3

# version 28.6.26 04

import rospy
import socket


class RealRobotArm:

    def __init__(self):

        host = rospy.get_param("robot_ip")
        port_ur = 30002
        port_gripper = 63352

        # main_NF.py already calls rospy.init_node('pick_and_place'); calling
        # init_node again with a different name raises a ROSException. Guard it.
        if not rospy.core.is_initialized():
            rospy.init_node('my_real_robot')
        rospy.sleep(3.0)        
        # Create socket connection to robot arm and gripper
        self.socket_ur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_ur.connect((host, port_ur))
        self.socket_gripper = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_gripper.connect((host, port_gripper))
        # activate the gripper — this triggers a one-time open/close calibration
        # cycle. Wait for it to finish before any further command races it.
        self.socket_gripper.sendall(b'SET ACT 1\n')
        rospy.loginfo("Activating gripper (it will open/close once to calibrate)...")
        rospy.sleep(3.0)


    def send_joint_command(self, joint_angles, a=1.2, v=0.8):
        """movej with adjustable acceleration a (rad/s^2) and velocity v (rad/s).
        UR defaults are a=1.4, v=1.05 — kept lower here for safe early runs.
        Pass a/v per call to speed up free transfers or slow down approaches."""
        values = ', '.join(['{:.4f}'.format(float(i)) for i in joint_angles])
        command = "movej([{}], a={}, v={})\n".format(values, a, v)
        rospy.loginfo("Sending: %s", command.strip())
        self.socket_ur.send(str.encode(command))

    def send_gripper_command(self, opening_mm):
        """Convert a gripper opening in mm (0..85) to the Robotiq position
        count (0 = fully open, 255 = fully closed) and command it.
        main_NF.py passes mm: 85 to open, args.width to grasp."""
        if opening_mm < 0 or opening_mm > 85:
            rospy.logerr("Invalid gripper opening: %s mm (expected 0..85)", opening_mm)
            return
        gripper_value = int(255 - (opening_mm / 85.0 * 255))
        gripper_value = max(0, min(255, gripper_value))
        command = 'SET POS ' + str(gripper_value) + '\n'
        self.socket_gripper.send(str.encode(command))
        # make the gripper move
        self.socket_gripper.send(b'SET GTO 1\n')


    def close_connection(self):
        self.socket_ur.close()
        self.socket_gripper.close()

if __name__ == '__main__':
    robot = RealRobotArm()

    # send joint angles
    joint_angles = [0, -1.57, 0, 0, 0, 0] # upright position
    robot.send_joint_command(joint_angles)

    # send gripper command
    # opening in mm (0 = closed .. 85 = fully open)
    robot.send_gripper_command(40)
    robot.close_connection()