#! /usr/bin/env python3

import rospy
import socket
from sensor_msgs.msg import JointState


class RealRobotArm:

    def __init__(self):

        host = rospy.get_param("robot_ip")
        port_ur = 30002
        port_gripper = 63352

        rospy.init_node('my_real_robot')
        rospy.sleep(3.0)
        # Create socket connection to robot arm and gripper
        self.socket_ur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_ur.connect((host, port_ur))
        self.socket_gripper = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_gripper.connect((host, port_gripper))
        # activate the gripper
        self.socket_gripper.sendall(b'SET ACT 1\n')
        rospy.Subscriber('my_joint_states', JointState, self.callback)


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
        self.close_connection()

    def callback(self, msg):
        joint_values = msg.position[:6]
        self.send_joint_command(joint_values) 

    def send_joint_command(self, joint_angles):
        values = ', '.join(['{:.2f}'.format(i) if type(i) == float else str(i) for i in joint_angles])
        self.socket_ur.send (str.encode("movej(["+ values + "])\n"))

    def send_gripper_command(self, value):
        if (value >= 0 and value <= 255):
            command = 'SET POS ' + str(value) + '\n'
            self.socket_gripper.send(str.encode(command))
            # make the gripper move
            self.socket_gripper.send(b'SET GTO 1\n')


    def close_connection(self):
        self.socket_ur.close()
        self.socket_gripper.close()

if __name__ == '__main__':
    robot = RealRobotArm()
    robot.run()