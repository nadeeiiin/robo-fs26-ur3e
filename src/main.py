#! /usr/bin/env python3

# version 28.6.26 03

import rospy
import argparse
from kinematics import Kinematics
from move_real_robot import RealRobotArm


def main():
    rospy.init_node('pick_and_place', anonymous=False)

    parser = argparse.ArgumentParser(description='Pick and Place')
    parser.add_argument('--pick_x',  type=float, required=True, help='Pick X in meters')
    parser.add_argument('--pick_y',  type=float, required=True, help='Pick Y in meters')
    parser.add_argument('--place_x', type=float, required=True, help='Place X in meters')
    parser.add_argument('--place_y', type=float, required=True, help='Place Y in meters')
    parser.add_argument('--height',  type=float, required=True, help='Object height in meters')
    parser.add_argument('--width',   type=float, required=True, help='Gripper opening in mm')
    args = parser.parse_args()

    rospy.loginfo("PICK AND PLACE")
    rospy.loginfo("Pick:  X=%.0fmm Y=%.0fmm", args.pick_x*1000,  args.pick_y*1000)
    rospy.loginfo("Place: X=%.0fmm Y=%.0fmm", args.place_x*1000, args.place_y*1000)
    rospy.loginfo("Height: %.0fmm  Gripper: %.0fmm", args.height*1000, args.width)

    kinematics = Kinematics()
    waypoints  = kinematics.run(
        pick_x=args.pick_x,
        pick_y=args.pick_y,
        place_x=args.place_x,
        place_y=args.place_y,
        object_height=args.height,
        gripper_width_mm=args.width,
    )

    # 8 waypoints now: home, start, pick_approach, pick, pick_retreat,
    # place_approach, place, place_retreat
    if waypoints is None or len(waypoints) != 8:
        rospy.logerr("Failed to calculate joint angles")
        return

    robot = RealRobotArm()

    # Begin: upright home -> safe start pose (largest swings, give them time)
    rospy.loginfo("0. Home (upright)"); robot.send_joint_command(waypoints['home']);          rospy.sleep(5.0)
    rospy.loginfo("1. Safe start");     robot.send_joint_command(waypoints['start']);         rospy.sleep(5.0)

    # Pick and place
    rospy.loginfo("2. Open gripper");   robot.send_gripper_command(85);                       rospy.sleep(2.0)
    rospy.loginfo("3. Pick approach");  robot.send_joint_command(waypoints['pick_approach']); rospy.sleep(5.0)
    rospy.loginfo("4. Pick");           robot.send_joint_command(waypoints['pick']);          rospy.sleep(3.0)
    rospy.loginfo("5. Grip");           robot.send_gripper_command(args.width);               rospy.sleep(3.0)
    rospy.loginfo("6. Pick retreat");   robot.send_joint_command(waypoints['pick_retreat']);  rospy.sleep(3.0)
    rospy.loginfo("7. Place approach"); robot.send_joint_command(waypoints['place_approach']);rospy.sleep(5.0)
    rospy.loginfo("8. Place");          robot.send_joint_command(waypoints['place']);         rospy.sleep(3.0)
    rospy.loginfo("9. Release");        robot.send_gripper_command(85);                       rospy.sleep(3.0)
    rospy.loginfo("10. Place retreat"); robot.send_joint_command(waypoints['place_retreat']); rospy.sleep(3.0)

    # End: safe start pose -> upright home
    rospy.loginfo("11. Safe start");    robot.send_joint_command(waypoints['start']);         rospy.sleep(5.0)
    rospy.loginfo("12. Home (upright)");robot.send_joint_command(waypoints['home']);          rospy.sleep(5.0)

    rospy.loginfo("Done!")
    robot.close_connection()


if __name__ == '__main__':
    main()