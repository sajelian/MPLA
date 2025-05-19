#!/usr/bin/env python3
import time
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander

class Manipulator():
    def __init__(self):
        # initialize MoveIt interfaces
        self.arm_group     = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")

    def open_gripper(self):
        self.gripper_group.set_named_target("open")
        self.gripper_group.go(wait=True)

    def close_gripper(self):
        self.gripper_group.set_named_target("close")
        self.gripper_group.go(wait=True)

    def move_arm_to_home(self):
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)

    def move_arm_to_ready(self):
        self.arm_group.set_named_target("ready")
        self.arm_group.go(wait=True)

def main():
    rospy.init_node("manipulator_test", anonymous=True)
    manip = Manipulator()

    # give everything a moment to initialize
    rospy.sleep(2.0)

    rospy.loginfo("Moving arm to 'home'")
    manip.move_arm_to_home()
    rospy.sleep(1.0)

    rospy.loginfo("Moving arm to 'ready'")
    manip.move_arm_to_ready()
    rospy.sleep(1.0)

    rospy.loginfo("Closing gripper")
    manip.close_gripper()
    rospy.sleep(1.0)

    rospy.loginfo("Opening gripper")
    manip.open_gripper()
    rospy.sleep(1.0)

    rospy.loginfo("Done. Returning to 'home'")
    manip.move_arm_to_home()

if __name__ == "__main__":
    main()