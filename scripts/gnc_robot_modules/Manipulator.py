import time
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from gnc_robot_modules.actions import rotate_to_obstacle
import tf

class Manipulator():
    def __init__(self):
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")
        self.arm_group.set_planning_time(10.0)
        self.gripper_group.set_planning_time(10.0)
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)

    def open_gripper(self):
        self.gripper_group.set_named_target("open")
        self.gripper_group.go(wait=True)    

    def open_gripper_direct(self):
        joint_vals = self.gripper_group.get_current_joint_values()
        joint_vals[0] = 0.01  
        self.gripper_group.set_joint_value_target(joint_vals)
        self.gripper_group.go(wait=True)

    def close_gripper(self):
        self.gripper_group.set_named_target("close")
        self.gripper_group.go(wait=True)

    def close_gripper_direct(self):
        joint_vals = self.gripper_group.get_current_joint_values()
        joint_vals[0] = -0.01   
        self.gripper_group.set_joint_value_target(joint_vals)
        self.gripper_group.go(wait=True)

    def move_arm_to_home(self):
        self.arm_group.set_named_target("home")
        self.arm_group.go(wait=True)

    def move_arm_to_home_direct(self):
        joint_vals = self.arm_group.get_current_joint_values()
        joint_vals[0] =  0.0   # joint1
        joint_vals[1] = -1.0   # joint2
        joint_vals[2] =  0.3   # joint3
        joint_vals[3] =  0.7   # joint4
        self.arm_group.set_joint_value_target(joint_vals)
        self.arm_group.go(wait=True)

    def move_arm_to_ready(self):
        self.arm_group.set_named_target("ready")
        self.arm_group.go(wait=True)
    
    def move_arm_to_ready_direct(self):
        joint_vals = self.arm_group.get_current_joint_values()
        joint_vals[0] =  0.0   # joint1
        joint_vals[1] = 0.53  # joint2
        joint_vals[2] =  -0.83   # joint3
        joint_vals[3] =  0.28   # joint4
        self.arm_group.set_joint_value_target(joint_vals)
        self.arm_group.go(wait=True)

    def manipulate_valve(self, controller, waypoint):
        #position correctly before manipulating
        rotate_to_obstacle(controller, waypoint)
        time.sleep(2)

        print(f"Executing manipulate a valve at {waypoint} ...")

        # get arm into position
        rospy.loginfo("Moving arm to 'ready' position")
        self.move_arm_to_ready_direct()
        rospy.sleep(5.0)

        # close at valve
        rospy.loginfo("Closing gripper")
        self.close_gripper_direct()
        rospy.sleep(3.0)

        # open back up
        rospy.loginfo("Opening gripper")
        self.open_gripper_direct()
        rospy.sleep(3.0)

        # return to home
        rospy.loginfo("Returning arm to 'home' position")
        self.move_arm_to_home_direct()
        rospy.sleep(3.0)