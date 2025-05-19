#!/usr/bin/env python3
import rospy
from math import pi
from time import time

#2) GNC module (path-followig and PID controller for the robot)
from gnc_robot_modules.Controller import RobotController

#4) Program here the turtlebot actions (based in your AI planner)
from gnc_robot_modules.actions import *

# 5) Program here the main commands of your mission planner code
def main():

    # 5.0) init GNC module components
    controller: RobotController = RobotController.RobotController(ctrl_type="flatness")# pure_pursuit, flatness
    algorithm = "RRTstar" # HybridAstar or RRTstar

    rospy.sleep(1) # wait for the controller to start
    t0 = time.time()

    move_robot(controller, "waypoint0", "waypoint2", algorithm, robot_pos=True)
    move_robot(controller, "waypoint2", "waypoint1", algorithm, robot_pos=True)
    move_robot(controller, "waypoint1", "waypoint0", algorithm, robot_pos=True)
                    
    t1 = time.time()

    print(f"Time taken: {t1-t0} seconds")

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")