#!/usr/bin/env python3
import rospy
from time import time
from utils.info import print_info

""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
date: 27.04.25
key components: STP Planner, Pure Pursuit, RRT*, ROS
robot: Turtlebot3
version: 1.2
authors: Group 3 - Tim Horstmann, Benjamin Thanh Tran, Emelie Mühlhans, Haig Conti Georges Sajelian
""" 

# AI planner
from ai_planner_modules.STP_Plan import STP_Plan

# GNC module and actions
from gnc_robot_modules.Controller import RobotController
from gnc_robot_modules.Manipulator import Manipulator
from gnc_robot_modules.actions import *


def main():
    # Sim or Real
    rospy.init_node('MissionExecution', anonymous=False)
    real_map = rospy.get_param('/real_map')
    try:
        rospy.loginfo("Environment is set.")
    except KeyError:
        rospy.loginfo("Environment is not set. Please run the launch file.")

    # Init GNC module components
    controller = RobotController(real_map=real_map)
    manipulator = Manipulator()

    # Action Executor
    action_executor = ActionExecutor(controller, manipulator, real_map)

    # AI Planner Setup
    planner = STP_Plan()

    # Generate the plan
    planner.run_planner(plan=False)
    
    # Reading the plan
    rospy.loginfo("Reading the plan from AI planner")
    plan_general = planner.parse_plan()

    # Mission Execution
    rospy.loginfo("Starting mission execution")

    # Wait for start signal
    info_stop = True
    if info_stop:
        print_info(real_map)
        input_t=input("")

    rospy.sleep(1)

    t0 = time.time() 
    
    # main execution loop
    for action in plan_general:
        
        action_executor.execute_action(action["action"])
        rospy.sleep(1.0)
  
    print("\n--------------------------------------")
    print("Mission successfully exectued.")

    # Metrics
    t1 = time.time()
    elapsed_time = t1 - t0
    print(f"Elapsed time: {elapsed_time:.2f} seconds")

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Mission execution terminated.")