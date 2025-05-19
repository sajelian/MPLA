from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import rospy
from datetime import datetime
from pathlib import Path
import shutil
import time
from geometry_msgs.msg import Twist
from math import atan2, pi
from pathfinding_modules.RRTstar import main_RRTstar
from pathfinding_modules.HybridAstar import main_hybrid_a, PLANNING_WAYPOINTS
import numpy as np
from gnc_robot_modules import *
import os


class TakePhoto:
    def __init__(self, real_map=True):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        if real_map:
            img_topic = "/camera/image" 
        else:
            img_topic = "/camera/rgb/image_raw" # the topic is different in simulation...

        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False
        
def taking_photo_exe(controller, waypoint: str):
    #position correctly before taking picture 
    rotate_to_obstacle(controller, waypoint)
    time.sleep(0.5)

    # Initialize
    camera = TakePhoto(controller.real_map)

    # Default value is 'photo.jpg'
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")
    # Build full path where we want to save image
    home = str(Path.home())
    
    ws_name = os.environ.get("ROS_WS_NAME", "group3_ws")
    save_folder = os.path.join(home, ws_name, 'src', 'assignment4_ttk4192', 'scripts')

    img_title = 'photo' + dt_string + '.jpg'
    full_path = os.path.join(save_folder, img_title)

    # Take and save the picture
    if camera.take_picture(full_path):
        rospy.loginfo("Saved image " + full_path)
    else:
        rospy.loginfo("No images received")
    rospy.sleep(1)


def rotate_to_obstacle(controller, waypoint: str,
                          tol: float = 0.01,
                          angular_speed: float = 0.5):
    """
    Rotate in place until controller.theta is within tol of the
    map-defined heading for `waypoint` (from parse_waypoints).
    """
    # parse_waypoints returns ([x,y,theta], [x,y,theta])
    _, end_pose = parse_waypoints(waypoint, waypoint)
    target_theta = end_pose[2]
    rospy.loginfo(f"[rotate_to_obstacle] {waypoint} → target θ={target_theta:.2f} rad")
    max_time = 5.0
    max_iter = int(max_time / 0.1) #TODO Dont hardcode
    i = 0
    while not rospy.is_shutdown():
        i += 1
        err = normalize_angle(target_theta - controller.theta)
        if abs(err) < tol:
            break
        twist = Twist()
        Kp = 1
        twist.angular.z = max(-angular_speed, min(angular_speed, Kp*err))
        controller.cmd_vel_pub.publish(twist)
        controller.rate.sleep()
        if i > max_iter:
            break

    controller.stop()

def check_pump_picture_ir_waypoint(controller, waypoint: str):
    #position correctly before taking picture
    rotate_to_obstacle(controller, waypoint)
    taking_photo_exe(controller, waypoint)

    print(f"Taking IR picture at {waypoint} ...")
    a=0
    while a<3:
        time.sleep(1)
        a=a+1
    time.sleep(0.5)

def check_seals_valve_picture_eo_waypoint(controller, waypoint: str):
    #position correctly before taking picture 
    rotate_to_obstacle(controller, waypoint)
    taking_photo_exe(controller, waypoint)

    print(f"Taking EO picture at {waypoint} ...")
    a=0
    while a<3:
        time.sleep(1)
        a=a+1
    time.sleep(0.5)

# Charging battery 
def charge_battery_waypoint(controller, waypoint: str):
    rotate_to_obstacle(controller, waypoint)

    print(f"charging battery at {waypoint} ...")
    time.sleep(0.5)

def parse_waypoints(start_waypoint: str, end_waypoint: str, real=True):
    offset = 0.0
    if real:
        waypoints_mapping = {
            "waypoint0": [0.3+offset, 0.3+offset, 0],
            "waypoint1": [1.75, 0.7-offset, pi/2],
            "waypoint2": [3.6, 0.9+offset, -pi/2],
            "waypoint3": [3.31, 2.75-offset, pi/2],
            "waypoint4": [5.21-offset, 0.2+offset, pi/2],
            "waypoint5": [0.95+offset, 2.40, pi/2],
            "waypoint6": [3.85, 1.75-offset, pi/2]
            }
    else:
        waypoints_mapping = {
            "waypoint0": [0.2+offset, 0.1+offset, 0],
            "waypoint1": [1.6, 0.6-offset, pi/2],
            "waypoint2": [3.31, 0.9+offset, -pi/2],
            "waypoint3": [3.21, 2.75-offset, pi/2],
            "waypoint4": [5.21-offset, 0.3+offset, pi/2],
            "waypoint5": [0.8+offset, 2.45, pi/2],
            "waypoint6": [3.91, 1.75-offset, pi/2]
            }
        
    # Convert waypoints to positions
    return waypoints_mapping[start_waypoint], waypoints_mapping[end_waypoint]


def normalize_angle(angle):
    """Wrap to [-pi, +pi]."""
    return (angle + pi) % (2*pi) - pi

_ROTATE_POINTS = {"waypoint0", "waypoint1", "waypoint2", "waypoint5", "waypoint6"}

def rotate_to_path(controller, waypoints, start_waypoint, tol=0.05, angular_speed=0.5):
    """
    Rotate in place to face the first segment of the given waypoints.
    
    controller: our RobotController instance
    waypoints: list of [x, y] positions (at least one entry)
    tol: angular error tolerance (rad)
    angular_speed: spin speed (rad/s)
    """

    if start_waypoint not in _ROTATE_POINTS:
        rospy.loginfo(f"Skipping spin at '{start_waypoint}'")
        return

    # Compute desired yaw toward the first waypoint
    first_wp = waypoints[20]
    dx = first_wp[0] - controller.position[0]
    dy = first_wp[1] - controller.position[1]
    target_yaw = atan2(dy, dx)
    rospy.loginfo(f"Rotating to heading {target_yaw:.2f} rad")

    # In-place spin until aligned
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        err = normalize_angle(target_yaw - controller.theta)
        if abs(err) < tol:
            break
        twist = Twist()
        twist.angular.z = angular_speed if err > 0 else -angular_speed
        pub.publish(twist)
        rate.sleep()

    # stop rotation
    pub.publish(Twist())
    rospy.sleep(0.2)

def move_robot(controller, start_waypoint: str, end_waypoint: str, robot_pos=False, real_map=True):
    plot = False
    start_pos, end_pos = parse_waypoints(start_waypoint, end_waypoint, real=real_map)
    if robot_pos:
        start_pos = controller.position
    
    rospy.loginfo(f"Next waypoint: {end_waypoint}")
    rospy.loginfo("Calculating new path...")

    for i in range(5):
        rospy.loginfo(f"Trying to find a path. Attempt {i+1}...")
        x_spline, y_spline, T_sim, waypoints = main_RRTstar(start_pos, end_pos, stepsize=0.1, max_iter=1500, radius_to_goal=0.1, real_map=real_map, plot=plot)
        
        if len(waypoints) > 2:
            break

    if waypoints is None:
        rospy.loginfo("No path found")
        return
    
    #let robot turn into direction of path first to avoid collision
    rotate_to_path(controller, waypoints, start_waypoint)
    
    controller.follow_path(x_spline, y_spline, T_sim, waypoints[-1], plot=plot)

class ActionExecutor:
    def __init__(self, controller, manipulator, real_map=True):
        self.real_map = real_map
        self.controller = controller
        self.manipulator = manipulator

    def execute_action(self, action):
        if action[0] == "move_robot_high_to_medium":
            move_robot(self.controller, action[2], action[3], robot_pos=True, real_map=self.real_map)
        elif action[0] == "move_robot_medium_to_low":
            move_robot(self.controller, action[2], action[3], robot_pos=True, real_map=self.real_map)
        elif action[0] == "move_robot_low_to_critical":
            move_robot(self.controller, action[2], action[3], robot_pos=True, real_map=self.real_map)
        elif action[0] == "charge_critical_battery":
            charge_battery_waypoint(self.controller, action[2])
        elif action[0] == "manipulate_valve":
            self.manipulator.manipulate_valve(self.controller, action[2])
        elif action[0] == "check_seals_valve_picture_eo":
            check_seals_valve_picture_eo_waypoint(self.controller, action[2])
        elif action[0] == "check_pump_picture_ir":
            check_pump_picture_ir_waypoint(self.controller, action[2])
        elif action[0] == "photograph_subject":
            taking_photo_exe(self.controller, action[2])
        else:
            print("Unhandled action detected")
