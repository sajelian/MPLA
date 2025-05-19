#!/usr/bin/env python3
"""
Waypoint following controller with sequential waypoint following capabilities.

This module provides a controller for autonomous robots with differential drive
systems, supporting both pure pursuit and flatness-based control methods.
"""

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from gnc_robot_modules.PID_Controller import PID_Controller
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class RobotController():
    """
    Waypoint following controller with sequential waypoint following.
    
    This controller implements differential flatness-based control
    """
    def __init__(self, real_map=True):
        """
        Initialize the robot controller.
        
        Args:
            lookahead_distance (float): Look-ahead distance for trajectory following
        """
        
        # Robot state
        self.real_map = real_map
        self.position = np.array([0.0, 0.0])
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Controller parameters
        self.lookahead_distance = 0.1
        
        # Target and path parameters
        self.current_waypoint = None  # Store only a single waypoint
        self.goal_reached = False
        self.waypoint_reached_threshold = 0.05  # meters

        # Robot constraints
        self.speed_limit = 0.26  # m/s
        self.angular_speed_limit = 1.82  # rad/s
        

        # PID Controllers
        self.theta_pid = PID_Controller(
            6.0, 0, 0.0, 
            output_limits=(-self.angular_speed_limit, self.angular_speed_limit)
        )
        self.theta_pid.is_angle = True
        
        self.long_pid = PID_Controller(
            1.0, 0.0, 0.0,
            output_limits=(0.0, self.speed_limit)
        )

        self.lat_pid = PID_Controller(
            6.0, 0.0, 0.0, 
            output_limits=(0.0, self.speed_limit)
        )
        
        # ROS communication
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz control loop
    
    def odom_callback(self, msg):
        """
        Update robot state from odometry messages.
        
        Args:
            msg (Odometry): ROS odometry message
        """
        # would be better to move the map origin
        if self.real_map:
            self.position[0] = msg.pose.pose.position.x + 0.25 
            self.position[1] = msg.pose.pose.position.y + 0.20
        else:
            self.position[0] = msg.pose.pose.position.x
            self.position[1] = msg.pose.pose.position.y
        
        # Convert quaternion to Euler angles
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, self.theta = euler_from_quaternion(quaternion)
        
        # Record velocities
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
    
    def set_waypoint(self, waypoint):
        """
        Set end point of the trajectory.
        
        Args:
            waypoint (array-like): Target position [x, y]
        """
        self.current_waypoint = waypoint
        self.goal_reached = False

    def move_to_point(self):
        """
        Move the robot to a specified point using PID control.
        """
        diff_x = self.current_waypoint[0] - self.position[0]
        diff_y = self.current_waypoint[1] - self.position[1]
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/np.sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = np.arctan2(diff_y, diff_x)

        self.theta_pid.set_setpoint(theta)

        # Adjust orientation first
        while not rospy.is_shutdown():
            angular_vel = self.theta_pid.update(self.theta)
            if abs(angular_vel) > 0.2:
                angular_vel = angular_vel/abs(angular_vel)*0.2
            if abs(self.theta - theta < 0.01):
                break
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(vel_cmd)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.theta_pid.set_setpoint(theta)

        # Move to the target point
        while not rospy.is_shutdown():
            diff_x = self.current_waypoint[0] - self.position[0]
            diff_y = self.current_waypoint[1] - self.position[1]
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) # projection
            if abs(linear) > 0.2:
                linear = linear/abs(linear)*0.2

            angular = self.theta_pid.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2

            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            vel_cmd = Twist()
            vel_cmd.linear.x = 1.0*linear
            vel_cmd.angular.z = angular
            self.cmd_vel_pub.publish(vel_cmd)
            self.rate.sleep()
        self.stop()
    
    def trajectory_precompute(self, x_spline, y_spline, T_sim):
        """
        Precompute trajectory for flatness-based controller.
        
        Args:
            x_spline: Cubic spline for x-coordinates
            y_spline: Cubic spline for y-coordinates
            T_sim (float): Simulatin Duration
            
        Returns:
            dict: Precomputed trajectory data
        """
        # # Generate trajectory
        num_steps = 1000
        t = np.linspace(0, T_sim, num_steps)
        x_ref = x_spline(t)
        y_ref = y_spline(t)
        
        # Calculate derivatives
        x_dot_ref = x_spline(t, 1)
        y_dot_ref = y_spline(t, 1)
        x_ddot_ref = x_spline(t, 2)
        y_ddot_ref = y_spline(t, 2)
        
        # Calculate reference theta
        theta_ref = np.arctan2(y_dot_ref, x_dot_ref)
        
        # Calculate control inputs
        v_ref = np.zeros(num_steps)
        omega_ref = np.zeros(num_steps)

        # Avoid division by zero
        epsilon = 1e-3

        # Calculate linear and angular velocities
        for i in range(num_steps):
            # Linear velocity is the magnitude of the velocity vector
            v_ref[i] = np.sqrt(x_dot_ref[i]**2 + y_dot_ref[i]**2)
            
            # Angular velocity calculation
            denominator = x_dot_ref[i]**2 + y_dot_ref[i]**2
            if denominator > epsilon:
                omega_ref[i] = (y_ddot_ref[i] * x_dot_ref[i] - x_ddot_ref[i] * y_dot_ref[i]) / denominator
            else:
                omega_ref[i] = 0.0

        # smooth omega_ref
        omega_ref = np.convolve(omega_ref, np.ones(10)/10, mode='same')
        
        # Package data for controller
        precomputed_data = {
            'x_ref': x_ref,
            'y_ref': y_ref,
            'theta_ref': theta_ref,
            'v_ref': v_ref,
            'omega_ref': omega_ref,
            't': t
        }
        
        return precomputed_data
    
    def update_setpoints(self, e_long, e_lat, e_theta):
        self.long_pid.set_setpoint(0.0)
        self.long_pid.update(e_long)
        self.lat_pid.set_setpoint(0.0)
        self.lat_pid.update(e_lat)
        self.theta_pid.set_setpoint(0.0)
        self.theta_pid.update(e_theta)  

    def compute_control(self, precomputed_data=None):
        """
        Flatness-based controller for differential drive robots using the unicycle model.
        
        Args:
            precomputed_data (dict): Dictionary containing precomputed trajectory data
                with keys: 'x_ref', 'y_ref', 'theta_ref', 'v_ref', 'omega_ref', etc.
        
        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        if self.current_waypoint is None or self.goal_reached or precomputed_data is None:
            return 0.0, 0.0
        
        # Extract reference trajectories
        x_ref = precomputed_data['x_ref']
        y_ref = precomputed_data['y_ref']
        v_ref = precomputed_data['v_ref']
        omega_ref = precomputed_data['omega_ref']
        theta_ref = precomputed_data['theta_ref']

        v_ref[-1] = 0
        
        # Calculate distances to all points on the reference trajectory
        distances = np.sqrt((x_ref - self.position[0])**2 + (y_ref - self.position[1])**2)
        
        # Find index of closest point
        idx = np.argmin(distances)

        # Look ahead a few indices for better tracking (helps with curves)
        lookahead_idx = min(idx + int(10 * self.lookahead_distance), len(x_ref) - 1) #TODO tune this
        
        # Extract feedforward control terms
        v_ff = v_ref[lookahead_idx]
        omega_ff = omega_ref[lookahead_idx]
        
        # Calculate tracking errors
        e_x = x_ref[lookahead_idx] - self.position[0]
        e_y = y_ref[lookahead_idx] - self.position[1]
        e_theta = theta_ref[lookahead_idx] - self.theta
        
        # Normalize angle to [-pi, pi]
        if e_theta > np.pi:
            e_theta -= 2 * np.pi
        elif e_theta < -np.pi:
            e_theta += 2 * np.pi
        
        # Transform errors to robot frame
        e_long = e_x * np.cos(self.theta) + e_y * np.sin(self.theta)  # longitudinal error
        e_lat = -e_x * np.sin(self.theta) + e_y * np.cos(self.theta)  # lateral error

        # Update setpoints for PID controllers
        self.update_setpoints(e_long, e_lat, e_theta)

        long_correction = self.long_pid.update(-e_long)
        lat_correction = self.lat_pid.update(-e_lat)
        heading_correction = self.theta_pid.update(-e_theta)
        
        # Compute control with feedforward and feedback terms
        v = v_ff + long_correction
        omega = omega_ff + lat_correction + heading_correction
        
        # Limit velocities to robot constraints
        v = np.clip(v, 0, self.speed_limit)
        omega = np.clip(omega, -self.angular_speed_limit, self.angular_speed_limit)
        
        return v, omega
    
    def follow_trajectory(self, precomputed_data, plot=False):
        """
        Follow precomputed trajectory.
        
        Args:
            precomputed_data (dict): Dictionary of precomputed trajectory data
            
        Returns:
            bool: True if waypoint reached successfully
        """
        # Reset goal flag
        self.goal_reached = False
        success = False
        iterations = 0
        max_iterations = 1.05*precomputed_data['t'][-1]/0.1 #TODO check if it's enough


        x_pos = []
        y_pos = []
        theta_pos = []
        k = []

        # Main control loop        
        while not rospy.is_shutdown() and not self.goal_reached:
            
            if plot:
                # Store current position for plotting
                x_pos.append(self.position[0])
                y_pos.append(self.position[1])
                theta_pos.append(self.theta)
                k.append(iterations)

            iterations += 1
            # Compute control signals using flatness-based controller
            linear_vel, angular_vel = self.compute_control(precomputed_data)
            
            # Create and publish velocity command
            vel_cmd = Twist()
            vel_cmd.linear.x = linear_vel
            vel_cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(vel_cmd)

            # Check if the robot has reached the waypoint
            if self.current_waypoint is not None:
                distance_to_waypoint = np.sqrt(np.sum((np.array(self.current_waypoint) - self.position)**2))
                if distance_to_waypoint < 0.075:
                    print(f"Waypoint reached! Distance: {distance_to_waypoint:.3f}")
                    self.goal_reached = True
                    success = True
        
            if iterations >= max_iterations and not self.goal_reached:
                print(f"WARNING: Maximum iterations ({max_iterations}) reached")
                success = False
                break

            # Sleep to maintain control rate
            self.rate.sleep()
        
        # Stop the robot when done
        self.stop()
        trajectory = {"x": x_pos, "y": y_pos, "theta": theta_pos, "k": k}
        return success, trajectory
    
    def follow_path(self, x_spline, y_spline, T_sim, endpoint, plot=False):
        """
        Follow planar path using flatness-based control.
        """
        # Precompute feed-forward data for flatness control
        precomputed_data = self.trajectory_precompute(x_spline, y_spline, T_sim)

        if plot:
            omega_ref = precomputed_data['omega_ref']
            v_ref = precomputed_data['v_ref']

            plt.plot(precomputed_data['t'], omega_ref, label='omega_ref')
            plt.plot(precomputed_data['t'], v_ref, label='v_ref')
            plt.ylim(-3.0, 3.0) #2.6 should suffice but idk...
            plt.show()

        self.set_waypoint(endpoint)

        success, trajectory = self.follow_trajectory(precomputed_data, plot=plot)

        # Benchmark
        if plot:
            metrics = benchmark_controller_performance(precomputed_data, trajectory)

        if success:
            None
        else:
            self.set_waypoint(endpoint)
            print("Waypoint not reached, going straight to waypoint")
            rospy.sleep(0.5)
            self.move_to_point()
    
    def stop(self):
        """Stop the robot by publishing zero velocity commands."""
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        
        # Send stop command several times to ensure it's received
        for _ in range(3):
            self.cmd_vel_pub.publish(vel_cmd)
            rospy.sleep(0.1)

# Benchmarking function for tuning
def benchmark_controller_performance(precomputed_data, trajectory):
    """
    Benchmark controller performance by comparing reference and actual trajectories.
    
    Args:
        precomputed_data: Dictionary containing reference trajectory data
            with keys 'x_ref', 'y_ref', 't', etc.
        trajectory: Dictionary containing actual trajectory data
            with keys 'x', 'y', 'k', etc.
    """
    # Extract data
    t_ref = precomputed_data['t']
    x_ref = precomputed_data['x_ref']
    y_ref = precomputed_data['y_ref']
    
    k = np.array(trajectory['k'])
    x_actual = np.array(trajectory['x'])
    y_actual = np.array(trajectory['y'])
    
    # Convert iteration indices to time
    # Estimate time step from average control loop rate
    estimated_dt = t_ref[-1] / k[-1]
    t_actual = k * estimated_dt
    
    # Create figure for time-domain comparisons
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    # X position comparison
    ax1.plot(t_ref, x_ref, 'b-', label='Reference x')
    ax1.plot(t_actual, x_actual, 'r--', label='Actual x')
    ax1.set_ylabel('X Position (m)')
    ax1.legend()
    ax1.grid(True)
    
    # Y position comparison
    ax2.plot(t_ref, y_ref, 'b-', label='Reference y')
    ax2.plot(t_actual, y_actual, 'r--', label='Actual y')
    ax2.set_ylabel('Y Position (m)')
    ax2.legend()
    ax2.grid(True)
    
    # Calculate position error over time
    # Interpolate reference to match actual time points
    x_ref_interp = interp1d(t_ref, x_ref, bounds_error=False, fill_value='extrapolate')
    y_ref_interp = interp1d(t_ref, y_ref, bounds_error=False, fill_value='extrapolate')
    
    x_ref_at_actual = x_ref_interp(t_actual)
    y_ref_at_actual = y_ref_interp(t_actual)
    
    # Calculate Euclidean error
    position_error = np.sqrt((x_actual - x_ref_at_actual)**2 + (y_actual - y_ref_at_actual)**2)
    
    # Plot position error
    ax3.plot(t_actual, position_error, 'g-', label='Position Error')
    ax3.set_ylabel('Error (m)')
    ax3.set_xlabel('Time (s)')
    ax3.legend()
    ax3.grid(True)
    
    # Add overall title
    fig.suptitle('Controller Performance: Reference vs. Actual Trajectory', fontsize=16)
    plt.tight_layout()
    
    # Calculate performance metrics
    mean_error = np.mean(position_error)
    max_error = np.max(position_error)
    rms_error = np.sqrt(np.mean(position_error**2))
    
    # Create XY path comparison plot
    plt.figure(figsize=(8, 8))
    plt.plot(x_ref, y_ref, 'b-', linewidth=2, label='Reference Path')
    plt.plot(x_actual, y_actual, 'r--', linewidth=2, label='Actual Path')
    plt.scatter(x_ref[0], y_ref[0], color='green', s=100, marker='o', label='Start')
    plt.scatter(x_ref[-1], y_ref[-1], color='red', s=100, marker='x', label='End')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.title('Path Comparison: Reference vs. Actual')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    
    # Display performance metrics
    print(f"Controller Performance Metrics:")
    print(f"Mean Position Error: {mean_error:.4f} m")
    print(f"Maximum Position Error: {max_error:.4f} m")
    print(f"RMS Position Error: {rms_error:.4f} m")
    
    plt.show()
    
    return {
        'mean_error': mean_error,
        'max_error': max_error,
        'rms_error': rms_error
    }