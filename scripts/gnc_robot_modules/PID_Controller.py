from math import pi
import numpy as np
import rospy

class PID_Controller:
    """
    Enhanced PID controller with anti-windup and improved tuning
    """
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.output_limits = output_limits  # (min, max)
        self.last_time = rospy.Time.now()
    
    def update(self, current_value):
        # Calculate time delta
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        
        # Avoid division by zero
        if dt <= 0:
            dt = 0.01
        
        # Calculate error
        error = self.setpoint - current_value
        
        # Handle circular values like angles
        if hasattr(self, 'is_angle') and self.is_angle:
            if error > pi:
                error -= 2 * pi
            elif error < -pi:
                error += 2 * pi
        
        # Calculate P, I, D terms
        p_term = self.kp * error
        
        # Integrate with anti-windup
        self.integral += error * dt
        if self.output_limits:
            self.integral = np.clip(self.integral, 
                                    self.output_limits[0] / self.ki if self.ki != 0 else -float('inf'),
                                    self.output_limits[1] / self.ki if self.ki != 0 else float('inf'))
        i_term = self.ki * self.integral
        
        # Derivative (on measurement to avoid derivative kick)
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Apply output limits if provided
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
            
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.Time.now()
    
    def set_parameters(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd