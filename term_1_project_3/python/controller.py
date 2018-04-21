"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0


class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""
 
        # Body-rate controller gains
        self.body_rate_p_k_p = 0.1 
        self.body_rate_q_k_p = 0.1 
        self.body_rate_r_k_p = 0.1 

        # Altitude controller gains
        self.altitude_k_p = 25. 
        self.altitude_k_d = 10. 

        # Yaw controller gains
        self.yaw_k_p = 1.5 #0.02

        # Roll-pitch controller gains
        self.roll_pitch_k_p_roll = 10.5 
        self.roll_pitch_k_p_pitch = 10.5

        # Lateral controller gains
        self.lateral_x_k_p = 0.25 
        self.lateral_x_k_d = 0.02 
        self.lateral_y_k_p = 0.25 
        self.lateral_y_k_d = 0.02 


    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]
            
        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        
        
        return (position_cmd, velocity_cmd, yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        x_target, y_target = local_position_cmd
        x_dot_target, y_dot_target = local_velocity_cmd

        x, y = local_position
        x_dot, y_dot = local_velocity

        x_dot_dot_ff, y_dot_dot_ff = acceleration_ff

        x_dot_err = x_dot_target - x_dot
        x_err = x_target - x

        x_dot_dot =  self.lateral_x_k_p * x_err + self.lateral_x_k_d * x_dot_err + x_dot_dot_ff

        y_dot_err =  y_dot_target - y_dot
        y_err = y_target - y
      
        y_dot_dot = self.lateral_x_k_p * y_err + self.lateral_y_k_d * y_dot_err + y_dot_dot_ff

        return np.array([-x_dot_dot, -y_dot_dot])
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: thrust command for the vehicle (+up)
        """

        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity
        roll, pitch, yaw = attitude
        b_z = euler2RM(roll, pitch, yaw )[2,2]

        u_1 = self.altitude_k_p * z_err + self.altitude_k_d * z_err_dot + acceleration_ff
        thrust = (u_1 - GRAVITY)/b_z

        if thrust>=MAX_THRUST:
            thrust = MAX_THRUST

        return thrust 
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        
        b_x_cmd, b_y_cmd = acceleration_cmd
        roll, pitch, yaw = attitude

        rot_mat = euler2RM(roll, pitch, yaw)

        b_x = rot_mat[0,2]
        #b_x_err = b_x_cmd/thrust_cmd - b_x
        b_x_err = b_x_cmd - b_x
        b_x_p_term = self.roll_pitch_k_p_roll * b_x_err
        
        b_y = rot_mat[1,2]
        b_y_err = b_y_cmd - b_y  
        #b_y_err = b_y_cmd/thrust_cmd - b_y  
        b_y_p_term = self.roll_pitch_k_p_pitch * b_y_err 
        
        b_x_commanded_dot = b_x_p_term
        b_y_commanded_dot = b_y_p_term
        
        rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]
        
        rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
        p_c = rot_rate[0]
        q_c = rot_rate[1]
                
        return np.array([p_c, q_c])
    
    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        
        p_cmd, q_cmd, r_cmd = body_rate_cmd
        p_actual, q_actual, r_actual = body_rate

        p_err = p_cmd - p_actual 
        u_bar_p = self.body_rate_p_k_p * p_err
        
        q_err = q_cmd - q_actual 
        u_bar_q = self.body_rate_q_k_p * q_err

        r_err = r_cmd - r_actual
        u_bar_r = self.body_rate_r_k_p * r_err
        
        return np.array([u_bar_p, u_bar_q, u_bar_r])
    
    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        yaw_error = yaw_cmd - yaw

        r_c = self.yaw_k_p * yaw_error        
        return r_c
