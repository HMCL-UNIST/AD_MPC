#!/usr/bin/env python3.6
""" ROS node for the Stanley ctrl

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

import time
import rospy
import threading
import numpy as np
import pandas as pd
import math 
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped 
from hmcl_msgs.msg import Lane, VehicleSteering, VehicleStatus
from carla_msgs.msg import CarlaEgoVehicleStatus
from ad_3d import AD3D
from utils.utils import quaternion_to_euler,  unit_quat,  wrap_to_pi, euler_to_quaternion
from ref_traj import RefTrajectory
import std_msgs.msg
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray, Marker


class Stanley_ctrl:
    def __init__(self,environment="carla"):
        self.ad = AD3D(noisy=False, noisy_input= False)
        # Control at 50 (sim) or 60 (real) hz. 
        
        self.n_nodes = rospy.get_param('~n_nodes', default=20.0)
        self.t_horizon = rospy.get_param('~t_horizon', default=2.0)
        self.traj_resample_vel = rospy.get_param('~traj_resample_vel', default=True)                
        self.dt = self.t_horizon / self.n_nodes


#################################################################        
        self.last_target_idx = 0
        self.current_target_idx = 0 
        self.stanley_gain_k = 0.5
#################################################################
        # Last state obtained from odometry
        self.x = None
        self.velocity = None
        self.steering = None

        self.steering_min = self.ad.steering_min
        self.steering_max = self.ad.steering_max
        self.steering_rate_min = self.ad.steering_rate_min
        self.steering_rate_max = self.ad.steering_rate_max

        self.environment = environment
        # Elapsed time between two recordings
        self.last_update_time = time.time()

        # Last references. Use hovering activation as input reference
        self.last_x_ref = None
        self.last_u_ref = None

        self.odom_available = False  
        
        # Reference trajectory variables
        self.x_ref = None
        self.t_ref = None
        self.u_ref = None
        self.current_idx = 0
    
        # set up reference generator
        


        # check topic availability
        self.vehicle_status_available = False
        self.pose_available = False 
        self.waypoint_available = False 
      
        # Setup node publishers and subscribers. The odometry (sub) and control (pub) topics will vary depending on
        # Assume Carla simulation environment         
        if self.environment == "carla":            
            # pose_topic = "/state_estimator/estimated_pose"
            pose_topic = "/current_pose"            
            vehicle_status_topic = "/carla/ego_vehicle/vehicle_status"
            control_topic = "/carla/ego_vehicle/ackermann_cmd"            
            waypoint_topic = "/local_traj"
            odom_topic = "/carla/ego_vehicle/odometry"
            status_topic = "/is_mpc_busy"
        else:
            # Real world setup
            pose_topic = "/geo_pose_estimate"
            vehicle_status_topic = "/vehicle_status"
            control_topic = "/hmcl_ctrl_cmd"            
            waypoint_topic = "/local_traj"
            odom_topic = "/pose_estimate"
        
        status_topic = "/is_mpc_busy"
        # Publishers
        self.control_pub = rospy.Publisher(control_topic, AckermannDrive, queue_size=1, tcp_nodelay=True)
        self.ref_puf_publisher = rospy.Publisher("/mpc_ref_trajectory", MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher(status_topic, Bool, queue_size=1)
        self.vel_setpoint_pub = rospy.Publisher("/setpoint",Float64, queue_size = 5)
        if self.environment == "carla":
            self.steering_pub = rospy.Publisher("/mpc_steering", Float64, queue_size=1)
        else:
            self.steering_pub = rospy.Publisher("/usafe_steer_cmd", VehicleSteering, queue_size=1)
        # Subscribers
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback)
        if self.environment == "carla":   
            self.vehicle_status_sub = rospy.Subscriber(vehicle_status_topic, CarlaEgoVehicleStatus, self.vehicle_status_callback)
        else:
            self.vehicle_status_sub = rospy.Subscriber(vehicle_status_topic, VehicleStatus, self.vehicle_status_callback)
        self.waypoint_sub = rospy.Subscriber(waypoint_topic, Lane, self.waypoint_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.ref_gen = RefTrajectory(traj_horizon=self.n_nodes, traj_dt=self.dt)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            msg = Bool()
            msg.data = not (self.x_ref is None and self.pose_available)
            self.status_pub.publish(msg)
            rate.sleep()


    def vehicle_status_callback(self,msg):
        if msg.velocity is None:
            return
        self.velocity = msg.velocity
        self.steering = -msg.control.steer

        if self.vehicle_status_available is False:
            self.vehicle_status_available = True        

    
    def waypoint_visualize(self,x_ref,y_ref,psi_ref):
        
        marker_refs = MarkerArray() 
        for i, xx in enumerate(x_ref):
            marker_ref = Marker()
            marker_ref.header.frame_id = "map"  
            marker_ref.ns = "mpc_ref"+str(i)
            marker_ref.id = i
            marker_ref.type = Marker.ARROW
            marker_ref.action = Marker.ADD                
            marker_ref.pose.position.x = x_ref[i] 
            marker_ref.pose.position.y = y_ref[i]              
            quat_tmp = euler_to_quaternion(0.0, 0.0, psi_ref[i])     
            quat_tmp = unit_quat(quat_tmp)                 
            marker_ref.pose.orientation.w = quat_tmp[0]
            marker_ref.pose.orientation.x = quat_tmp[1]
            marker_ref.pose.orientation.y = quat_tmp[2]
            marker_ref.pose.orientation.z = quat_tmp[3]
            marker_ref.color.r, marker_ref.color.g, marker_ref.color.b = (255, 0, 0)
            marker_ref.color.a = 0.5
            marker_ref.scale.x, marker_ref.scale.y, marker_ref.scale.z = (0.8, 0.3, 0.2)
            marker_refs.markers.append(marker_ref)
            i+=1
        self.ref_puf_publisher.publish(marker_refs)
    
  
    

    def resample_vel(self):    
        MAX_bound = math.sqrt(self.v_x**2 + self.v_y**2)  
        for i in range(len(self.vel_ref)):
            if(self.vel_ref[i] > MAX_bound):
                self.vel_ref[i] = MAX_bound
            MAX_bound = MAX_bound + self.ad.acc_max*self.dt*0.8 
    
    
    def waypoint_callback(self, msg):
        """
        :type msg: autoware_msgs/Lane 
        """                        
        if not self.odom_available:
            return

        msg.waypoints = msg.waypoints[0:-1]
        if not self.waypoint_available:
            self.waypoint_available = True
        
        self.x_ref = [msg.waypoints[i].pose.pose.position.x for i in range(len(msg.waypoints))]
        self.y_ref = [msg.waypoints[i].pose.pose.position.y for i in range(len(msg.waypoints))]                        
        quat_to_euler_lambda = lambda o: quaternion_to_euler([o[0], o[1], o[2], o[3]])            
        self.psi_ref = [wrap_to_pi(quat_to_euler_lambda([msg.waypoints[i].pose.pose.orientation.w,msg.waypoints[i].pose.pose.orientation.x,msg.waypoints[i].pose.pose.orientation.y,msg.waypoints[i].pose.pose.orientation.z])[2]) for i in range(len(msg.waypoints))]                                    
        
        self.vel_ref = [msg.waypoints[i].twist.twist.linear.x*3 for i in range(len(msg.waypoints))]
        #### vel_remap via current vel 
        self.resample_vel()
        

        while len(self.x_ref) < self.n_nodes:
            self.x_ref.insert(-1,self.x_ref[-1])
            self.y_ref.insert(-1,self.y_ref[-1])
            self.psi_ref.insert(-1,self.psi_ref[-1])
            self.vel_ref.insert(-1,self.vel_ref[-1])
      
        self.ref_gen.set_traj(self.x_ref, self.y_ref, self.psi_ref, self.vel_ref)
        
        
    def odom_callback(self, msg):
        if self.odom_available is False:
            self.odom_available = True  
        if self.environment == "carla":   
            self.v_x = msg.twist.twist.linear.x 
            self.v_y = msg.twist.twist.linear.y
        else:
            # global to local frame 
            quat_ = [msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]
            euler_ = quaternion_to_euler(quat_)
            yaw = euler_[2] 
            global_x = msg.twist.twist.linear.x  # vector in global x 
            global_y = msg.twist.twist.linear.y # vector in global x 
            self.v_x = global_x*np.cos(-1*yaw) - global_y*np.sin(-1*yaw)
            self.v_y = global_x*np.sin(-1*yaw) + global_y*np.cos(-1*yaw)
        self.psi_dot = msg.twist.twist.angular.z
        
                
    def calc_target_index(self,current_state, waypoint_dict):
        fx = current_state[0] + self.ad.L/2.0 * np.cos(current_state[2])
        fy = current_state[1] + self.ad.L/2.0 * np.sin(current_state[2])
        cx    = waypoint_dict['x_ref']            
        cy    = waypoint_dict['y_ref']
        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(current_state[2] + np.pi / 2),
                    -np.sin(current_state[2] + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
        return target_idx, error_front_axle 
        
    def pose_callback(self, msg):
        """                
        :type msg: PoseStamped
        """      
        if (self.velocity is None) or (self.odom_available is False) or (self.waypoint_available is False):
            return        

        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y
        self.cur_z = msg.pose.position.z                
        cur_euler = quaternion_to_euler([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])            
        self.cur_yaw = wrap_to_pi(cur_euler[2])        
        
        p_x = [self.cur_x]
        p_y = [self.cur_y]
        psi = [self.cur_yaw] 
        v_x = [self.v_x]
        v_y = [self.v_y]
        psi_dot = [self.psi_dot]        
        steering = [self.steering]

        self.x = p_x+p_y+psi+v_x+v_y+psi_dot+steering
        
        try:
            
            # reset the reference traj 
            if self.waypoint_available is False:
                return
            
            
            waypoint_dict = self.ref_gen.get_waypoints(p_x[0], p_y[0], psi[0])
            
            waypoint_dict['x_ref']   = waypoint_dict['x_ref'][2:]
            waypoint_dict['y_ref']   = waypoint_dict['y_ref'][2:]
            waypoint_dict['psi_ref']   = waypoint_dict['psi_ref'][2:]
            waypoint_dict['v_ref']   = waypoint_dict['v_ref'][2:]

            x_ref    = waypoint_dict['x_ref']            
            y_ref    = waypoint_dict['y_ref']
            psi_ref = waypoint_dict['psi_ref']        
            vel_ref = waypoint_dict['v_ref']                  
                
            self.waypoint_visualize(x_ref,y_ref,psi_ref)
            
        except AttributeError:
            rospy.loginfo("......set_state fail")
            return

        if self.pose_available is False:
            self.pose_available = True        
        current_state = [self.cur_x, self.cur_y, self.cur_yaw]
        ###################### stanley controller 
        target_idx, error_front_axle = self.calc_target_index(current_state, waypoint_dict)
        if self.last_target_idx >= self.current_target_idx:
            self.current_target_idx = self.last_target_idx

        # theta_e corrects the heading error
        theta_e = wrap_to_pi(psi_ref[self.current_target_idx] - self.cur_yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.stanley_gain_k * error_front_axle, np.sqrt(self.v_x**2 + self.v_y**2))
        # Steering control
        delta = theta_e + theta_d

        next_control = AckermannDriveStamped()                
        next_control.header = std_msgs.msg.Header()
        next_control.header.stamp = rospy.Time.now()        
        next_control.drive.steering_angle = delta
        next_control.drive.steering_angle_velocity = 0        
        next_control.drive.speed = 5                
        next_control.drive.acceleration =  2# w_opt[0]   
        control_msg = AckermannDrive()
        control_msg = next_control.drive                                                                             
        control_msg.steering_angle = max(min(self.steering_max, delta), self.steering_min)                        
        tt_steering = Float64()
        tt_steering.data = -1*control_msg.steering_angle            
        if self.environment == "carla":
            self.steering_pub.publish(tt_steering)            
            self.control_pub.publish(control_msg)   
        else:
            steering_msg = VehicleSteering()
            steering_msg.steering_angle = control_msg.steering_angle
            self.steering_pub.publish(steering_msg)
            vel_msg = Float64()
            vel_msg.data = vel_ref[0]
            self.vel_setpoint_pub.publish(vel_msg)

     
###################################################################################

def main():
    rospy.init_node("stanley_ctrl")
    env = rospy.get_param('~environment', default='gazebo')
    Stanley_ctrl(env)

if __name__ == "__main__":
    main()
