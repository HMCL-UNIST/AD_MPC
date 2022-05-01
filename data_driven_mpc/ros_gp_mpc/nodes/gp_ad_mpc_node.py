#!/usr/bin/env python3.6
""" ROS node for the data-augmented MPC, to use in the Gazebo simulator and real world experiments.

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

import json
import time
import rospy
import threading
import numpy as np
import pandas as pd
import random 
import math 
from tqdm import tqdm
from std_msgs.msg import Bool, Empty, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped, Pose
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped 
from hmcl_msgs.msg import Lane, Waypoint, VehicleStatus, VehicleSteering, VehicleSCC
from carla_msgs.msg import CarlaEgoVehicleStatus

from ad_mpc.create_ros_ad_mpc import ROSGPMPC
from ad_mpc.ad_3d import AD3D
from utils.utils import ButterWorth2dFilter, jsonify, interpol_mse, quaternion_state_mse, load_pickled_models, v_dot_q, \
    separate_variables
from utils.visualization import trajectory_tracking_results, mse_tracking_experiment_plot, \
    load_past_experiments, get_experiment_files
from experiments.point_tracking_and_record import make_record_dict, get_record_file_and_dir, check_out_data
from model_fitting.rdrv_fitting import load_rdrv

from utils.utils import quaternion_to_euler, skew_symmetric, v_dot_q, unit_quat, quaternion_inverse, wrap_to_pi, euler_to_quaternion, bound_angle_within_pi
from ad_mpc.ref_traj import RefTrajectory

from visualization_msgs.msg import MarkerArray, Marker
from utils.utils import ButterWorth2dFilter


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


class GPMPCWrapper:
    def __init__(self,environment="real"):
        
        
        self.ad = AD3D(noisy=False, noisy_input= False)
        # Control at 50 (sim) or 60 (real) hz. Use time horizon=1 and 10 nodes
        self.n_mpc_nodes = rospy.get_param('~n_nodes', default=40.0)
        self.t_horizon = rospy.get_param('~t_horizon', default=2.0)
        self.traj_resample_vel = rospy.get_param('~traj_resample_vel', default=True)  
        self.control_freq_factor = rospy.get_param('~control_freq_factor', default=5 if environment == "carla" else 5)
        self.dt = self.t_horizon / self.n_mpc_nodes
        self.opt_dt = self.t_horizon / (self.n_mpc_nodes * self.control_freq_factor)
#################################################################        
        # Initialize GP MPC for point tracking
        self.gp_mpc = ROSGPMPC(self.t_horizon, self.n_mpc_nodes, self.opt_dt)     
        self.mpc_ready = True   
#################################################################
        self.mpc_safe_count_threshold = 10  ## 
        self.mpc_safe_count = 0
        self.solver_status = 0 # 0= feasible, #else = error(infeasible)        
        # Last state obtained from odometry
        self.x = None
        self.velocity = None
        self.steering = None        

        ################
        pose_dt = 0.01
        self.x_filter        = ButterWorth2dFilter(pose_dt,2)
        self.y_filter        = ButterWorth2dFilter(pose_dt,2)
        self.psi_filter      = ButterWorth2dFilter(pose_dt,2)
        self.steering_filter = ButterWorth2dFilter(pose_dt,2)

        
        
        self.pose_sub_count = 0
        self.minus = 1
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
        self.prev_target_steer = None
        # Reference trajectory variables
        self.x_ref = None
        self.t_ref = None
        self.u_ref = None
        self.current_idx = 0
        # self.ad_trajectory = None
        # self.ad_controls = None
        # self.w_control = None
        self.prev_delta_dot = 0.0
        self.prev_accel      = 0.0
        self.pred_N = 0.2/ self.dt
        # set up reference generator
        self.ref_gen = RefTrajectory(traj_horizon=self.n_mpc_nodes, traj_dt=self.t_horizon/self.n_mpc_nodes)

        # To measure optimization elapsed time
        self.optimization_dt = 0

        # Thread for MPC optimization
        self.mpc_thread = threading.Thread()

        # check topic availability
        self.vehicle_status_available = False
        self.pose_available = False 
        self.waypoint_available = False 
        # Binary variable to run MPC only once every other odometry callback
        self.optimize_next = True

        #  Remember the sequence number of the last odometry message received.
        self.last_odom_seq_number = 0

        # Setup node publishers and subscribers. The odometry (sub) and control (pub) topics will vary depending on
        # Assume Carla simulation environment         
        if self.environment == "carla":            
            # pose_topic = "/state_estimator/estimated_pose"            
            pose_topic = "/current_pose"            
            vehicle_status_topic = "/carla/ego_vehicle/vehicle_status"
            control_topic = "/carla/ego_vehicle/ackermann_cmd"            
            waypoint_topic = "/local_traj"
            odom_topic = "/carla/ego_vehicle/odometry"
        else:
            # Real world setup
            pose_topic = "/geo_pose_estimate"
            vehicle_status_topic = "/vehicle_status"
            control_topic = "/hmcl_ctrl_cmd"            
            waypoint_topic = "/local_traj"
            odom_topic = "/pose_estimate"
            scc_topioc = "/usafe_acc_cmd"
        
        status_topic = "/is_mpc_busy"
        # Publishers
        self.scc_pub = rospy.Publisher(scc_topioc, VehicleSCC, queue_size=2, tcp_nodelay=True)
        self.control_pub = rospy.Publisher(control_topic, AckermannDrive, queue_size=2, tcp_nodelay=True)
        self.ref_puf_publisher = rospy.Publisher("/mpc_ref_trajectory", MarkerArray, queue_size=2)
        self.mpc_predicted_trj_publisher = rospy.Publisher("/mpc_pred_trajectory", MarkerArray, queue_size=2)
        self.final_ref_publisher = rospy.Publisher("/final_trajectory", MarkerArray, queue_size=2)
        self.status_pub = rospy.Publisher(status_topic, Bool, queue_size=1)
        if self.environment == "carla":
            self.steering_pub = rospy.Publisher("/mpc_steering", Float64, queue_size=2)
        else:
            self.steering_pub = rospy.Publisher("/usafe_steer_cmd", VehicleSteering, queue_size=2)
        
        ##debugging
        self.debugging_pub = rospy.Publisher("/mpc_debug", Pose, queue_size=2)
        
        
        # Subscribers
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback)
        if self.environment == "carla":   
            self.vehicle_status_sub = rospy.Subscriber(vehicle_status_topic, CarlaEgoVehicleStatus, self.vehicle_status_callback)
        else:
            self.vehicle_status_sub = rospy.Subscriber(vehicle_status_topic, VehicleStatus, self.vehicle_status_callback)
        self.waypoint_sub = rospy.Subscriber(waypoint_topic, Lane, self.waypoint_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        # self.blend_min = 3
        # self.blend_max = 5

        rate = rospy.Rate(1)
     
        while not rospy.is_shutdown():
            # Publish if MPC is busy with a current trajectory
            msg = Bool()
            msg.data = not (self.x_ref is None and self.pose_available)
            self.status_pub.publish(msg)
            rate.sleep()

    def reset_mpc_optimizer(self):
        # del self.gp_mpc
        self.mpc_ready = False
        # self.gp_mpc = ROSGPMPC(self.t_horizon, self.n_mpc_nodes, self.opt_dt)        
        self.mpc_ready = True
    
    def run_mpc(self, odom, x_ref,y_ref,psi_ref,vel_ref, recording=True):
        """
        :param odom: message from subscriber.
        :type odom: Odometry
        :param recording: If False, some messages were skipped between this and last optimization. Don't record any data
        during this optimization if in recording mode.
        """
        if not self.pose_available or not self.vehicle_status_available:
            return
        if not self.waypoint_available:
            return
        # Measure time between initial state was checked in and now
        dt = odom.header.stamp.to_time() - self.last_update_time

        # model_data, x_guess, u_guess = self.set_reference()         --> previous call
        if len(x_ref) < self.n_mpc_nodes:
            ref = [x_ref[-1], y_ref[-1], psi_ref[-1], 0.0, 0.0, 0.0]
            u_ref = [0.0, 0.0]   
            terminal_point = True               
        else:        
            ref = np.zeros([6,len(x_ref)])
            ref[0] = x_ref
            ref[1] = y_ref
            ref[2] = psi_ref
            ref[3] = vel_ref
            ref = ref.transpose()
            u_ref = np.zeros((len(vel_ref)-1,2))
            terminal_point = False
        
        model_data = self.gp_mpc.set_reference(ref,u_ref,terminal_point)        
    
        if self.mpc_ready is False: 
            return
        # Run MPC and publish control
        try:
            tic = time.time()            
            next_control, w_opt, x_opt, self.solver_status = self.gp_mpc.optimize(model_data)
            ####################################
            if x_opt is not None:                
                self.predicted_trj_visualize(x_opt)
            ####################################
            ## Check whether the predicted trajectory is close to the actual reference trajectory // if not apply auxillary control
            self.pred_trj_healthy = self.check_pred_trj(x_opt,ref)
            
            

            if self.solver_status > 0:                                
                self.mpc_safe_count = 0
                self.reset_mpc_optimizer()                
            else: 
                self.mpc_safe_count = self.mpc_safe_count + 1
            ###### check the number of success rounds of the MPC optimization
            if self.mpc_safe_count < self.mpc_safe_count_threshold:
                return

            if not self.pred_trj_healthy:
                return
            
            self.optimization_dt += time.time() - tic
            print("MPC thread. Seq: %d. Topt: %.4f" % (odom.header.seq, (time.time() - tic) * 1000))            
            control_msg = AckermannDrive()
            control_msg = next_control.drive                                                                     
            # if self.environment == "carla":
            #     steering_val = max(min(self.steering_rate_max, next_control.drive.steering_angle_velocity), self.steering_rate_min)                        
            #     control_msg.steering_angle = max(min(self.steering_max, steering_val*0.1 + self.steering), self.steering_min)                        
            # else:
            target_steering_angle = max(min(self.steering_max, next_control.drive.steering_angle), self.steering_min)                      
            
            if self.prev_target_steer is not None:
                str_compensate =  0# (self.prev_target_steer-self.steering)
            else:
                str_compensate = 0
           
            # steering_val = max(min(self.steering_rate_max, next_control.drive.steering_angle_velocity), self.steering_rate_min)                        
            ###########################
            # ###########################
            # # For testing ###########################
            # if self.steering > 0.35: 
            #     self.minus = -1
            # # elif self.steering < -0.35: 
            # #     self.minus = 1
            # steering_val = 1
            # steering_increment = steering_val*0.05
            
            ###########################
            # For angle control ###########################
            # steering_increment = target_steering_angle - self.steering
            ###########################
            # if target_steering_angle > 0.0:
            #     steering_increment = max(steering_increment,0.002)
            # else:
            #     steering_increment = min(steering_increment,-0.002) 
            # steering_increment = self.minus*steering_increment

            control_msg.steering_angle = target_steering_angle #  max(min(self.steering_max, steering_increment+self.steering), self.steering_min)                          
            
            # self.prev_delta_dot = steering_increment/self.dt
            self.prev_accel     = w_opt[0]

            self.prev_target_steer = control_msg.steering_angle
           
            debug_msg = Pose()
            debug_msg.position.x = control_msg.steering_angle
            debug_msg.position.y = self.prev_target_steer
            debug_msg.position.z = next_control.drive.steering_angle_velocity
            
            self.debugging_pub.publish(debug_msg)
            ############################################################
            ############################################################
            ############################################################
            # control_msg.acceleration = -110.0
            ############################################################
            ############################################################
            ############################################################
            if self.environment == "carla":   
                tt_steering = Float64()
                tt_steering.data = -1*control_msg.steering_angle    
                self.steering_pub.publish(tt_steering)            
            else:
                steer_msg = VehicleSteering()
                steer_msg.header.stamp = rospy.Time.now()
                # steer_msg.steering_angle = control_msg.steering_angle   
                steer_msg.steering_angle = self.steering_filter.filter(control_msg.steering_angle)         
                self.steering_pub.publish(steer_msg)       

                acc_msg = VehicleSCC()
                acc_msg.header.stamp = rospy.Time.now()
                acc_msg.acceleration = 0 # w_opt[0]
                self.scc_pub.publish(acc_msg)
                


            self.control_pub.publish(control_msg)            
            

        except KeyError:
            self.recording_warmup = True
            # Should not happen anymore.
            rospy.logwarn("Tried to run an MPC optimization but MPC is not ready yet.")
            return

        if w_opt is not None:            
            self.w_opt = w_opt            

   
    def check_pred_trj(self,x_opt, ref):
        # the average distance between reference and the predicted trj le
        tmp_dist = np.zeros(len(ref))        
        for i in range(0,len(ref)-1):
            tmp_dist[i] = math.sqrt((ref[i,0] - x_opt[i,0])**2+(ref[i,1] - x_opt[i,1])**2)
    
        if np.mean(tmp_dist) < 3.0 and np.cov(tmp_dist) < 2 and np.max(tmp_dist) < 4:
            return True
        else:
            return False
        

    def vehicle_status_callback(self,msg):
  
        
        if self.environment == "carla":   
            if msg.velocity is None:
                return
            self.velocity = msg.velocity
            self.steering = -msg.control.steer
        else:
            if msg.wheelspeed.wheel_speed is None:
                return
            self.velocity = msg.wheelspeed.wheel_speed
            self.steering = msg.steering_info.steering_angle

        if self.vehicle_status_available is False:
            self.vehicle_status_available = True        

    def predicted_trj_visualize(self,predicted_state):
        
        marker_refs = MarkerArray() 
        for i in range(len(predicted_state[:,0])):
            marker_ref = Marker()
            marker_ref.header.frame_id = "map"  
            marker_ref.ns = "mpc_ref"+str(i)
            marker_ref.id = i
            marker_ref.type = Marker.ARROW
            marker_ref.action = Marker.ADD                
            marker_ref.pose.position.x = predicted_state[i,0] 
            marker_ref.pose.position.y = predicted_state[i,1]              
            quat_tmp = euler_to_quaternion(0.0, 0.0, predicted_state[i,2])     
            quat_tmp = unit_quat(quat_tmp)                 
            marker_ref.pose.orientation.w = quat_tmp[0]
            marker_ref.pose.orientation.x = quat_tmp[1]
            marker_ref.pose.orientation.y = quat_tmp[2]
            marker_ref.pose.orientation.z = quat_tmp[3]
            marker_ref.color.r, marker_ref.color.g, marker_ref.color.b = (255, 255, 0)
            marker_ref.color.a = 0.5
            marker_ref.scale.x, marker_ref.scale.y, marker_ref.scale.z = (0.6, 0.4, 0.3)
            marker_refs.markers.append(marker_ref)
            i+=1
        self.mpc_predicted_trj_publisher.publish(marker_refs)
        
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
    
    def final_waypoint_visualize(self):
        marker_refs = MarkerArray() 
        for i, x_ref in enumerate(self.x_ref):
            marker_ref = Marker()
            marker_ref.header.frame_id = "map"  
            marker_ref.ns = "mpc_ref"+str(i)
            marker_ref.id = i
            marker_ref.type = Marker.ARROW
            marker_ref.action = Marker.ADD                
            marker_ref.pose.position.x = self.x_ref[i] 
            marker_ref.pose.position.y = self.y_ref[i]              
            quat_tmp = euler_to_quaternion(0.0, 0.0, self.psi_ref[i])                      
            quat_tmp = unit_quat(quat_tmp)
            marker_ref.pose.orientation.w = quat_tmp[0]
            marker_ref.pose.orientation.x = quat_tmp[1]
            marker_ref.pose.orientation.y = quat_tmp[2]
            marker_ref.pose.orientation.z = quat_tmp[3]
            marker_ref.color.r, marker_ref.color.g, marker_ref.color.b = (0, 0, 255)
            marker_ref.color.a = 0.5
            marker_ref.scale.x, marker_ref.scale.y, marker_ref.scale.z = (0.5, 0.4, 0.3)
            marker_refs.markers.append(marker_ref)
            i+=1
        self.final_ref_publisher.publish(marker_refs)
    

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

        msg.waypoints = msg.waypoints[1:-1]
        if not self.waypoint_available:
            self.waypoint_available = True
        
        self.x_ref = [msg.waypoints[i].pose.pose.position.x for i in range(len(msg.waypoints))]
        self.y_ref = [msg.waypoints[i].pose.pose.position.y for i in range(len(msg.waypoints))]                        
        quat_to_euler_lambda = lambda o: quaternion_to_euler([o[0], o[1], o[2], o[3]])            
        self.psi_ref = [wrap_to_pi(quat_to_euler_lambda([msg.waypoints[i].pose.pose.orientation.w,msg.waypoints[i].pose.pose.orientation.x,msg.waypoints[i].pose.pose.orientation.y,msg.waypoints[i].pose.pose.orientation.z])[2]) for i in range(len(msg.waypoints))]                                    
        
        self.vel_ref = [msg.waypoints[i].twist.twist.linear.x*2.0 for i in range(len(msg.waypoints))]
        #### vel_remap via current vel 
        self.resample_vel()
       

        while len(self.x_ref) < self.n_mpc_nodes:
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
            # print("yaw = " + str(yaw*180/np.pi))
            global_x = msg.twist.twist.linear.x  # vector in global x 
            global_y = msg.twist.twist.linear.y # vector in global x 
            # l_x_ = np.array([np.cos(yaw), np.sin(yaw)])   # vector v:
            # l_y_ = np.array([np.cos(yaw+np.pi/2.0), np.sin(yaw+np.pi/2.0)])   # vector v:
            # # print("l_x = "+str(l_x_[0])+" " +str(l_x_[1]) + ",  l_y = " +str(l_y_[0])+" " +str(l_y_[1]))
            
            # proj_of_global_x_on_l_x = l_x_ * np.dot(global_x, l_x_) / np.dot(l_x_, l_x_)
            # proj_of_global_y_on_l_x = l_x_ * np.dot(global_y, l_x_) / np.dot(l_x_, l_x_)
           
            # proj_of_global_x_on_l_y = l_y_ * np.dot(global_x, l_y_) / np.dot(l_y_, l_y_)
            # proj_of_global_y_on_l_y =  l_y_ * np.dot(global_y, l_y_) / np.dot(l_y_, l_y_)
            self.v_x = global_x*np.cos(-1*yaw) - global_y*np.sin(-1*yaw)
            self.v_y = global_x*np.sin(-1*yaw) + global_y*np.cos(-1*yaw)
            # proj_of_global_x_on_l_x = (np.dot(global_x, l_x_))*l_x_        
            # proj_of_global_y_on_l_x = (np.dot(global_y, l_x_))*l_x_        

            # proj_of_global_x_on_l_y = (np.dot(global_x, l_y_))*l_y_        
            # proj_of_global_y_on_l_y = (np.dot(global_y, l_y_))*l_y_        

            # self.v_x = (proj_of_global_x_on_l_x + proj_of_global_y_on_l_x)[0]
            # self.v_y = (proj_of_global_x_on_l_y + proj_of_global_y_on_l_y)[0]
            

            
        self.psi_dot = msg.twist.twist.angular.z
                

    def pose_callback(self, msg):
        """                
        :type msg: PoseStamped
        """      
        if self.velocity is None or not self.odom_available or not self.waypoint_available:
            return        
        
        

        # self.cur_x = msg.pose.position.x
        # self.cur_y = msg.pose.position.y
        
        cur_euler = quaternion_to_euler([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])            
        # self.cur_yaw = wrap_to_pi(cur_euler[2])        
        
        self.cur_x = self.x_filter.filter(msg.pose.position.x)        
        self.cur_y = self.y_filter.filter(msg.pose.position.y)      
        self.cur_z = msg.pose.position.z            
        self.cur_yaw = wrap_to_pi(self.psi_filter.filter(cur_euler[2]))         
        
        

        p_x = [self.cur_x]
        p_y = [self.cur_y]
        psi = [self.cur_yaw] 
        v_x = [self.v_x]
        v_y = [self.v_y]
        psi_dot = [self.psi_dot]        
        

        self.x = p_x+p_y+psi+v_x+v_y+psi_dot
        # dt = 0.05 
        # self.predicted_states(self.x)
        
        self.pose_sub_count = self.pose_sub_count+1
        if self.pose_sub_count < 10:
            return
        else:
            self.pose_sub_count = 0
        
        
        try:
            if self.mpc_ready:
                # Update the state estimate of the AD
                self.gp_mpc.set_state(self.x)
                # reset the reference traj 
                if not self.waypoint_available:
                    return
                
                waypoint_dict = self.ref_gen.get_waypoints(p_x[0], p_y[0], psi[0])
                
                x_ref    = waypoint_dict['x_ref']            
                y_ref    = waypoint_dict['y_ref']
                psi_ref = waypoint_dict['psi_ref']        
                vel_ref = waypoint_dict['v_ref']                  
                    
                self.waypoint_visualize(x_ref,y_ref,psi_ref)
            
        except AttributeError:
            rospy.loginfo("mpc_node......set_state fail")
            return

        if self.pose_available is False:
            self.pose_available = True        

        # run mpc 
        def _thread_func():
            self.run_mpc(msg,x_ref,y_ref,psi_ref,vel_ref)            
        
        self.mpc_thread = threading.Thread(target=_thread_func(), args=(), daemon=True)
        self.mpc_thread.start()
        self.mpc_thread.join()
        self.last_odom_seq_number = msg.header.seq
        self.optimize_next = False
        
        def _aux_thread_func():
                self.run_pure(msg,x_ref,y_ref,psi_ref,vel_ref)
        
        if self.mpc_safe_count < self.mpc_safe_count_threshold or not self.pred_trj_healthy:         
            # implement auxilary controller (e.g. PID)             
            self.pure_thread = threading.Thread(target=_aux_thread_func(), args=(), daemon=True)
            self.pure_thread.start()
            self.pure_thread.join()
    
    def run_pure(self, odom, x_ref,y_ref,psi_ref,vel_ref, recording=True):
        if not self.pose_available or not self.vehicle_status_available:
            return
        if not self.waypoint_available:
            return
        # Measure time between initial state was checked in and now
        dt = odom.header.stamp.to_time() - self.last_update_time        
        ref = np.zeros([7,len(x_ref)])
        ref[0] = x_ref
        ref[1] = y_ref
        ref[2] = psi_ref
        ref[3] = vel_ref
        ref = ref.transpose()
        
        # Run MPC and publish control
        try:
            tic = time.time()                        
            control_msg = AckermannDrive()                        
            control_msg.steering_angle =  self.steering 
            control_msg.acceleration = -3          
            self.control_pub.publish(control_msg)            
            

        except KeyError:            
            # Should not happen anymore.
            rospy.logwarn("Tried to run an auximlary controller is not ready yet.")
            return
    
    #################### 
    #  propogate state
    # 
    def predicted_states(self):                
        
        p_x     = self.cur_x
        p_y     = self.cur_y
        psi     = self.cur_yaw           
        v_x     = self.v_x            
        v_y     = self.v_y
        psi_dot = self.psi_dot
        delta   = self.steering    
        
        for i in range(self.pred_N):
            px_dot      =  v_x*np.cos(psi)-v_y*np.sin(psi)
            py_dot      =  v_x*np.sin(psi)+v_y*np.cos(psi)
            psi_dot     = psi_dot_dot
            vx_dot      = self.prev_accel
            vy_dot      =  (self.prev_delta_dot*v_x+delta*self.prev_accel)*self.ad.L_R/(self.ad.L_R+self.ad.L_F)        
            psi_dot_dot = (self.prev_delta_dot*v_x+delta*self.prev_accel)/(self.ad.L_R+self.ad.L_F)        
            delta_dot   = self.prev_delta_dot

            p_x      =  p_x    + self.dt*px_dot    
            p_y      =  p_y    + self.dt*py_dot    
            psi      =  psi    + self.dt*psi_dot   
            v_x      =  v_x    + self.dt*vx_dot    
            v_y      =  v_y    + self.dt*vy_dot    
            psi_dot  =  psi_dot+ self.dt*psi_dot_dot
            delta    =  delta  + self.dt*delta_dot 

        self.cur_x   = [p_x]
        self.cur_y   = [p_y]
        self.cur_yaw = [psi] 
        self.v_x  =  [v_x]  
        self.v_y  = [v_y]
        self.psi_dot = [psi_dot]   
        self.steering
        p_x = [p_x]
        p_y = [p_y]
        psi = [psi] 
        v_x = [v_x]
        v_y = [v_y]
        psi_dot = [psi_dot]        
        steering = [delta]
        self.x  = p_x+p_y+psi+v_x+v_y+psi_dot+steering

        
    
###################################################################################

def main():
    rospy.init_node("gp_mpc")
    env = rospy.get_param('~environment', default='real')
    GPMPCWrapper(env)

if __name__ == "__main__":
    main()
