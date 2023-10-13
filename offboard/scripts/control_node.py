#! /usr/bin/env python3
import numpy as np
import rospy
import math
import tf
from std_msgs.msg import Float32, String, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.msg import State, PositionTarget
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dynamic_reconfigure.client import Client

from ysdrone_msgs.srv import *
from koreauav_utils import auto_service, math_utils


class ControlClass(object):
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.cmd_state = 0
        self.mission_num = Float32()
        self.mission_rep = String() #mission_rep = mission name
        self.resp = DroneCommandResponse()
        self.resp.mode = 'Takeoff Mode'

        #========================================
        '''Mission 1(Avoidance Mode)'''
        self.avoidance = PoseStamped()
        self.dynamic_client = Client('/local_planner_node', timeout=30)
        self.destination_command_marker = Marker()
        self.destination_command_marker_array = MarkerArray()

        # Subscriber
        self.destination_command_sub = rospy.Subscriber('/destination_command', PoseStamped, self.destination_command_cb)
        self.avoidance_pos_sub = rospy.Subscriber('/avoidance/setpoint_position/local', PoseStamped, self.avoidance_pos_cb)
        #========================================

        #========================================
        '''Mission 2(Building Search Mode)'''
        self.building_target = PoseStamped()
        self.building_target_marker = Marker()
        self.building_target_marker_array = MarkerArray()

        # Subscriber
        self.building_target_sub = rospy.Subscriber('/building/search/target_pose', PoseStamped, self.building_target_cb)
        #========================================

        #========================================
        '''Mission 3(Marker Approach Mode)'''
        self.launch_setposition = PoseStamped()
        self.launch_setposition_marker = Marker()
        self.launch_setposition_marker_array = MarkerArray()

        # SUbscriber
        self.launch_setposition_sub = rospy.Subscriber('/launch_setposition', PoseStamped, self.launch_setposition_cb)
        #========================================

        #========================================
        '''Mission 4(Cargo Launch Mode)'''
        self.move = PoseStamped()
        self.move_marker = Marker()
        self.move_marker_array = MarkerArray()

        # Subscriber
        self.move_sub = rospy.Subscriber('/move_to_launch', PoseStamped, self.move_cb)
        #========================================

        #========================================
        '''Mission 5(Of course I Still Love You)'''
        self.isly_destination_command_marker = Marker()
        self.isly_destination_command_marker_array = MarkerArray()

        # Subscriber
        self.isly_destination_command_sub = rospy.Subscriber('/isly_destination_command', PoseStamped, self.isly_destination_command_cb)
        #========================================

        #========================================
        '''Mission 6(Precision Landing with PID)'''
        self.relative_dis = Float32MultiArray()
        self.desired_landing = PositionTarget()
        self.desired_landing_position = PoseStamped()

        # Subscriber
        self.desired_landing_sub = rospy.Subscriber('/desired_landing', PositionTarget, self.desired_landing_cb)
        self.desired_landing_position_sub = rospy.Subscriber('/desired_landing_position', PoseStamped, self.desired_landing_position_cb)
        self.relative_dis_sub = rospy.Subscriber("/relative_distance", Float32MultiArray, self.relative_dis_cb)
        #========================================

        #========================================
        '''Mission 10(Precision Landing with Reinforcement Learning)'''
        self.landing_velocity_position = PoseStamped()
        self.landing_velocity = PositionTarget()

        # Subscriber
        self.landing_velocity_sub = rospy.Subscriber('/landing_velocity', PositionTarget, self.landing_velocity_cb)
        self.landing_velocity_position_sub = rospy.Subscriber('/landing_velocity_position', PoseStamped, self.landing_velocity_position_cb)
        #========================================
    
        #Publisher
        self.target_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.avoidance_pos_pub = rospy.Publisher('input/goal_position', MarkerArray, queue_size=1)
        self.desired_landing_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.mission_pub = rospy.Publisher('/mission', Float32, queue_size=1)
        self.mission_rep_pub = rospy.Publisher('/mission_rep', String, queue_size=1)
        
        #Subscriber
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)

    
    def avoidance_pos_cb(self, msg): # Callback for using avoidance
        self.avoidance = msg

        if self.cmd_state == 2 and self.building_target.pose.position.z != 0:
            self.avoidance = self.building_target
        elif self.cmd_state == 3 and self.launch_setposition.pose.position.z != 0:
            self.avoidance = self.launch_setposition
        elif self.cmd_state == 4 and self.move.pose.position.z != 0:
            self.avoidance = self.move

        if self.cmd_state not in [6, 10, 11]:
            self.target_pose_pub.publish(self.avoidance)
    
    def move_cb(self, msg):
        self.move = msg
        self.move_marker.pose.position.x = msg.pose.position.x
        self.move_marker.pose.position.y = msg.pose.position.y
        self.move_marker.pose.position.z = msg.pose.position.z
        self.move_marker_array.markers.clear()
        self.move_marker_array.markers.append(self.move_marker)

    def building_target_cb(self, msg):
        self.building_target = msg
        self.building_target_marker.pose.position.x = msg.pose.position.x
        self.building_target_marker.pose.position.y = msg.pose.position.y
        self.building_target_marker.pose.position.z = msg.pose.position.z
        self.building_target_marker_array.markers.clear()
        self.building_target_marker_array.markers.append(self.building_target_marker)
        self.building_target.pose.position.x = msg.pose.position.x
        self.building_target.pose.position.y = msg.pose.position.y
        self.building_target.pose.position.z = msg.pose.position.z
        self.building_target.pose.orientation.x = msg.pose.orientation.x
        self.building_target.pose.orientation.y = msg.pose.orientation.y
        self.building_target.pose.orientation.z = msg.pose.orientation.z
        self.building_target.pose.orientation.w = msg.pose.orientation.w

    def launch_setposition_cb(self, msg):
        self.launch_setposition = msg
        self.launch_setposition_marker.pose.position.x = msg.pose.position.x
        self.launch_setposition_marker.pose.position.y = msg.pose.position.y
        self.launch_setposition_marker.pose.position.z = msg.pose.position.z
        self.launch_setposition_marker_array.markers.clear()
        self.launch_setposition_marker_array.markers.append(self.launch_setposition_marker)
        self.launch_setposition.pose.position.x = msg.pose.position.x
        self.launch_setposition.pose.position.y = msg.pose.position.y
        self.launch_setposition.pose.position.z = msg.pose.position.z
        self.launch_setposition.pose.orientation.x = msg.pose.orientation.x
        self.launch_setposition.pose.orientation.y = msg.pose.orientation.y
        self.launch_setposition.pose.orientation.z = msg.pose.orientation.z
        self.launch_setposition.pose.orientation.w = msg.pose.orientation.w

    def desired_landing_cb(self, msg):
        self.desired_landing.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.desired_landing.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
        self.desired_landing.velocity.x = msg.velocity.x
        self.desired_landing.velocity.y = msg.velocity.y
        self.desired_landing.velocity.z = msg.velocity.z
        self.desired_landing.yaw = msg.yaw
        self.desired_landing.yaw_rate = 1

    def desired_landing_position_cb(self, msg):
        self.desired_landing_position = msg
        
    def landing_velocity_cb(self, msg):
        self.landing_velocity.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.landing_velocity.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ 
        self.landing_velocity.velocity.x = msg.velocity.x
        self.landing_velocity.velocity.y = msg.velocity.y
        self.landing_velocity.velocity.z = msg.velocity.z
        self.landing_velocity.yaw = msg.yaw
        self.landing_velocity.yaw_rate = 1
        
    def landing_velocity_position_cb(self, msg):
        self.landing_velocity_position = msg
        
    def relative_dis_cb(self, msg): #Callback for relative distance between drone and target
        self.relative_dis = msg

    def destination_command_cb(self, msg):
        self.destination_command_marker.pose.position.x = msg.pose.position.x
        self.destination_command_marker.pose.position.y = msg.pose.position.y
        self.destination_command_marker.pose.position.z = msg.pose.position.z
        self.destination_command_marker_array.markers.clear()
        self.destination_command_marker_array.markers.append(self.destination_command_marker)
    
    def state_cb(self, msg):
        self.current_state = msg
    
    def pose_cb(self, msg):
        self.current_pose = msg
        
    def isly_destination_command_cb(self, msg):
        self.isly_destination_command_marker.pose.position.x = msg.pose.position.x
        self.isly_destination_command_marker.pose.position.y = msg.pose.position.y
        self.isly_destination_command_marker.pose.position.z = msg.pose.position.z
        self.isly_destination_command_marker_array.markers.clear()
        self.isly_destination_command_marker_array.markers.append(self.isly_destination_command_marker)
    
    
    def cmdreact_cb(self, req):
        self.cmd_state = req.command
        self.resp = DroneCommandResponse()
        if self.cmd_state == 0:
            self.resp.mode = "Takeoff Mode"
            self.resp.res = True
        elif self.cmd_state == 1:
            self.resp.mode = 'Avoidance Mode'
            self.resp.res = True
        elif self.cmd_state == 2:
            self.resp.mode = 'Building Search Mode'
            self.resp.res = True
        elif self.cmd_state == 3:
            self.resp.mode = 'Marker Approach Mode'
            self.resp.res = True
        elif self.cmd_state == 4:
            self.resp.mode = 'Cargo Launch Mode'
            self.resp.res = True
        elif self.cmd_state == 5:
            self.resp.mode = 'Of course I Still Love You'
            self.resp.res = True
        elif self.cmd_state == 6:
            self.resp.mode = 'Precision Landing Mode'
            self.resp.res = True
        elif self.cmd_state == 10:
            self.resp.mode = 'RL Landing with Aruco'
            self.resp.res = True
        elif self.cmd_state == 11:
            self.resp.mode = 'Landing and Disarming'
            self.resp.res = True
        rospy.loginfo(f'Received request : {req.command} && Current Mode : {self.resp.mode} && Enable :{self.resp.res}')
        return self.resp
    
    def main_controller(self, e):
        self.time_now = rospy.Time.now()
        self.mission_num.data = self.cmd_state
        self.mission_pub.publish(self.mission_num)
        self.mission_rep.data = self.resp.mode
        self.mission_rep_pub.publish(self.mission_rep)


        if self.cmd_state == 0:
            self.target_pose.pose.position.x = 0
            self.target_pose.pose.position.y = 0
            self.target_pose.pose.position.z = 15.0
            self.target_pose.pose.orientation.x = self.current_pose.pose.orientation.x
            self.target_pose.pose.orientation.y = self.current_pose.pose.orientation.y
            self.target_pose.pose.orientation.z = self.current_pose.pose.orientation.z
            self.target_pose.pose.orientation.w = self.current_pose.pose.orientation.w
            self.target_pose_pub.publish(self.target_pose)

            if abs(self.target_pose.pose.position.z - self.current_pose.pose.position.z) < 0.1:
                auto_service.call_drone_command(1)
        # Mission 1(Obstacle Avoidance Planner)
        if self.cmd_state == 1:
            new_config = {"obstacle_cost_param_": 4}
            config = self.dynamic_client.update_configuration(new_config)
            self.avoidance_pos_pub.publish(self.destination_command_marker_array)

        # Mission 2(Building Searching)
        if self.cmd_state == 2:
            self.avoidance_pos_pub.publish(self.building_target_marker_array)

        # Mission 3(Cross Detection Mode)
        if self.cmd_state == 3:
            new_config = {"obstacle_cost_param_": 1}
            config = self.dynamic_client.update_configuration(new_config)
            self.avoidance_pos_pub.publish(self.launch_setposition_marker_array) 
    
        # Mission 4(Cargo Launching Mode)
        if self.cmd_state == 4:
            self.avoidance_pos_pub.publish(self.move_marker_array)

        # Mission 5(Of course I Still Love you)
        if self.cmd_state == 5:
            new_config = {"obstacle_cost_param_": 5}
            config = self.dynamic_client.update_configuration(new_config)
            self.avoidance_pos_pub.publish(self.isly_destination_command_marker_array)

        # Mission 6(Safety Landing)
        if self.cmd_state == 6:
            try:
                if np.isnan(self.relative_dis.data[0]):
                    self.target_pose_pub.publish(self.desired_landing_position)
                else:
                    self.desired_landing_pub.publish(self.desired_landing)
            except IndexError:
                self.target_pose_pub.publish(self.current_pose)
 
        # Mission 10(RL Landing with Aruco)
        if self.cmd_state == 10:
            try:
                if np.isnan(self.relative_dis.data[0]):
                    self.target_pose_pub.publish(self.landing_velocity_position)
                else:
                    self.desired_landing_pub.publish(self.landing_velocity)
            except IndexError as e:
                self.target_pose_pub.publish(self.current_pose)
            
        if self.cmd_state == 11:
            pass
            
if __name__ == "__main__":
    rospy.init_node('control_node')
    control_node_handler = ControlClass()
    rate = rospy.Rate(100)
    # wait for FCU connection
    while not rospy.is_shutdown() and not control_node_handler.current_state.connected:
        rate.sleep()
    rospy.loginfo("Control Node : FCU connected")
    
    rospy.Service('drone_command', DroneCommand, control_node_handler.cmdreact_cb)
    rospy.Timer(rospy.Duration(0.05), control_node_handler.main_controller)

    rospy.spin()