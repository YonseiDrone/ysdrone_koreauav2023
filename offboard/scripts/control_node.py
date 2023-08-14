#! /usr/bin/env python3
import rospy
import math
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.msg import State, PositionTarget
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dynamic_reconfigure.client import Client

from ysdrone_msgs.srv import *
#====================================================================
# Mathmatical conversion functions
def rad2deg(radian):
    return radian * 180 / math.pi

def deg2rad(degree):
    return degree * math.pi /180

def to_quaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy

    return qx, qy, qz, qw

def to_euler_angles(x, y, z, w):
    # roll(x-axis rotation)
    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    angles_roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch(y-axis rotation)
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        angles_pitch = math.copysign(math.pi/2, sinp) # use 90 degrees if out of range
    else:
        angles_pitch = math.asin(sinp)
    
    # yaw(z-axis rotation)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    angles_yaw = math.atan2(siny_cosp, cosy_cosp)

    return angles_roll, angles_pitch, angles_yaw
#====================================================================

#====================================================================
# Fucntions to limit the velocity
# var : current velocity, val : limit velocity
# val(limit velocity) is always positive 이거 나중에 다시 확인하자!!!!!!
def lim(var, val):
    if val >= 0:
        if var > val:
            var = val
    elif val < 0:
        if var < (-1) * val:
            var = (-1) * val
    return var

def lim_z_vel(var, val):
    if var >= 0:
        if var > val:
            var = val
    elif var < 0:
        if var < (-1) * val:
            var = (-1) * val
    return var
#====================================================================

class ControlClass(object):
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.destination_command_marker = Marker()
        self.destination_command_marker_array = MarkerArray()
        self.cmd_state = 0
        self.isly_destination_command_marker = Marker()
        self.isly_destination_command_marker_array = MarkerArray()
        self.desired_landing = PositionTarget()
        self.mission_num = Float32()
        self.RL_target_vel = Twist()
        self.launch_setposition = PoseStamped()
        self.launch_setposition_marker = Marker()
        self.launch_setposition_marker_array = MarkerArray()
        self.dynamic_client = Client('/local_planner_node', timeout=30)
        self.avoidance = PoseStamped()
        self.building_target = PoseStamped()
        self.building_target_marker = Marker()
        self.building_target_marker_array = MarkerArray()
    
        #Subscriber
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb) 
        self.destination_command_sub = rospy.Subscriber('/destination_command', PoseStamped, self.destination_command_cb)
        self.isly_destination_command_sub = rospy.Subscriber('/isly_destination_command', PoseStamped, self.isly_destination_command_cb)
        self.desired_landing_sub = rospy.Subscriber('/desired_landing', PositionTarget, self.desired_landing_cb)
        self.RL_target_vel_sub = rospy.Subscriber('/landing_velocity', Twist, self.RL_target_vel_cb)
        self.launch_setposition_sub = rospy.Subscriber('/launch_setposition', PoseStamped, self.launch_setposition_cb)
        self.avoidance_pos_sub = rospy.Subscriber('/avoidance/setpoint_position/local', PoseStamped, self.avoidance_pos_cb)
        self.building_target_sub = rospy.Subscriber('/building/search/target_pose', PoseStamped, self.building_target_cb)

        #Publisher
        self.target_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.avoidance_pos_pub = rospy.Publisher('input/goal_position', MarkerArray, queue_size=1)
        self.landing_velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        self.desired_landing_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        
        self.mission_pub = rospy.Publisher('/mission', Float32, queue_size=1)
    
    #=====================TODO=======================
    # MISSION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
    def avoidance_pos_cb(self, msg):
        # self.avoidance = msg
        # if self.cmd_state in [1,4,5]:
        #     self.target_pose_pub.publish(self.avoidance)
        # elif self.cmd_state == 2:
        #     self.avoidance = self.building_target
        #     self.target_pose_pub.publish(self.avoidance)
        # elif self.cmd_state==3:
        #     self.avoidance = self.launch_setposition
        #     rospy.loginfo(self.avoidance)
        #     self.target_pose_pub.publish(self.avoidance)
        self.avoidance = msg
        if self.cmd_state == 2 and self.building_target.pose.position.z != 0:
            rospy.loginfo(f"Building_search")
            self.avoidance = self.building_target
        elif self.cmd_state==3 and self.launch_setposition.pose.position.z != 0:
            rospy.loginfo(f"launch_setposition")
            self.avoidance = self.launch_setposition

        rospy.loginfo(self.avoidance)
        self.target_pose_pub.publish(self.avoidance)
    #====================================================

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
        self.desired_landing.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
        self.desired_landing.velocity.x = msg.velocity.x
        self.desired_landing.velocity.y = msg.velocity.y
        self.desired_landing.velocity.z = msg.velocity.z
        self.desired_landing.yaw = msg.yaw
        self.desired_landing.yaw_rate = 1

    def destination_command_cb(self, msg):
        # 여기 나중에 pre_destioination_command, destination_command 찍어서 디버깅해보자!!
        # if (self.destination_command.pose.position.x != msg.pose.position.x) or (self.destination_command.pose.position.y != msg.pose.position.y) or (self.destination_command.pose.position.z != msg.pose.position.z):
        #     self.pre_destination_command.pose.position.x = self.destination_command.pose.position.x
        #     self.pre_destination_command.pose.position.y = self.destination_command.pose.position.y
        #     self.pre_destination_command.pose.position.z = self.destination_command.pose.position.z
        # self.destination_command.pose.position.x = msg.pose.position.x
        # self.destination_command.pose.position.y = msg.pose.position.y
        # self.destination_command.pose.position.z = msg.pose.position.z
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
        # if (self.isly_destination_command.pose.position.x != msg.pose.position.x) or (self.isly_destination_command.pose.position.y != msg.pose.position.y) or (self.isly_destination_command.pose.position.z != msg.pose.position.z):
        #     self.pre_isly_destination_command.pose.position.x = self.isly_destination_command.pose.position.x
        #     self.pre_isly_destination_command.pose.position.y = self.isly_destination_command.pose.position.y
        #     self.pre_isly_destination_command.pose.position.z = self.isly_destination_command.pose.position.z
        # self.isly_destination_command.pose.position.x = msg.pose.position.x
        # self.isly_destination_command.pose.position.y = msg.pose.position.y
        # self.isly_destination_command.pose.position.z = msg.pose.position.z
        self.isly_destination_command_marker.pose.position.x = msg.pose.position.x
        self.isly_destination_command_marker.pose.position.y = msg.pose.position.y
        self.isly_destination_command_marker.pose.position.z = msg.pose.position.z
        self.isly_destination_command_marker_array.markers.clear()
        self.isly_destination_command_marker_array.markers.append(self.isly_destination_command_marker)
    
    def RL_target_vel_cb(self, msg):
        self.RL_target_vel = msg
    
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
            self.resp.mode = 'Building Searching Mode'
            self.resp.res = True
        elif self.cmd_state == 3:
            self.resp.mode = 'Cross Detection Mode'
            self.resp.res = True
        elif self.cmd_state == 4:
            self.resp.mode = 'Cargo Launching Mode'
            self.resp.res = True
        elif self.cmd_state == 5:
            self.resp.mode = 'Of course I Still Love You'
            self.resp.res = True
        elif self.cmd_state == 6:
            self.resp.mode = 'Safety Landing Mode'
            self.resp.res = True
        elif self.cmd_state == 7:
            self.resp.mode = 'Position Control Mode'
            self.resp.res = True
        elif self.cmd_state == 8:
            self.resp.mode = 'Position Landing'
            self.resp.res = True
        elif self.cmd_state == 9:
            self.resp.mode = 'RL Landing with GPS'
            self.resp.res = True
        elif self.cmd_state == 10:
            self.resp.mode = 'RL Landing with Aruco'
            self.resp.res = True
        elif self.cmd_state == 11:
            self.resp.mode = 'Landing and disarming'
            self.resp.res = True
        rospy.loginfo(f'Received request : {req.command} && Current Mode : {self.resp.mode} && Enable :{self.resp.res}')
        return self.resp
    
    def main_controller(self, e):
        self.time_now = rospy.Time.now()
        if self.cmd_state == 0:
            self.target_pose.pose.position.x = 0
            self.target_pose.pose.position.y = 0
            self.target_pose.pose.position.z = 4.2
            self.target_pose_pub.publish(self.target_pose)
        # Mission 1(Obstacle Avoidance Planner)
        if self.cmd_state == 1:
            new_config = {"obstacle_cost_param_": 5}
            config = self.dynamic_client.update_configuration(new_config)
            self.avoidance_pos_pub.publish(self.destination_command_marker_array)
            #rospy.loginfo(f"{self.destination_command_marker_array}")
            #rospy.loginfo(f'Target Waypoint - x :{self.destination_command_marker.pose.position.x}, y :{self.destination_command_marker.pose.position.y}, z :{self.destination_command_marker.pose.position.z}')

        # Mission 2(Building Searching)
        if self.cmd_state == 2:
            self.mission_num.data = self.cmd_state
            self.mission_pub.publish(self.mission_num)
            self.avoidance_pos_pub.publish(self.building_target_marker_array)
            #rospy.loginfo(f"Mission published to [Building Search] data: {self.mission_num.data}")

        
        # Mission 3(Cross Detection Mode)
        if self.cmd_state == 3:
            self.mission_num.data = self.cmd_state
            self.mission_pub.publish(self.mission_num)
            new_config = {"obstacle_cost_param_": 1}
            config = self.dynamic_client.update_configuration(new_config)

            self.avoidance_pos_pub.publish(self.launch_setposition_marker_array) 
            #rospy.loginfo(f"Mission published to [Cross Detection] data: {self.mission_num.data}")


        # Mission 4(Cargo Launching Mode)
        if self.cmd_state == 4:
            pass

        # Mission 5(Of course I Still Love you)
        if self.cmd_state == 5:
            new_config = {"obstacle_cost_param_": 5}
            config = self.dynamic_client.update_configuration(new_config)
            self.avoidance_pos_pub.publish(self.isly_destination_command_marker_array)
            rospy.loginfo(f'Target Waypoint - x :{self.isly_destination_command_marker.pose.position.x}, y :{self.isly_destination_command_marker.pose.position.y}, z :{self.isly_destination_command_marker.pose.position.z}')

        # Mission 6(Safety Landing)
        if self.cmd_state == 6:
            self.desired_landing_pub.publish(self.desired_landing)
            rospy.loginfo(f"Velocity - x: {self.desired_landing.velocity.x}, y: {self.desired_landing.velocity.y}, z : {self.desired_landing.velocity.z}")
        
        
        # Mission 7(Position Control)
        if self.cmd_state == 7:
            self.target_pose.pose.position.x = 0.0
            self.target_pose.pose.position.y = 0.0
            self.target_pose.pose.position.z = 4.0
            self.target_pose_pub.publish(self.target_pose)

        # Mission 8(Position Landing)    
        if self.cmd_state == 8:
            self.target_pose.pose.position.x = 0.0
            self.target_pose.pose.position.y = 0.0
            self.target_pose.pose.position.z = self.current_pose.pose.position.z - 0.2
            self.target_pose_pub.publish(self.target_pose)

        # Mission 9(RL Landing with GPS)
        if self.cmd_state == 9:
            self.landing_velocity_pub.publish(self.RL_target_vel)
            rospy.loginfo(f"Velocity - x: {self.RL_target_vel.linear.x}, y: {self.RL_target_vel.linear.y}, z: {self.RL_target_vel.linear.z}")

        # Mission 10(RL Landing with Aruco)
        if self.cmd_state == 10:
            qx, qy, qz, qw = to_quaternion(90*math.pi/180, 0, 0)
            self.target_pose.pose.orientation.x = qx
            self.target_pose.pose.orientation.y = qy
            self.target_pose.pose.orientation.z = qz
            self.target_pose.pose.orientation.w = qw
            self.target_pose_pub.publish(self.target_pose) 
            self.landing_velocity_pub.publish(self.RL_target_vel)
            rospy.loginfo(f"Velocity - x: {self.RL_target_vel.linear.x}, y: {self.RL_target_vel.linear.y}, z: {self.RL_target_vel.linear.z}")
            
        if self.cmd_state == 11:
            msg = Float32()
            msg.data = self.cmd_state
            self.mission_pub.publish(msg)
            






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












