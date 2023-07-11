#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import random

# Change the mode of drone
from mavros_msgs.msg import OffboardControlMode, PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

import onnxruntime


class RL_Landing(object):
    def __init__(self):
        rospy.init_node('rl_landing_node')

        # Set up ROS publishers and subscribers
        self.position_setpoint_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.offboard_control_mode_publisher = rospy.Publisher('/mavros/set_mode', SetMode, queue_size=1)
        self.arm_command_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.position_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        self.velocity_subscriber = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_callback)
        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # Initialize variables
        self.counter = 0
        self.position = PoseStamped()
        self.velocity = TwistStamped()
        self.state = State()

        # Randomly generate takeoff position
        self.takeoff_pos = [-random.random()*4, random.random()*4, -(random.random()*4 + 8)]
        rospy.loginfo(f'Takeoff to x: {self.takeoff_pos[0]:.3f}, y: {self.takeoff_pos[1]:.3f}, z: {self.takeoff_pos[2]:.3f}')

        # First takeoff is in position mode, landing is in velocity mode
        self.posmode = True

        # Load ONNX model
        self.model = onnxruntime.InferenceSession("DroneLanding-8078831.onnx")

        # Create a timer to publish control commands
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def get_state(self):
        state = []
        state.append(-self.position.pose.position.x)
        state.append(-self.position.pose.position.y)
        state.append(self.position.pose.position.z)
        state.append(self.velocity.twist.linear.x)
        state.append(self.velocity.twist.linear.y)
        state.append(self.velocity.twist.linear.z)
        return state

    def position_callback(self, msg):
        self.position = msg

    def velocity_callback(self, msg):
        self.velocity = msg

    def state_callback(self, msg):
        self.state = msg

    def arm(self):
        """Send an arm command to the vehicle."""
        return self.arm_command_service(True)

    def disarm(self):
        """Send a disarm command to the vehicle."""
        return self.arm_command_service(False)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        mode_msg = SetMode()
        mode_msg.custom_mode = "OFFBOARD"
        self.offboard_control_mode_publisher.publish(mode_msg)
        rospy.loginfo("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        mode_msg = SetMode()
        mode_msg.custom_mode = "AUTO.LAND"
        self.offboard_control_mode_publisher.publish(mode_msg)
        rospy.loginfo("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                        PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "local_origin"
        msg.velocity.z = 0.0  # Set zero velocity in z-axis
        self.position_setpoint_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the position setpoint."""
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                        PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "local_origin"
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        self.position_setpoint_publisher.publish(msg)
        # rospy.loginfo(f"Publishing position setpoints {[x, y, z]}")

    def timer_callback(self, event):
        self.publish_offboard_control_heartbeat_signal()
        # Offboard mode start
        if self.counter == 10:
            self.engage_offboard_mode()
            self.arm()
            rospy.loginfo("TakeOff!!")
        # Takeoff
        if self.counter >= 20 and self.counter < 200 and self.state.mode == "OFFBOARD":
            self.publish_position_setpoint(self.takeoff_pos[0], self.takeoff_pos[1], self.takeoff_pos[2])

            if self.counter == 190:
                rospy.loginfo("Landing Start")

        # Landing (inferencing)
        elif self.counter == 200 and self.position.pose.position.z < -0.05:
            self.posmode = False
            state = self.get_state()
            ort_inputs = {self.model.get_inputs()[0].name: [state]}
            action = self.model.run(None, ort_inputs)
            action = np.multiply(action[2][0], [0.0125, 0.0125, 0.0125/2])
            self.publish_position_setpoint(action[0], action[1], action[2])

        # Landing
        elif self.counter == 200:
            rospy.loginfo("Landing Complete!")
            self.land()
            rospy.signal_shutdown('Landing Complete!')

        # Counter
        if self.counter < 200:
            self.counter += 1

def main():
    rl_landing = RL_Landing()
    rospy.spin()

if __name__ == '__main__':
    main()
