#!/usr/bin/env python
import rospy
import math
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Float32
from koreauav_utils import auto_service
		

### https://mavlink.io/en/messages/common.html  
 
# MAV_CMD_DO_SET_SERVO (183 )
# Set a servo to a desired PWM value.

# Param (:Label)	Description	Values		Units
# 1: Instance		Servo instance number.	min:0 increment:1	
# 2: PWM			Pulse Width Modulation.	min:0 increment:1	us
# 3	Empty		
# 4	Empty		
# 5	Empty		
# 6	Empty		
# 7	Empty		

# MAV_CMD_DO_SET_ACTUATOR (187 )
# Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).

# Param (:Label)	Description																Values
# 1: Actuator 1		Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 2: Actuator 2		Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 3: Actuator 3		Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 4: Actuator 4		Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 5: Actuator 5		Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 6: Actuator 6		Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 7: Index			Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)	min:0 increment:1

# MAV_CMD_DO_GRIPPER (211 )
# Mission command to operate a gripper.

# Param (:Label)	Description	Values
# 1: Instance	Gripper instance number.	min:1 increment:1
# 2: Action	Gripper action to perform.	GRIPPER_ACTIONS
# 3	Empty	
# 4	Empty	
# 5	Empty	
# 6	Empty	
# 7	Empty	

###
class CargoLaunch(object):
	def __init__(self):
		self.mission = 0
		self.current_state = State()
		self.current_pose = PoseStamped()
		self.launch_setposition = PoseStamped()
		self.launch_setposition_list = []
		self.move = PoseStamped()
		self.offset = 1.0

		self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
		self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
		self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)
		self.launch_setposition_sub = rospy.Subscriber('/launch_setposition', PoseStamped, self.launch_setposition_cb)

		self.move_pub = rospy.Publisher('/move_to_launch', PoseStamped, queue_size=1)

	def launch_setposition_cb(self, msg):
		self.launch_setposition = msg
		self.launch_setposition_list.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

	def state_cb(self, msg):
		prev_state = self.current_state
		self.current_state = msg
		if self.current_state.mode != prev_state.mode:
			rospy.loginfo(f"Current Mode : {self.current_state.mode}")
		if self.current_state.armed != prev_state.armed:
			rospy.loginfo(f"Vehicle armed : {self.current_state.armed}")

	def pose_cb(self, msg):
		self.current_pose = msg
	def mission_cb(self, msg):
		self.mission = msg.data
	
	def calc_xy_err(self, cur, dest):
		xy_err = math.sqrt((cur.pose.position.x - dest.pose.position.x)**2 + (cur.pose.position.y - dest.pose.position.y)**2)
		return xy_err

	def calc_z_err(self, cur, dest):
		z_err = math.sqrt((cur.pose.position.z - dest.pose.position.z)**2)
		return z_err
	
	def move_to_target(self, e):
		if self.mission == 4:
			self.move.pose.position.x = self.launch_setposition_list[-1][0]
			self.move.pose.position.y = self.launch_setposition_list[-1][1]
			self.move.pose.position.z = self.launch_setposition_list[-1][2] + self.offset
			self.move.pose.orientation.x = self.launch_setposition_list[-1][3]
			self.move.pose.orientation.y = self.launch_setposition_list[-1][4]
			self.move.pose.orientation.z = self.launch_setposition_list[-1][5]
			self.move.pose.orientation.w = self.launch_setposition_list[-1][6]
			self.move_pub.publish(self.move)

			if self.calc_xy_err(self.move, self.current_pose)<0.1 and self.calc_z_err(self.move, self.current_pose)<0.1:
				#TODO: Timer가 다시 돌아도 Actuator 제어가 한번만 되도록 mission을 0으로 초기화함. 
				self.mission = 0
				rospy.loginfo("Cargo Launching!!")
				# self.MAV_CMD_DO_SET_ACTUATOR(0, 1)
				# sleep(3)
				self.MAV_CMD_DO_SET_ACTUATOR(1, 0)
				# sleep(3)
				auto_service.call_drone_command(5)

	def MAV_CMD_DO_SET_ACTUATOR(self, param_1, param_2):
		rospy.loginfo('Waiting for server...')
		rospy.wait_for_service('/mavros/cmd/command')
		try:
			servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
	
			msg = CommandLong()
			resp = servo_control_srv(broadcast=False, command=187, confirmation=False, param1=param_1, param2=param_2, param3=0, param4=0, param5=1, param6=1, param7=0)
	
			rospy.loginfo('Try service call...')
			if resp.success:
				print("Servo controlled successfully")
				print(f"result: {resp.result}")
			else:
				print("Failed to control servo")

		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)


if __name__ == "__main__":
	rospy.init_node('cargo_launch', anonymous=True)
	rospy.wait_for_service('/mavros/cmd/command')
	try:
		cargo_launch_node_handler = CargoLaunch()
		rate = rospy.Rate(100)
		# wait for FCU connection
		while not rospy.is_shutdown() and not cargo_launch_node_handler.current_state.connected:
			rate.sleep()
		rospy.loginfo(f"Cargo Launch Node : FCU connected")

		rospy.Timer(rospy.Duration(0.05), cargo_launch_node_handler.move_to_target)
		rospy.spin()
	except rospy.ROSInitException as exception:
		pass
	rospy.loginfo("Done")
