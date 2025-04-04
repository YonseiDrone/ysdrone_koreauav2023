#!/usr/bin/env python
import rospy
import math, time
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Float32
from koreauav_utils import auto_service
		

### https://mavlink.io/en/messages/common.html  

# MAV_CMD_DO_SET_ACTUATOR (187)
# Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).

# Param (:Label)	Description																Values
# 1: Actuator 1		Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 2: Actuator 2		Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 3: Actuator 3		Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 4: Actuator 4		Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 5: Actuator 5		Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 6: Actuator 6		Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.					min: -1 max:1
# 7: Index			Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)	min:0 increment:1

# MAV_CMD_DO_GRIPPER (211)
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
		self.cargo_offset = rospy.get_param("cargo_offset")

		self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
		self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
		self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)
		self.launch_setposition_sub = rospy.Subscriber('/launch_setposition', PoseStamped, self.launch_setposition_cb)

		self.move_pub = rospy.Publisher('/move_to_launch', PoseStamped, queue_size=1)

		self.servo_init = False
		self.servo_block = False
		self.servo_launch = False
		self.t_pwm = time.time()
		self.DEBUG = False

	### Callback functions
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
	
	### Math functions
	def calc_xy_err(self, cur, dest):
		xy_err = math.sqrt((cur.pose.position.x - dest.pose.position.x)**2 + (cur.pose.position.y - dest.pose.position.y)**2)
		return xy_err

	def calc_z_err(self, cur, dest):
		z_err = math.sqrt((cur.pose.position.z - dest.pose.position.z)**2)
		return z_err
	
	# Main function
	def move_to_target(self, e):
		if self.mission == 4:
			self.move.pose.position.x = self.launch_setposition_list[-1][0]
			self.move.pose.position.y = self.launch_setposition_list[-1][1]
			self.move.pose.position.z = self.launch_setposition_list[-1][2] + self.cargo_offset
			self.move.pose.orientation.x = self.launch_setposition_list[-1][3]
			self.move.pose.orientation.y = self.launch_setposition_list[-1][4]
			self.move.pose.orientation.z = self.launch_setposition_list[-1][5]
			self.move.pose.orientation.w = self.launch_setposition_list[-1][6]
			self.move_pub.publish(self.move)

			if self.calc_xy_err(self.move, self.current_pose)<0.1 and self.calc_z_err(self.move, self.current_pose)<0.1:
				self.CARGO_MISSION()
    
		if(self.DEBUG):
			self.CARGO_MISSION()
       
	# Servo 1 (Open)
	# Note: param1 & param2 == -1 (Check QGC pwm values)
	def MAV_CMD_DO_SET_ACTUATOR_1(self):
		# rospy.loginfo('Waiting for server...')
		rospy.wait_for_service('/mavros/cmd/command')
		try:
			servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
	
			msg = CommandLong()
			resp = servo_control_srv(broadcast=False, command=187, confirmation=False, param1=-1, param2=-1, param3=0, param4=0, param5=0, param6=0, param7=0)
	
			# rospy.loginfo('Try service call...')
			if resp.success:
				rospy.loginfo("Servo 1 controlled successfully")
				# print(f"result: {resp.result}")
			else:
				rospy.loginfo("Failed to control servo 1")
			return resp.success

		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)
			return False
 

	# Servo 2 (Activate)
	# Note: param2 == 1
	def MAV_CMD_DO_SET_ACTUATOR_2(self):
		# rospy.loginfo('Waiting for server...')
		rospy.wait_for_service('/mavros/cmd/command')
		try:
			servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
	
			msg = CommandLong()
			resp = servo_control_srv(broadcast=False, command=187, confirmation=False, param1=-1, param2=1, param3=0, param4=0, param5=0, param6=0, param7=0)
	
			# rospy.loginfo('Try service call...')
			if resp.success:
				rospy.loginfo("Servo 2 controlled successfully")
				# print(f"result: {resp.result}")
			else:
				rospy.loginfo("Failed to control servo 2")
			return resp.success

		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)
			return False

	def CARGO_MISSION(self):
		if not self.servo_init:
			self.t_init = time.time()
			self.servo_init = True
   
		if not self.servo_block and time.time() - self.t_init > 2:
			self.t_pwm = time.time()
			self.servo_block = self.MAV_CMD_DO_SET_ACTUATOR_1()
		elif self.servo_block:
			if not self.servo_launch and time.time() - self.t_pwm < 2:
				pass
			elif not self.servo_launch:
				self.t_pwm = time.time()
				self.servo_launch = self.MAV_CMD_DO_SET_ACTUATOR_2()
			elif time.time() - self.t_pwm < 2:
				pass
			else:
				auto_service.call_drone_command(5)
		else:
			rospy.logwarn("Servo Error")


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

		# move_to_target() 함수를 주어진 주기마다 호출
		rospy.Timer(rospy.Duration(0.05), cargo_launch_node_handler.move_to_target)
		rospy.spin()
	except rospy.ROSInitException as exception:
		pass
	rospy.loginfo("Done")
