#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import OverrideRCIn
		

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
def MAV_CMD_DO_SET_SERVO():
	rospy.loginfo('Waiting for server...')
	rospy.wait_for_service('/mavros/cmd/command')
	try:
		servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
  
		resp = servo_control_srv(broadcast=False, command=183, confirmation=False, param1=1, param2=2000, param3=0, param4=0, param5=0, param6=0, param7=0)
  
		rospy.loginfo('Try service call...')
		if resp.success:
			print("Servo controlled successfully")
			print(f"result: {resp.result}")
		else:
			print("Failed to control servo")

	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)
  
  
def MAV_CMD_DO_SET_ACTUATOR():
	rospy.loginfo('Waiting for server...')
	rospy.wait_for_service('/mavros/cmd/command')
	try:
		servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
  
		msg = CommandLong()
		resp = servo_control_srv(broadcast=False, command=187, confirmation=False, param1=1, param2=0, param3=0, param4=0, param5=1, param6=1, param7=0)
  
		rospy.loginfo('Try service call...')
		if resp.success:
			print("Servo controlled successfully")
			print(f"result: {resp.result}")
		else:
			print("Failed to control servo")

	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)
  
  
def MAV_CMD_DO_GRIPPER():
	
	try:
		servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
		msg = CommandLong()
		msg.command = 211 # MAV_CMD_DO_GRIPPER
		msg.param2 = 1
		# QGC에서 Gripper로 설정해놓은 Servo PIN에 Action을 준다.(true)
  
		resp = servo_control_srv(msg)
		if resp.success:
			print("Servo controlled successfully")
		else:
			print("Failed to control servo")

	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)


if __name__ == "__main__":
	rospy.init_node('cargo_launch', anonymous=True)
	rospy.wait_for_service('/mavros/cmd/command')
	# MAV_CMD_DO_GRIPPER()
	# MAV_CMD_DO_SET_ACTUATOR()
	MAV_CMD_DO_SET_SERVO()
	rospy.loginfo("Done")
