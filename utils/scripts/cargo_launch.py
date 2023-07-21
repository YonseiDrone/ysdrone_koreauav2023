#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import OverrideRCIn
		
def mav_cmd_187():
	
	try:
		servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
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

###
		resp = servo_control_srv(False, 183, 94, 1, 1500, 0, 0, 0, 0, 0)
	
		if resp.success:
			print("Servo controlled successfully")
		else:
			print("Failed to control servo")

	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)
  
def set_pwm():
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    # Give the node a moment to initialize and create the publisher
    rospy.sleep(1)

    msg = OverrideRCIn()
    msg.channels[0] = 2000  # set servo at channel 1 to mid-point
    # keep the other channels at 0
    for i in range(1, 8):
        msg.channels[i] = 2000

    pub.publish(msg)
    rospy.loginfo("Published PWM values to /mavros/rc/override") 

if __name__ == "__main__":
	rospy.init_node('cargo_launch', anonymous=True)
	rospy.wait_for_service('/mavros/cmd/command')
	mav_cmd_187()
	set_pwm()
	rospy.loginfo("Done")
