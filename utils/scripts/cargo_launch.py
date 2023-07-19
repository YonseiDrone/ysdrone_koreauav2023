#!/usr/bin/env python

import rospy
from mavros_msgs.srv import ParamSet, ParamGet, CommandLong
from mavros_msgs.msg import ParamValue

def get_mavros_param():
    # Wait for the service to become available
    rospy.wait_for_service('/mavros/param/get')

    try:
        # Create a handle to the service
        param_get_service = rospy.ServiceProxy('/mavros/param/get', ParamGet)

        # Call the service
        resp = param_get_service(param_id='RC_MAP_AUX1')
        
        if resp.success:
            print("Parameter RC_MAP_AUX1 fetched successfully")
            print("Integer value: ", resp.value.integer)
            print("Real value: ", resp.value.real)
        else:
            print("Failed to fetch parameter")

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        
def set_mavros_param():
    rospy.init_node('cargo_launch', anonymous=True)

    # Wait for the service to become available
    rospy.wait_for_service('/mavros/param/set')
    rospy.wait_for_service('/mavros/cmd/command')
    
    try:
        # Create a handle to the service
        param_set_service = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        servo_control_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        # Create the parameter value
        param_value = ParamValue()
        param_value.integer = 1
        param_value.real = 0.0

        # Call the service
        resp_set = param_set_service(param_id='RC_MAP_AUX1', value=param_value)
        if resp_set.success:
            print("Parameter set successfully")
        else:
            print("Failed to set parameter")
        get_mavros_param()
        
    	try:
        resp = servo_control_service(False, 187, 94, 1, 1500, 0, 0, 0, 0, 0)
        
		if resp.success:
		    print("Servo controlled successfully")
		else:
		    print("Failed to control servo")

    	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    set_mavros_param()

