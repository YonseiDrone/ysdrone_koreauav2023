#!/usr/bin/env python

import rospy
from mavros_msgs.srv import ParamSet, ParamGet
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

    try:
        # Create a handle to the service
        param_set_service = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        # Create the parameter value
        param_value = ParamValue()
        param_value.integer = 1
        param_value.real = 0.0

        # Call the service
        resp = param_set_service(param_id='RC_MAP_AUX1', value=param_value)
        if resp.success:
            print("Parameter set successfully")
        else:
            print("Failed to set parameter")
        get_mavros_param()
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    set_mavros_param()

