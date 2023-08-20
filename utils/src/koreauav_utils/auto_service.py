import rospy
from ysdrone_msgs.srv import *

def call_drone_command(data):
    srv_mode = rospy.get_param('srv_mode', True)

    if srv_mode == True:
        pass
    else:
        rospy.wait_for_service('/drone_command')
        try:
            service = rospy.ServiceProxy('/drone_command', DroneCommand)
            request = DroneCommandRequest()
            request.command = data
            respose = service(request)
            return respose
        except rospy.ServiceException as e:
            rospy.loginfo(f'Service Call failed: {e}')

if __name__ == "__main__":
    rospy.init_node('auto_service', anonymous=True)