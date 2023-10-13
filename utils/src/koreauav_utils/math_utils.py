import rospy
import math
from ysdrone_msgs.srv import *

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

if __name__ == "__main__":
    rospy.init_node('math_utils', anonymous=True)