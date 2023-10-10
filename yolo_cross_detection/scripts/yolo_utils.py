import rospy
# import msgs...
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

#==========================Debugging Functions============================================
def make_cube_marker(pos, color, scale): # make cube marker for gazebo
        marker = Marker()
        marker.type = Marker.CUBE
        marker.header.frame_id = 'local_origin'
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.position = Point(pos[0], pos[1], pos[2])
        return marker

def make_line_marker(pos, radius): # make line marker for visualize potential field bound
    marker2 = Marker()
    marker2.lifetime = rospy.Duration()
    marker2.header.frame_id = "local_origin"
    marker2.type = Marker.LINE_STRIP
    marker2.action = Marker.ADD
    marker2.color.a = 1.0
    marker2.color.r = 1.0
    marker2.color.g = 1.0
    marker2.color.b = 1.0
    marker2.scale.x = 0.1

    points = []
    samples = 50
    for i in range(samples):
        x = pos[0] + radius * math.cos(math.pi * 2.0 * float(i) / float(samples))
        y = pos[1] + radius * math.sin(math.pi * 2.0 * float(i) / float(samples))
        z = pos[2]
        points.append(Point(x, y, z))
    marker2.points = points
    return marker2


#=============================Util Functions===================================
def to_euler_angles(x, y, z, w): # quarternion to euler angles
    # roll(x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    angles_roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch(y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        angles_pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        angles_pitch = math.asin(sinp)
    
    # yaw(z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    angles_yaw = math.atan2(siny_cosp, cosy_cosp)

    return angles_roll, angles_pitch, angles_yaw

def square_sampling(left_top, right_bottom, interval=2): # sample points from marker
    result = []
    x_start, y_start = left_top
    x_end, y_end = right_bottom

    for x in range(x_start, x_end, interval):
        for y in range(y_start, y_end, interval):
            result.append((x, y))

    return result