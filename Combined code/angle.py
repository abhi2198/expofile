import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import time
import math


class angle:

    def dir(self, msg):
        self.angle = math.degrees(math.atan2(msg.twist.linear.y, msg.twist.linear.x))
        self.get = 1

    def __init__(self):

        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.dir)
#        rospy.init_node('angle_of_direction')
        rate = rospy.Rate(10)
        self.angle = 90
        self.get = 0

    def get_angle(self):
        while True:
            if self.get != 0:
                break
        self.angle = self.angle - 180
        if self.angle < 0:
            self.angle = self.angle + 360
        return self.angle

if '__name__' == '__main__':

    a = angle()
