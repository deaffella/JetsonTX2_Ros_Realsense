# The class to control the robot.

import serial
import rospy
from std_msgs.msg import Int8


class FourXFourBotControl():
    def __init__(self):
        # self.ser = serial.Serial('/dev/ttyUSB1')  # Connection to the wheels via USB
        # Try sudo usermod -a -G tty tx2, sudo usermod -a -G dialout tx2 before the start.
        #self.ser.baudrate = 460800  # baudrate 115200 is standard
        # about rpm: 120 is max, moreover, you shouldn't use rpm less then 10
        # wheels configuration: ST0+Forward Left/Forward Right/Back Right/Back Left, + means roll right, - - left

        self.lb_pub = rospy.Publisher('rpm_leftBack_sub', Int8, queue_size=10)
        self.lf_pub = rospy.Publisher('rpm_leftFront_sub', Int8, queue_size=10)
        self.rb_pub = rospy.Publisher('rpm_rightBack_sub', Int8, queue_size=10)
        self.rf_pub = rospy.Publisher('rpm_rightFront_sub', Int8, queue_size=10)

    def set_wheels_frequency(self, left_wheels, right_wheels):
        """
        Manual wheels speed control.
        """
        self.lb_pub.publish(-left_wheels)
        self.lf_pub.publish(-left_wheels)
        self.rb_pub.publish(right_wheels)
        self.rf_pub.publish(right_wheels)
