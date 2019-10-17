#!/usr/bin/env python
import rospy
from robot_msgs.msg import GimbalInfo


if __name__ == '__main__':
    rospy.init_node('gimbal_test_node', anonymous=True)
    pub = rospy.Publisher('gimbal_control', GimbalInfo, queue_size=10)
    info=GimbalInfo()
    while True:
        print "\n[absolute mode],please intput tagret angle (float) "
        try:
            info.pitch_angle = float(raw_input("pitch_angle: "))
            info.yaw_angle = float(raw_input("yaw_angle: "))
        except:
            break
        pub.publish(info)
        print "send--------------------------------\n"
                                

