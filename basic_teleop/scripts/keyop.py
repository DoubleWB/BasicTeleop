#!/usr/bin/env python
# license removed for brevity
import rospy
import tty
import sys
from std_msgs.msg import String
from basic_teleop.msg import *

def talker():
    fd = sys.stdin.fileno()
    tty.setraw(fd)
    pub = rospy.Publisher('movement', Move, queue_size=10)
    rospy.init_node('director', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    print "ready to read"
    while not rospy.is_shutdown():
        key_str = sys.stdin.read(1)
	topic = Move()
        if key_str == 'w':
	    topic = Move("fwd", 1)
        elif key_str == 's':
            topic = Move("bwd", 1)
        elif key_str == 'a':
            topic = Move("ccw", 1)
        elif key_str == 'd':
            topic = Move("cw", 1)
        elif key_str == 'q':
	    quit()
        rospy.loginfo(topic)
        pub.publish(topic)
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
