#!/usr/bin/env python
import rospy
import time
import RPi.GPIO as GPIO

from std_msgs.msg import String
from basic_teleop.msg import *

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17,GPIO.OUT)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.direction)
    if data.direction == "bwd":
	GPIO.output(17, True)
   	GPIO.output(18, False)
	GPIO.output(22, False)
	GPIO.output(23, True)
    elif data.direction == "fwd":
        GPIO.output(17, False)
        GPIO.output(18, True)
        GPIO.output(22, True)
        GPIO.output(23, False)
    elif data.direction == "cw":
        GPIO.output(17, False)
        GPIO.output(18, True)
        GPIO.output(22, False)
        GPIO.output(23, True)
    elif data.direction == "ccw":
        GPIO.output(17, True)
        GPIO.output(18, False)
        GPIO.output(22, True)
        GPIO.output(23, False)
    time.sleep(data.duration/2)
    GPIO.output(17, False)
    GPIO.output(18, False)
    GPIO.output(22, False)
    GPIO.output(23, False)

    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('driver', anonymous=True)

    rospy.Subscriber("movement", Move, callback)
    print "ready to hear"
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
