#!/usr/bin/env python

#ssh pi@192.168.0.100 - robo
#ssh pi@10.0.0.49 - becker1a

import rospy
import time
import numpy

from std_msgs.msg import String
from sensor_msgs.msg import *
from basic_teleop.msg import *

class BBox(object):
    
    def __init__(self, topX, topY, botX, botY):
	self.topX = topX
	self.topY = topY
	self.botX = botX
	self.botY = botY

    def __str__(self):
	return "Top corner is {0}, {1}.\n Bottom corner is {2}, {3}.\n center is {4}.".format(self.topX, self.topY, self.botX, self.botY, self.getCenter())

    def getCenter(self):
	return [(self.botX + (self.botX - self.topX)//2), (self.botY + (self.botY - self.topY)//2)]

    def getArea(self):
	return (self.botX - self.topX) * (self.botY - self.topY)

class DepthGrid(object):

    def __init__(self, data, width, height):
	normalized = numpy.fromstring(data, numpy.uint16)
        #grid = []
	grid = {}    	
	curheight = 0
    	curwidth = 0
    	rawPos = 0
    	while (curheight < height):
	    #row = []
	    while (curwidth < width):
                #row.append(normalized[rawPos])
		grid[(curwidth, curheight)] = normalized[rawPos]
            	rawPos += 1
            	curwidth += 1
	    #grid.append(row)
	    curheight += 1
	    curwidth = 0
    	self.depths = grid
	self.width = width
	self.height = height

    def refresh(self, data):
	normalized = numpy.fromstring(data, numpy.uint16)
	curheight = 0
	curwidth = 0
	rawPos = 0
	while (curheight < self.height): 
	    while (curwidth < curwidth):
		self.depths[(curwidth, curheight)] = normalized[rawPos]
		rawPos += 1
		curwidth += 1
	    curheight += 1
            curwidth = 0

    def getXY(self, x, y):
	return self.depths[(x, y)]#self.depths[y][x]

class DepthArray(object):

    def __init__(self, data, width, height):
	self.depths = numpy.fromstring(data, numpy.uint16)
	self.width = width
	self.height = height

    def getXY(self, x, y):
	return self.depths[(y * self.width) + x]

historyOfCenter=[]

def callback(data):
    rospy.loginfo(rospy.get_caller_id())
    #print data.height
    #print data.width
    #print len(data.data)
    #print data.encoding
    #print data.step

    frame = DepthArray(data.data, data.width, data.height)

    directTowardsSuitableBlob(frame)

def directTowardsClosestPoint(grid):
    least = 99999999
    unacceptablyClose = 0
    leastX = 0
    leastY = 0
    for x in range (0, len(grid[0])):
	for y in range (0, len(grid)):
            if ((grid.getXY(x, y) <= least) and (grid.getXY(x, y) !=0)):
		least = grid.getXY(x, y)
		leastX = x
		leastY = y
    if (leastX <= 120):
	print "left"
        pub.publish(Move("ccw", 1))
    elif (leastX >= 200):
	print "right"
	pub.publish(Move("cw", 1))
    else:
	print "stop"
	pub.publish(Move("fwd", 1))

def findSimpleBBoxIncluding(grid, x, y):
    standard = grid.getXY(x, y)
    topX= x
    bottomX = x
    topY = y
    bottomY = y
    while ((grid.getXY(bottomX, bottomY) < (standard + 10)) and (grid.getXY(bottomX, bottomY) > (standard - 10)) and (bottomX < grid.width - 1)):
	standard = grid.getXY(bottomX, bottomY)
	bottomX += 1
    while ((grid.getXY(bottomX, bottomY) < (standard + 10)) and (grid.getXY(bottomX, bottomY) > (standard - 10)) and (bottomY < grid.height - 1)):
	standard = grid.getXY(bottomX, bottomY)
	bottomY += 1
    
    standard = grid.getXY(x, y)

    while ((grid.getXY(topX, topY) < (standard + 10)) and (grid.getXY(topX, topY) > (standard - 10)) and (topX > 0)):
	standard = grid.getXY(topX, topY)
	topX -= 1
    while ((grid.getXY(topX, topY) < (standard + 10)) and (grid.getXY(topX, topY) > (standard - 10)) and (topY > 0)):
	standard = grid.getXY(topX, topY)
	topY -= 1
    return BBox(topX, topY, bottomX, bottomY)

def findSuitableBBox(grid):
    output = BBox(0, 0, 0, 0)
    most = 0
    least = 99999999
    leastX = 0
    leastY = 0
    attemptCount = 0
    while(output.getArea() < 50):
        for x in range (0, grid.width):
	    for y in range (0, grid.height):
                if ((grid.getXY(x, y) <= least) and (grid.getXY(x, y) > most)):
		    least = grid.getXY(x, y)
		    leastX = x
		    leastY = y
        most = least
	output = findSimpleBBoxIncluding(grid, leastX, leastY)
	least = 99999999
	attemptCount += 1
        if (attemptCount > 4):
	    break	
    return output

def directTowardsSuitableBlob(grid):
    blob = findSuitableBBox(grid)
    blobCenter = blob.getCenter()
    if (blobCenter[0] <= 140):
	print "Blob found to the left"
        pub.publish(Move("ccw", 1))
    elif (blobCenter[0] >= 180):
	print "Blob found to the right"
	pub.publish(Move("cw", 1))
    else:
	print "Blob found in center; stop"
	pub.publish(Move("fwd", 1))


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('driver', anonymous=True)
    #rospy.Subscriber("movement", Move, callback)
    print "ready to listen + learn!"
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    print historyOfCenter

sub = rospy.Subscriber("/openni2_camera/depth/image_raw_drop", sensor_msgs.msg.Image, callback)
pub = rospy.Publisher('movement', Move, queue_size=10)

if __name__ == '__main__':
    listener()
