#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy
from random import randint
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion



class Example:

    def __init__(self):
	
	self.origin = {}
	self.origin['x'] = -50
	self.origin['y'] = -50
	self.resolution = 0.05
	self.ogm = []
	self.ogm_info = 0
	self.width=0
	self.height=0
	self.x=0
	self.y=0
	

	# What to do if shut down (e.g. Ctrl-C or failure)
	#rospy.on_shutdown(self.shutdown)
	# Tell the action client that we want to spin a thread by default
        #Read occupancy grid map
        rospy.Subscriber('/move_base_node/global_costmap/costmap',OccupancyGrid, self.readMap)

    def readMap(self, data):
	print 'Width is :'+str(data.info.width)
	print 'Height is :'+str(data.info.height)	
	self.width=data.info.width
	self.height=data.info.height
	print str(set(data.data))
	print str(data.data.count(0))
	print 'Origin :'+str(self.origin['x'])
	self.ogm_info = data.info
	self.ogm = numpy.zeros((data.info.width, data.info.height), dtype = numpy.int)
	#print 'Zero value at :'+str(data.data.index(0))
	for x in range(0, data.info.width):
	   for y in range(0, data.info.height):
	   	self.ogm[x][y] = data.data[x + data.info.width * y]
		#if self.ogm[x][y]==0:
			#print 'Element :'+str(x)+','+str(y)


	#print 'Ogm of 0.5-0.5 :'+str(self.ogm[(10.5-self.origin['x'])/self.resolution][(-23.8-self.origin['y'])/self.resolution])
	self.resolution = data.info.resolution
	self.origin['x'] = data.info.origin.position.x
	self.origin['y'] = data.info.origin.position.y
        print 'Resolution'+str(self.resolution)	
	
    
	

if __name__ == '__main__':
    try:
        # Init ROS node
        rospy.init_node('service_example')

        # Instatiated the class
        NC = Example()

        # Wait
        rospy.spin()

		
    except rospy.ROSInterruptException:
        pass
