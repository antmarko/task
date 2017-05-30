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
	self.origin['x'] = 0
	self.origin['y'] = 0
	self.resolution = 0.2
	self.ogm = []
	self.ogm_info = 0
	self.width=0
	self.height=0
	self.x=0
	self.y=0
	

	# What to do if shut down (e.g. Ctrl-C or failure)
	#rospy.on_shutdown(self.shutdown)
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")
	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))


    
    def select_target(self):

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'base_link'
	goal.target_pose.header.stamp = rospy.Time.now()
	#Transform ogm point to map coordinates (coordinate*resolution +map_origin)
	x=0.1
	y=0.0
	goal.target_pose.pose = Pose(Point(x,y, 0.000),Quaternion(0, 0, 0, 1))                          
	# Start moving
	self.move_base.send_goal(goal)
	print "Waiting"
	# Allow TurtleBot to complete task
	self.move_base.wait_for_result() 

	





	

if __name__ == '__main__':
    try:
        # Init ROS node
        rospy.init_node('service_example')

        # Instatiated the class
        NC = Example()

        # Wait
        #rospy.spin()
	

        NC.select_target()

		
    except rospy.ROSInterruptException:
        pass
