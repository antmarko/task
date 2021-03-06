#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class task1_5:
    def __init__(self):
        #measurments
        self.scan=[]
        # Create subscriber
        rospy.Subscriber("/robot0/laser_0",
                         LaserScan,
                         self.callback)
        # Create publisher
        self.pub = rospy.Publisher("/robot0/cmd_vel",
                                       Twist,
                                       queue_size = 10)	




    def callback(self,data):	
      (self.trans,self.rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
      print 'Transform is:'+str(self.trans)
      #Save laser values from the subscribed topic
      self.scan=data.ranges
      self.scan=[x*100 for x in self.scan]
      # Get the laser scan
      scan = self.scan
      print 'Number of beams :'+str(len(scan))  
      linear  = 0
      angular = 0
      
      s=0
      count=0
      rear=0
      
      rightspeed=0
      leftspeed=0
      #We divide laser beams in 3 groups(1 for left speed,1 for right,1 for front).Every beam is weighted using  gaussian functions which are 
      #centered exactly in front,right and left of the robot.Maximum range is decreased in 3m(we don't want to avoid far obstacles)      
      
      for i in range (0, 220): 
	
        if scan[i]<200:
	   
           s=s+(1/scan[i]**2)*(1/math.sqrt(0.2)*math.sqrt(2*math.pi))*math.exp(-(((i-121)**2)/(2*0.2)))
           count=count+1 
      if count>0:  
        rightspeed=(s/count)
      else:
        rightspeed=0
      

      s=0
      count=0
      
      k=0 
      s2=0
      for i in range (221, 445): 
        if scan[i]<200:
           s=s+(1/scan[i]**2)*(1/math.sqrt(0.2)*math.sqrt(2*math.pi))*math.exp(-(((i-333)**2)/(2*0.2)))
           count=count+1 
           
      if count>0:  
        linear=(-s/count)
      else:
        linear=0
      
      

      
      s=0
      count=0

      
      for i in range (446, 666): 
        if scan[i]<200:
           s=s+(1/scan[i]**2)*(1/math.sqrt(0.2)*math.sqrt(2*math.pi))*math.exp(-(((i-556)**2)/(0.2*2)))
           count=count+1 
      
      if count>0:  
        leftspeed=(-s/count)
      else:
        leftspeed=0
     
     
      
      angular=(rightspeed+leftspeed)
      if abs(angular*25000)<0.0001:
         angular=0
		
      #Create the message to be published
      msg=Twist()
      msg.linear.x=50000*linear+0.2
      msg.angular.z=50000*angular
      print 'Angular is :'+str(msg.angular.z)
      print 'Linear is :'+str(msg.linear.x)
      #Publish velocities to cmd_vel topic
      self.pub.publish(msg)	
	


	

if __name__ == '__main__':
	# Init ROS node
	rospy.init_node('task1_5')
	# Instatiated the class
	NC = task1_5()    

	#Create transform listener
	listener = tf.TransformListener()
	
        try:

		#(trans,rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
		#print 'Trans is :'+str(trans[0])	
		# Wait
		rospy.spin()

        except (rospy.ROSInterruptException,tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	        pass
