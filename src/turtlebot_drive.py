#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('autonomous_explore_map_plan')
import rospy
import tf
import math
import time
#import the library to compute transformations
from tf.transformations import euler_from_quaternion
from probabilistic_lib.functions import angle_wrap #Normalize angles between -pi and pi


#ROS messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from autonomous_explore_map_plan.srv import GotoWaypoint, GotoWaypointResponse, FindPathToGoal, FindPathToGoalResponse, FindPathToGoalRequest

#Numpy
import numpy as np

class Controller(object):

	def __init__(self):
		#rospy.init_node('turtlebot_drive', log_level=rospy.INFO)
		#rospy.loginfo("%s: starting turtlebot controller", rospy.get_name())
		#self.odometry_sub_ = rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size = 1)
		self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelcallback)
		#self.map_sub_ = rospy.Subscriber("/projected_map", OccupancyGrid, self.occupCallback, queue_size = 1)
		self.control_input_pub_ = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 1)
		
		self.serv_ = rospy.Service('/turtlebot_drive/goto', 
                                  GotoWaypoint, 
                                  self.calculateControlInput)
		
		self.current_position_ = np.zeros(2)
		self.current_orientation_ = 0.0
		
		self.desired_position_ = np.zeros(2)
		self.desired_orientation_ = 0.0
		
		# rotate once at the beginning before exploring
		rospy.sleep(1)
		self.rotateOnce()
		
		#print ('before service wait')
		rospy.wait_for_service('/turtlebot_drive/find_path_to_goal')
		try:
			self.find_path_to_goal_serv_ = rospy.ServiceProxy('/turtlebot_drive/find_path_to_goal', FindPathToGoal)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		return
		#print ('after service wait')

	'''
	def service(self):
		print ('in service function')
		self.serv_ = rospy.Service('/turtlebot_drive/goto', 
                                  GotoWaypoint, 
                                  self.calculateControlInput)
		rospy.wait_for_service('/turtlebot_drive/find_path_to_goal')
		try:
			self.find_path_to_goal_serv_ = rospy.ServiceProxy('/turtlebot_drive/find_path_to_goal', FindPathToGoal)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	'''

       
	def calculateControlInput(self, req):
		# define a request for FindPathToGoal service
		planner_request = FindPathToGoalRequest()
		planner_request.goal_state_x = req.goal_state_x
		planner_request.goal_state_y = req.goal_state_y
		
		planner_response = self.find_path_to_goal_serv_(planner_request)
		
		# Driver for the robot 
		for pose in planner_response.poses:
			#print pose
			control_input = Twist()
			self.desired_position_[0] = pose.x
			self.desired_position_[1] = pose.y
	 		
			loop_rate = rospy.Rate(100) # 10Hz
			orientation_approach = False
			while not rospy.is_shutdown():
				inc_x = self.desired_position_[0] - self.current_position_[0]
				inc_y = self.desired_position_[1] - self.current_position_[1]
	 			
				self.desired_orientation_ = wrapAngle(math.atan2(inc_y, inc_x))
				yaw_error = wrapAngle(self.desired_orientation_ - self.current_orientation_)
				distance_to_goal = math.sqrt(math.pow(inc_x, 2.0) + math.pow(inc_y, 2.0))
	 			
				if abs(yaw_error) > 0.04 and not orientation_approach:
					control_input.angular.x = 0.0
					control_input.angular.y = 0.0
					control_input.angular.z = yaw_error * 1.0
				
					control_input.linear.x = 0.08
					control_input.linear.y = 0.0
					control_input.linear.z = 0.0
				else:
					control_input.angular.x = 0.0
					control_input.angular.y = 0.0
					control_input.angular.z = 0.0
					
					orientation_approach = True
					liner_speed = abs(distance_to_goal) * 0.5
					
					if liner_speed < 0.1:
						control_input.linear.x = 0.1
					elif liner_speed > 0.2:
						control_input.linear.x = 0.2
					else:
						control_input.linear.x = liner_speed
					
					control_input.linear.y = 0.0
					control_input.linear.z = 0.0
	 				
				self.control_input_pub_.publish(control_input)
	 			
				rospy.logdebug("%s: current position: [%f, %f]\n", rospy.get_name(), self.current_position_[0], self.current_position_[1])
				rospy.logdebug("%s: desired position: [%f, %f]\n", rospy.get_name(), self.desired_position_[0], self.desired_position_[1])
				rospy.logdebug("%s: yaw_error: %f\n", rospy.get_name(), yaw_error)
				rospy.logdebug("%s: current orientation: %f\n", rospy.get_name(), self.current_orientation_)
				rospy.logdebug("%s: desired orientation: %f\n", rospy.get_name(), self.desired_orientation_)
				rospy.logdebug("%s: distance_to_goal: %f\n", rospy.get_name(), distance_to_goal)
				rospy.logdebug("%s: control_input.linear.x %f\n", rospy.get_name(), control_input.linear.x)
				rospy.logdebug("%s: control_input.angular.z %f\n", rospy.get_name(), control_input.angular.z)
	 			
				if distance_to_goal <= 0.4:
					break
				loop_rate.sleep()
		
		return GotoWaypointResponse()

	def odomCallback(self, odometry_msg):
		self.current_position_[0] = odometry_msg.pose.pose.position.x
		self.current_position_[1] = odometry_msg.pose.pose.position.y
		(r, p, y) = tf.transformations.euler_from_quaternion([odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w])
		self.current_orientation_ = wrapAngle(y)
		return


	def modelcallback(self, msg):
		#a = model_msg.name[6]
		#mobile_base_idx = 6
		#print (str(model_msg.pose[mobile_base_idx].orientation.x))
		#print (str(len(model_msg.name)))
		mobile_base_idx = -1
		for i in range(len(msg.name)):
		    #print ('in loop')
        	    if msg.name[i] == 'mobile_base':
        	        mobile_base_idx = i
        		#print (str(mobile_base_idx))
 			break
		#print (str(mobile_base_idx))
       
      
        	self.current_position_[0] = msg.pose[mobile_base_idx].position.x
        	self.current_position_[1] = msg.pose[mobile_base_idx].position.y
        	(ang_x, ang_y, ang_z) = euler_from_quaternion([msg.pose[mobile_base_idx].orientation.x,
                                                    msg.pose[mobile_base_idx].orientation.y,
                                                    msg.pose[mobile_base_idx].orientation.z,
                                                    msg.pose[mobile_base_idx].orientation.w]) 
        #Update theta with euler_z
        	self.current_orientation_ = wrapAngle(ang_z)
        #return



 	def printinfo(self):
 		print ('test')

 	def rotateOnce(self):
 		print ('current orientation' + str(self.current_orientation_))
 		control_input = Twist()
 		control_input.angular.z = 1
 		#rate = rospy.Rate(10) # 10hz
 		while np.abs(self.current_orientation_) < 0.5:
 			self.control_input_pub_.publish(control_input)
 		#rospy.sleep(1.0)

 		rotate = 0
 		while True:
 			self.control_input_pub_.publish(control_input)
 			#print ('current orientation' + str(self.current_orientation_))
 			rospy.sleep(0.1)
 			if np.abs(self.current_orientation_) < 0.1:
 				rotate += 1
 				print ('rotate' + str(rotate))
 			if rotate == 1:
 				break
 		

def wrapAngle(angle):
	"""wrapAngle
	
	Calculates angles values between 0 and 2pi"""
	return (angle + ( 2.0 * math.pi * math.floor( ( math.pi - angle ) / ( 2.0 * math.pi ) ) ) )

if __name__ == '__main__':
    rospy.init_node('turtlebot_drive', log_level=rospy.INFO)
    rospy.loginfo("%s: starting turtlebot controller", rospy.get_name())

    print ('before controller')
    controller = Controller()
    print ('after controller')
    print (str(controller.current_orientation_))
    #while not rospy.is_shutdown():
    #controller.rotateOnce()
    	
    #controller.service()
    print ('exit out drive')
    rospy.spin()

