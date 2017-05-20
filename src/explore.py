#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('autonomous_explore_map_plan')
import rospy
import tf
import math

#ROS messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from autonomous_explore_map_plan.srv import GotoWaypoint, GotoWaypointResponse, FindPathToGoal, FindPathToGoalResponse, FindPathToGoalRequest

#Numpy
import numpy as np

from MapProcessing import ProcessMap
from controller_turtlebot import Controller

if __name__ == '__main__':
    rospy.init_node('explore_node', log_level=rospy.INFO)
    rospy.loginfo("%s: starting turtlebot exploration", rospy.get_name())

    controller = Controller()

    # get map from controller through service call 
    # pass the map and its resolution to ProcessProjectedMap()
    pm = ProcessMap()
    pm.ProcessProjectedMap()

    rospy.spin()
