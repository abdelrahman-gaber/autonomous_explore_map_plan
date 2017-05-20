#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import scipy.io as sio
import scipy.ndimage as ndimage 
import rospy
import time
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from autonomous_explore_map_plan.srv import GotoWaypoint, GotoWaypointRequest, GotoWaypointResponse
np.set_printoptions(threshold='nan')


class ProcessMap(object):
	def __init__(self):
		self.odometry_sub_ = rospy.Subscriber("/odom",Odometry, self.OdometryCallback)
		self.current_position = np.zeros(2)

		# get the map >> call ProcessProjectedMap() 
		self.map_sub_ = rospy.Subscriber("/projected_map", OccupancyGrid, self.OccupancyGridCallback, queue_size = 1)
		
		# Use RRT* to go to best point 
		rospy.wait_for_service('/turtlebot_drive/goto')
		try:
			self.goto_serv_ = rospy.ServiceProxy('/turtlebot_drive/goto', GotoWaypoint)
		except rospy.ServiceException, e:
			print "Goto Service call failed: %s"%e
		return


	def OccupancyGridCallback(self, msg):			
		self.dat = msg.data
		self.wid = msg.info.width
		self.heigh = msg.info.height
		self.res = msg.info.resolution
		self.xorg = msg.info.origin.position.x
		self.yorg = msg.info.origin.position.y 

	def OdometryCallback(self, msg):
		self.current_position[0] = msg.pose.pose.position.x
		self.current_position[1] = msg.pose.pose.position.y

	def brushfire(self, map1):
		#print(map1)
		
		rows = map1.shape[0]
  		cols = map1.shape[1]
  		num_zeros = (map1 == 0).sum()

  		k=100; #Obstacle value
  		while num_zeros >0 :
  			for i in range(rows):
  				for j in range(cols):
  					if map1[i][j]==k:
  						if (i-1)>=0 and (i+1)<rows and (j-1)>=0 and (j+1)<cols:
  							if map1[i-1][j-1]==0:
  							    map1[i-1][j-1]=k+1
							    num_zeros=num_zeros-1
            						if map1[i-1][j]==0:
              						    map1[i-1][j]=k+1
              					            num_zeros=num_zeros-1
            						if map1[i-1][j+1]==0:
              						    map1[i-1][j+1]=k+1
              					            num_zeros=num_zeros-1
            						if map1[i][j-1]==0:
              					            map1[i][j-1]=k+1
              						    num_zeros=num_zeros-1
            						if map1[i][j+1]==0:
              						    map1[i][j+1]=k+1
              						    num_zeros=num_zeros-1
            						if map1[i+1][j-1]==0:
              						    map1[i+1][j-1]=k+1
              					            num_zeros=num_zeros-1
            						if map1[i+1][j]==0:
              						    map1[i+1][j]=k+1
              						    num_zeros=num_zeros-1
            						if map1[i+1][j+1]==0:
              						    map1[i+1][j+1]=k+1
              						    num_zeros=num_zeros-1
          				elif i-1>=0 and j-1>=0:
          					if map1[i-1][j-1]==0:
          						map1[i-1][j-1]=k+1
              					num_zeros=num_zeros-1
          				elif i-1>=0:
          					if map1[i-1][j]==0:
          						map1[i-1][j]=k+1
              					num_zeros=num_zeros-1
          				elif i-1>=0 and j+1<cols:
          					if map1[i-1][j+1]==0:
          						map1[i-1][j+1]=k+1
              					num_zeros=num_zeros-1
          				elif j-1>=0:
          					if map1[i][j-1]==0:
          						map1[i][j-1]=k+1
              					num_zeros=num_zeros-1
          				elif j+1<cols:
          					if map1[i][j+1]==0:
          						map1[i][j+1]=k+1
              					num_zeros=num_zeros-1
          				elif i+1<rows and j-1>=0:
          					if map1[i+1][j-1]==0:
          						map1[i+1][j-1]=k+1
              					num_zeros=num_zeros-1
          				elif i+1<rows:
          					if map1[i+1][j]==0:
          						map1[i+1][j]=k+1
              					num_zeros=num_zeros-1
          				elif i+1<rows and j+1<cols:
          					if map1[i+1][j+1]==0:
          						map1[i+1][j+1]=k+1
              					num_zeros=num_zeros-1
			
			k=k+1
		best=np.amax(map1)
  		#print(best)
  		index=[]

		for k in range(4,rows-5):
			for j in range(4,cols-5):
				if map1[k][j]==best:
					inr=[k,j]
					index.append(inr)
		
		N=[]
		index=np.asarray(index)
		for i in range(index.shape[0]):
			n = 0
			for k in range(-4,4):
				for j in range(-4,4):
					if map1[index[i][0]+k][index[i][1]+j]==-1:
						n=n+1
			N.append(n)	

		max_value = max(N)
		max_index = N.index(max_value)		
		bestPoint=[index[max_index][0],index[max_index][1]]
		BestGazebo=[float(index[max_index][0]*self.res+self.xorg),float(index[max_index][1]*self.res+self.yorg)]
		
		print('Best point is ' )
		print(bestPoint)
		print('best point in Gazebo is')
		print(BestGazebo)
		bestPoint = np.array(bestPoint)
		BestGazebo = np.array(BestGazebo)

		return bestPoint, BestGazebo, map1 

	# =================================================
	def  wavefront(self, map, start, goal):
		rows = map.shape[0]
		cols = map.shape[1]
		goal_row=goal[0][0]
		#print(goal_row)
		goal_col=goal[0][1]
		#print(goal_col)
		start_row=start[0][0]
		start_col=start[0][1]
		map[goal_row][goal_col]=2
		num_zeros = (map == 0).sum()
		k=2; #goal value
		while num_zeros>0:
			for i in range(rows):
				for j in range(cols):
					if map[i][j]==k:
						if i-1>=0 and i+1<rows and j-1>=0 and j+1<cols:
							if map[i-1][j-1]==0:
								map[i-1][j-1]=k+1
								num_zeros=num_zeros-1
							if map[i-1][j]==0:
								map[i-1][j]=k+1
								num_zeros=num_zeros-1
							if map[i-1][j+1]==0:
								map[i-1][j+1]=k+1
								num_zeros=num_zeros-1
							if map[i][j-1]==0:
								map[i][j-1]=k+1
								num_zeros=num_zeros-1
							if map[i][j+1]==0:
								map[i][j+1]=k+1
								num_zeros=num_zeros-1
							if map[i+1][j-1]==0:
								map[i+1][j-1]=k+1
								num_zeros=num_zeros-1
							if map[i+1][j]==0:
								map[i+1][j]=k+1
								num_zeros=num_zeros-1
							if map[i+1][j+1]==0:
								map[i+1][j+1]=k+1
								num_zeros=num_zeros-1
						elif i-1>=0 and j-1>=0:
							if map[i-1][j-1]==0:
								map[i-1][j-1]=k+1
								num_zeros=num_zeros-1
						elif i-1>=0:
							if map[i-1][j]==0:
								map[i-1][j]=k+1
								num_zeros=num_zeros-1
						elif i-1>=0 and j+1<cols:
							if map[i-1][j+1]==0:
								map[i-1][j+1]=k+1
								num_zeros=num_zeros-1
						elif j-1>=0:
							if map[i][j-1]==0:
								map[i][j-1]=k+1
								num_zeros=num_zeros-1
						elif j+1<cols:
							if map[i][j+1]==0:
								map[i][j+1]=k+1
								num_zeros=num_zeros-1
						elif i+1<=rows and j-1>=0:
							if map[i+1][j-1]==0:
								map[i+1][j-1]=k+1
								num_zeros=num_zeros-1
						elif i+1<rows:
							if map[i+1][j]==0:
								map[i+1][j]=k+1
								num_zeros=num_zeros-1
						elif i+1<rows and j+1<cols:
							if map[i+1][j+1]==0:
								map[i+1][j+1]=k+1
								num_zeros=num_zeros-1
			k=k+1
		traject=[]
		point = [start_row,start_col]
		traject.append(point)
		cr=start_row
		cc=start_col

		while map[cr][cc] -2:
			if map[cr-1][cc]<map[cr][cc] and map[cr-1][cc]>1:
				cr=cr-1
				point=[cr,cc]
				traject.append(point)
			elif map[cr+1][cc]<map[cr][cc] and map[cr+1][cc]>1:
				cr=cr+1
				point=[cr,cc]
				traject.append(point)
			elif map[cr][cc-1]<map[cr][cc] and map[cr][cc-1]>1:
				cc=cc-1
				point=[cr,cc]
				traject.append(point)
			elif map[cr][cc+1]<map[cr][cc] and map[cr-1][cc+1]>1:
				cc=cc+1
				point=[cr,cc]
				traject.append(point)
			elif map[cr-1][cc-1]<map[cr][cc] and map[cr-1][cc-1]>1:
				cr=cr-1
				cc=cc-1
				point=[cr,cc]
				traject.append(point)
			elif map[cr-1][cc+1]<map[cr][cc] and map[cr-1][cc+1]>1:
				cr=cr-1
				cc=cc-1
				point=[cr,cc]
				traject.append(point)
			elif map[cr+1][cc-1]<map[cr][cc] and map[cr+1][cc-1]>1:
				cr=cr+1
				cc=cc-1
				point=[cr,cc]
				traject.append(point)
			elif map[cr+1][cc+1]<map[cr][cc] and map[cr+1][cc+1]>1:
				cr=cr+1
				cc=cc-1
				point=[cr,cc]
				traject.append(point)
		point=[goal_row,goal_col]
		traject.append(point)

		return map,traject


	def ProcessProjectedMap(self):
		# get map
		# reshape the map to the given dimensions 
		rospy.sleep(15)
		data1 = np.reshape(self.dat, (self.wid ,self.heigh), order="F")
		#data1 = np.reshape(data,(155,120), order="F")
		data2 = np.asarray(data1)
		print('sahpe of map is:')
		print(data2.shape)

		# Dilation 
		data3 = ndimage.grey_dilation(data2, footprint=np.ones((3,3)))
		#data3 = data2
		#print(type(data2))
		#print(data2.shape)

		# Call brushfire 
		[value_m, BestGazebo, map1] = self.brushfire(data3)
		#print(BestGazebo.shape)
		#plt.imshow(map1)
		#plt.show()
		#value_m = self.brushfire(data2)
		#value_m = [81,86]

		GoalPoint = np.array([BestGazebo])
		#print(value_m)
		#print(GoalPoint.shape)

		# Send request to /goto service 
		print('got  first point ... finding a path ..... ')
		goto_request = GotoWaypointRequest()
		goto_request.goal_state_x = round(GoalPoint[0][0])
		goto_request.goal_state_y = round(GoalPoint[0][1])
		
		goto_response = self.goto_serv_(goto_request)
		

		
		#data1 = np.reshape(data,(155,120), order="F")
		#data2 = np.asarray(data1)
		#data3 = ndimage.grey_dilation(data2, footprint=np.ones((7,7)))

		# wavefront algorithm
		#[value_map, traject] = self.wavefront(data3, np.array([self.current_position]), GoalPoint)
		#traject=np.asarray(traject)
		#print(traject)
		#rows_traj=traject.shape[0]
		#cols_traj=traject.shape[1]
		#value_map[traject[0][0]][traject[0][1]]=110

		#for i in range (1,rows_traj-2):
		#	value_map[traject[i][0]][traject[i][1]]=120

		#value_map[traject[rows_traj-1][0]][traject[rows_traj-1][1]]=130
		#plt.imshow(value_map)
		#plt.show()
		#rospy.spin()


#if __name__ == "__main__":
#	pm = ProcessMap()
#	pm.ProcessProjectedMap()
    #main()

