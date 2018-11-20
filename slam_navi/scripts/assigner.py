#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf
from turtlebot3_slam.msg import PointArray
from time import time
import numpy as np
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from numpy import load, concatenate, dot, abs, tanh, exp, pi, min, arctan2, cos, sin
from functions import robot,informationGain,discount
from numpy.linalg import norm
import os

# Subscribers' callbacks----------------------------------------
mapData=OccupancyGrid()
frontiers=[]
scandata=[]
path=[]
lin_vel_t=[]
ang_vel_t=[]

def sig(data):
	return 1./(1+exp(-data))

def ILpanner(pos, ori, scandata, goal, linvel, angvel):
	angtogoal = arctan2(goal[1]-pos[1],goal[0]-pos[0])-ori
	if angtogoal>pi:
		angtogoal -=2*pi
	elif angtogoal<-pi:
		angtogoal +=2*pi
	disttogoal = ((goal[0]-pos[0])**2+(goal[1]-pos[1])**2)**0.5
	print("Distance to Local Goal :"),
	print(disttogoal)
	if disttogoal> 5:
		disttogoal = array([5.0])
	s = []
	
	for angle in range(5):
		s.append(min(scandata[225+27*angle:225+27*(angle+1)]))
	for angle in range(5):
		s.append(min(scandata[27*angle:27*(angle+1)]))	
	s=2*array(s)

	l=[linvel]
	a=[angvel]
	np.asarray(a)
	np.asarray(l)
	features = concatenate((s.reshape(1,10)/10, np.asarray(a).reshape(1,1), np.asarray(l).reshape(1,1), angtogoal.reshape(1,1)/pi, disttogoal.reshape(1,1)/10),axis=1)
	h = dot(features, w['actor/W1_a:0'])+w['actor/B1_a:0']
	h = (abs(h)+h)/2.
	h = dot(h, w['actor/W2_a:0'])+w['actor/B2_a:0']
	h = (abs(h)+h)/2.
	h = dot(h, w['actor/W3_a:0'])+w['actor/B3_a:0']
	h = (abs(h)+h)/2.
	global lin_vel
	global ang_vel
	lin_vel = 0.15*sig(dot(h,w['actor/WL_a:0'])+w['actor/BL_a:0'])
	ang_vel = 0.75*tanh(dot(h,w['actor/WA_a:0'])+w['actor/BA_a:0'])
	return lin_vel, ang_vel

def CallBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data

def scanCallBack(data):
	global scandata   
	scandata=list(data.ranges)
	i=0
	while i < len(scandata):
		if scandata[i] == 0 :
			scandata[i] = 10.0
		i=i+1

def pathCallback(data):
	global path
	path=[]
	for pose in data.poses:
		path.append(list([pose.pose.position.x,pose.pose.position.y]))

def lin_velocityCallback(data):
	global lin_vel_t
	lin_vel_t=data.linear.x

def ang_velocityCallback(data):
	global ang_vel_t
	ang_vel_t=data.angular.z
	
# Node------------------------------------------------------------------

def node():
	i=0
	global w,mapData,scandata,lin_vel_t,ang_vel_t
	rospy.init_node('assigner', anonymous=False)

	# load weights
	w = load('/home/seungchul/catkin_ws/src/turtlebot3/turtlebot3_slam/scripts/reinforcement_w.npy').item()
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',2.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	delay_after_assignement=rospy.get_param('~delay_after_assignement',4.0)
	rateHz = rospy.get_param('~rate',10)
	velocity_topic = rospy.get_param('~velocity_topic','/cmd_vel')
	scan_topic = rospy.get_param('~scan_topic','/scan')
	path_topic = rospy.get_param('~path_topic','/move_base/DWAPlannerROS/global_plan')
	rate = rospy.Rate(rateHz)
#-------------------------------------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, CallBack)
	rospy.Subscriber(scan_topic, LaserScan, scanCallBack)
	rospy.Subscriber(path_topic, Path, pathCallback)
	rospy.Subscriber(velocity_topic, Twist, lin_velocityCallback)
	rospy.Subscriber(velocity_topic, Twist, ang_velocityCallback)
#-------------------------------------------------------------------------
	pub = rospy.Publisher('RRT_goal', Marker, queue_size=10) 
	pub2 = rospy.Publisher('Path_Data', Marker, queue_size=10) 
	pub3 = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
#-------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)
	
	p=Point()
	q=Point()
	r=Twist()

	points_RRT=Marker()
	points_RRT.header.frame_id= mapData.header.frame_id
	points_RRT.header.stamp= rospy.Time.now()
	points_RRT.ns= "markers1"
	points_RRT.id = 1
	points_RRT.type = Marker.POINTS
	points_RRT.action = Marker.ADD
	points_RRT.pose.orientation.w = 1.0
	points_RRT.scale.x=0.1
	points_RRT.scale.y=0.1 
	points_RRT.color.r = 0.0/255.0
	points_RRT.color.g = 255.0/255.0
	points_RRT.color.b = 255.0/255.0
	points_RRT.color.a=1
	points_RRT.lifetime = rospy.Duration()

	points_NN=Marker()
	points_NN.header.frame_id= mapData.header.frame_id
	points_NN.header.stamp= rospy.Time.now()
	points_NN.ns= "markers2"
	points_NN.id = 2
	points_NN.type = Marker.POINTS
	points_NN.action = Marker.ADD
	points_NN.pose.orientation.w = 1.0
	points_NN.scale.x=0.05
	points_NN.scale.y=0.05
	points_NN.color.r = 0.0/255.0
	points_NN.color.g = 255.0/255.0
	points_NN.color.b = 255.0/255.0
	points_NN.color.a=1
	points_NN.lifetime = rospy.Duration()

	points_Path=Marker()
	points_Path.header.frame_id= mapData.header.frame_id
	points_Path.header.stamp= rospy.Time.now()
	points_Path.ns= "markers3"
	points_Path.id = 3
	points_Path.type = Marker.POINTS
	points_Path.action = Marker.ADD
	points_Path.pose.orientation.w = 1.0
	points_Path.scale.x=0.05
	points_Path.scale.y=0.05
	points_Path.color.r = 0.0/255.0
	points_Path.color.g = 255.0/255.0
	points_Path.color.b = 0.0/255.0
	points_Path.color.a=1
	points_Path.lifetime = rospy.Duration()

#wait if map is not received yet
	while (len(mapData.data)<1):
		pass
	robots=[]
	robots=robot()
	robots.sendGoal(robots.getPosition())			
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		centroids=copy(frontiers)
		if i==0:
			lin_vel_t=[[0.0]]
			ang_vel_t=[[0.0]]
		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))			
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		ir=[]
		
		for ip in range(0,len(centroids)):
			cost=norm(robots.getPosition()-centroids[ip])		
			information_gain=infoGain[ip]
			if (norm(robots.getPosition()-centroids[ip])<=hysteresis_radius):

				information_gain*=hysteresis_gain
			revenue=information_gain*info_multiplier-cost
			revenue_record.append(revenue)
			centroid_record.append(centroids[ip])
			id_record.append(ir)
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			i = i + 1
			winner_id=revenue_record.index(max(revenue_record))
			robots.sendGoal(centroid_record[winner_id])
			p.x=centroid_record[winner_id][0]
			p.y=centroid_record[winner_id][1]
			pp=[]
			pp.append(copy(p))
			points_RRT.points=pp
			pub.publish(points_RRT)			
			position=robots.getPosition()
			orientation=robots.getOrientation()
			print("***Odom***")
			print("Position :"),
			print(position)
			print("Orientation :"),
			print(orientation)
			print("\n"),
			print("***Path***")
			print("Global Goal :"),
			print(centroid_record[winner_id])
			print("Path length :"),
			print(len(path))
			print("\n"),
			print("***Main***")
			print("Input Linear vel :"),
			print(lin_vel_t)
			print("Input Angular vel :"),
			print(ang_vel_t)
			
			if len(path)>61: 
				print("Local Goal(1) :"),
				print(path[60])
				q.x=path[60][0]
				q.y=path[60][1]
				qq=[]
				qq.append(copy(q))
				points_Path.points=qq
				pub2.publish(points_Path)
				ILpanner(position, orientation, scandata, path[60], lin_vel_t, ang_vel_t)
				print("Output Input Linear vel :"),
				print(lin_vel)
				print("Output Angular vel :"),
				print(ang_vel)
				r.linear.x=lin_vel
				r.linear.y=0
				r.linear.z=0
				r.angular.x=0
				r.angular.y=0
				r.angular.z=ang_vel
				pub3.publish(r)

			elif len(path)>21: 
				print("Local Goal(2) :"),
				print(path[20])
				q.x=path[20][0]
				q.y=path[20][1]
				qq=[]
				qq.append(copy(q))
				points_Path.points=qq
				pub2.publish(points_Path)
				ILpanner(position, orientation, scandata, path[20], lin_vel_t, ang_vel_t)
				print("Output Input Linear vel :"),
				print(lin_vel)
				print("Output Angular vel :"),
				print(ang_vel)
				r.linear.x=lin_vel
				r.linear.y=0
				r.linear.z=0
				r.angular.x=0
				r.angular.y=0
				r.angular.z=ang_vel
				pub3.publish(r)

			else : 
				r.linear.x=0
				r.linear.y=0
				r.linear.z=0
				r.angular.x=0
				r.angular.y=0
				r.angular.z=0
				pub3.publish(r)
				print("\n")
				print("Robot Stanby")
	
			print("==========================================================================================================================")
			robots.sendGoal(centroid_record[winner_id])
			rospy.sleep(delay_after_assignement)

#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
