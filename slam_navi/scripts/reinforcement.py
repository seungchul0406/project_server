#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import tf
import numpy as np
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from numpy import load, concatenate, dot, abs, tanh, exp, pi, min, arctan2, cos, sin
from nav_msgs.msg import Path


# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
scandata=[]
odomdata=[]
path=[]

lin_vel_t=0
ang_vel_t=0

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
	
	for angle in range(10):
		s.append(min(scandata[45+27*angle:45+27*(angle+1)]))
	
	s=array(s)
	l=[linvel]
	a=[angvel]
	np.asarray(a)
	np.asarray(l)
	features = concatenate((s.reshape(1,10)/10., np.asarray(a).reshape(1,1), np.asarray(l).reshape(1,1), angtogoal.reshape(1,1)/pi, disttogoal.reshape(1,1)/10.),axis=1)
	h = dot(features, w['actor/W1_a:0'])+w['actor/B1_a:0']
	h = (abs(h)+h)/2.
	h = dot(h, w['actor/W2_a:0'])+w['actor/B2_a:0']
	h = (abs(h)+h)/2.
	h = dot(h, w['actor/W3_a:0'])+w['actor/B3_a:0']
	h = (abs(h)+h)/2.
	global lin_vel
	global ang_vel
	lin_vel = 0.3*sig(dot(h,w['actor/WL_a:0'])+w['actor/BL_a:0'])
	ang_vel = 1.5*tanh(dot(h,w['actor/WA_a:0'])+w['actor/BA_a:0'])
	return lin_vel, ang_vel

def mapCallBack(data):
    global mapData
    mapData=data

def scanCallBack(data):
	global scandata   
	scandata=list(data.ranges)
	i=0
	while i < len(scandata):
		if scandata[i] == inf :
			scandata[i] = 10.0
		i=i+1

def pathCallback(data):
	global path
	path=[]
	for pose in data.poses:
		path.append(list([pose.pose.position.x,pose.pose.position.y]))

def lin_velocitycallback(data):
	global lin_vel_t
	lin_vel_t=data.linear.x

def ang_velocitycallback(data):
	global ang_vel_t
	ang_vel_t=data.angular.z
	
def getPosition():
	cond=0
	while cond==0:	
		try:
			(trans,rot) = listener.lookupTransform(global_frame, '/base_link', rospy.Time(0))
			cond=1
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			cond==0
	position=array([trans[0],trans[1]])
	return position

def getOrientation():
	cond=0
	while cond==0:	
		try:
			(trans,rot) = listener.lookupTransform(global_frame, '/base_link', rospy.Time(0))
			cond=1
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			cond==0
	angle = tf.transformations.euler_from_quaternion(rot)
	orientation=array([angle[2]])
	return orientation

# Node----------------------------------------------

def node():
	global w,mapData,scandata,listener,global_frame,lin_vel_t,ang_vel_t
	rospy.init_node('reinforcement', anonymous=False)
	
	# load weights
	w = load('/home/seungchul/catkin_ws/src/turtlebot3/turtlebot3_slam/scripts/reinforcement_w.npy').item()
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	rateHZ = rospy.get_param('~rate',100)
	scan_topic = rospy.get_param('~scan_topic','/scan')
	path_topic = rospy.get_param('~path_topic','/move_base/DWAPlannerROS/global_plan')
	velocity_topic = rospy.get_param('~velocity_topic','/mobile_base/commands/velocity')
	rate = rospy.Rate(rateHZ)
#-------------------------------------------------------------------------
	global_frame=rospy.get_param('~global_frame','/map')
	listener=tf.TransformListener()
	listener.waitForTransform(global_frame, '/base_link', rospy.Time(0),rospy.Duration(10.0))
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(scan_topic, LaserScan, scanCallBack)
	rospy.Subscriber(path_topic, Path, pathCallback)
	rospy.Subscriber(velocity_topic, Twist, lin_velocitycallback)
	rospy.Subscriber(velocity_topic, Twist, ang_velocitycallback)
#-------------------------------------------------------------------------
	pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
	pub2 = rospy.Publisher('Path_Data', Marker, queue_size=10)
#-------------------------------------------------------------------------
	p=Point()
	q=Twist()
	velocity=Twist()

	points_Path=Marker()
	points_Path.header.frame_id= mapData.header.frame_id
	points_Path.header.stamp= rospy.Time.now()
	points_Path.ns= "markers"
	points_Path.id = 1
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

#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		# if i==0:
		# 	lin_vel_t=0.001
		# 	ang_vel_t=0.001
		
		if (len(path)>61):
			# i = i + 1
			position=getPosition()
			orientation=getOrientation()
			print("Orientation :"),
			print(orientation)
			print("Position :"),
			print(position)
			print("Local Goal :"),
			print(path[60])
			p.x=path[60][0]
			p.y=path[60][1]
			pp=[]
			pp.append(copy(p))
			points_Path.points=pp
			pub2.publish(points_Path)
			ILpanner(position, orientation, scandata, path[60], lin_vel_t, ang_vel_t)
			print("Linear vel :"),
			print(lin_vel)
			print("Angular vel :"),
			print(ang_vel)
			q.linear.x=lin_vel
			q.linear.y=0
			q.linear.z=0
			q.angular.x=0
			q.angular.y=0
			q.angular.z=ang_vel
			pub.publish(q)
			print("==================================================")
		elif (len(path)>11):
			position=getPosition()
			orientation=getOrientation()
			print("Orientation :"),
			print(orientation)
			print("Position :"),
			print(position)
			print("Local Goal :"),
			print(path[10])
			p.x=path[10][0]
			p.y=path[10][1]
			pp=[]
			pp.append(copy(p))
			points_Path.points=pp
			pub2.publish(points_Path)
			ILpanner(position, orientation, scandata, path[10], lin_vel_t, ang_vel_t)
			print("Linear vel :"),
			print(lin_vel)
			print("Angular vel :"),
			print(ang_vel)
			q.linear.x=lin_vel
			q.linear.y=0
			q.linear.z=0
			q.angular.x=0
			q.angular.y=0
			q.angular.z=ang_vel
			pub.publish(q)
			print("==================================================")
		else :
			q.linear.x=0
			q.linear.y=0
			q.linear.z=0
			q.angular.x=0
			q.angular.y=0
			q.angular.z=0
			pub.publish(q)
			print("Robot reached target")
			print("==================================================")

#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass