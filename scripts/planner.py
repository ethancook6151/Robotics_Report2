#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from robot_vision_lectures.msg import SphereParams 
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  

sphere_x = 0
sphere_y = 0
sphere_z = 0
sphere_radius = 0 
recieved_sphere_data = False 
rqt_toggle = False
pause_toggle = False

# Adds points to plan
def add_point(linearX, linearY, linearZ, angularX, angularY, angularZ, plan):
	point = Twist()
		
	point.linear.x = linearX
	point.linear.y = linearY
	point.linear.z = linearZ
	point.angular.x = angularX
	point.angular.y = angularY
	point.angular.z = angularZ
		
	plan.points.append(point)

# Gets sphere raw data
def get_sphere(data):	
	global sphere_x
	global sphere_y
	global sphere_z
	global sphere_radius
	global recieved_sphere_data
	
	sphere_x = data.xc
	sphere_y = data.yc
	sphere_z = data.zc
	sphere_radius = data.radius
	recieved_sphere_data = True 
	
def rqt_listener(data):
	global rqt_toggle
	rqt_toggle = data.data
	

def pause_listener(data):
	global pause_toggle
	pause_toggle = data.data
	
	
if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('planner', anonymous = True)
	# Subscriber for sphere parameters
	rospy.Subscriber('sphere_params', SphereParams, get_sphere)
	# Publisher for sending joint positions
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# Subscribers to cancel plan 
	rqt_toggle = rospy.Subscriber("/rqt_toggle", Bool, rqt_listener)
	# Subscriber to pause plan 
	pause_toggle = rospy.Subscriber("/pause_toggle", Bool, pause_listener)
	# Set a 10Hz frequency
	loop_rate = rospy.Rate(10)
	planned = False
	
	while not rospy.is_shutdown():
		# add a ros transform listener
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		if recieved_sphere_data: 
			# try getting the most update transformation between the camera frame and the base frame
			if not planned:
				try:
					trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
					print('Frames not available')
					loop_rate.sleep()
					continue
				# Define points in camera frame
				pt_in_cam = tf2_geometry_msgs.PointStamped()
				pt_in_cam.header.frame_id = 'camera_color_optical_frame'
				pt_in_cam.header.stamp = rospy.get_rostime()
	
				pt_in_cam.point.x = sphere_x
				pt_in_cam.point.y = sphere_y
				pt_in_cam.point.z = sphere_z
	
				# Convert points to base frame
				pt_in_base = tfBuffer.transform(pt_in_cam,'base', rospy.Duration(1.0))
				x,y,z,radius = pt_in_base.point.x, pt_in_base.point.y, pt_in_base.point.z, sphere_radius
	
				# Print coor before and after transform 
				print("Before tranformed: \n", "x: ", sphere_x, "y: ", sphere_y, "z: ", sphere_z, "radius: ", sphere_radius, "\n")
				print("Transformed: \n", "x: ", x, "y: ", y, "z: ", z, "radius: ", radius, "\n")
				planned = True

			# Define plan
			plan = Plan()
			roll, pitch, yaw = 3.126, 0.016, 1.530
			# Starting position 
			add_point(-0.014, -0.408, 0.274, roll, pitch, yaw, plan)
			# Position with x, y, z + radius
			add_point(x, y, z+.02, roll, pitch, yaw, plan)
			# Turn right 
			add_point(0.3, -0.408, 0.274, roll, pitch, yaw, plan)
			# Decrease z to drop ball 
			add_point(0.3, -0.408, z+.02, roll, pitch, yaw, plan)
			# Back to Start
			add_point(-0.014, -0.408, 0.274, roll, pitch, yaw, plan)
			# If not cancelled 
			# if not rqt_toggle:
				# publish the plan
			plan_pub.publish(plan)
			# If plan pause publish blank plan
			#if pause_toggle: 
				#plan_pub.publish(Plan())
			# wait for 0.1 seconds until the next loop and repeat
			loop_rate.sleep()
		






