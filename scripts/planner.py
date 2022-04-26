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
from std_msgs.msg import UInt8

sphere_x = 0
sphere_y = 0
sphere_z = 0
sphere_radius = 0 
recieved_sphere_data = False 
rob_avil = False
ball_avil = False
curr_pos = [0.0]*6


# Adds points to plan
def add_point(linearX, linearY, linearZ, angularX, angularY, angularZ, mode, plan):
	point = Twist()
	grip = UInt8()	
	
	point.linear.x = linearX
	point.linear.y = linearY
	point.linear.z = linearZ
	point.angular.x = angularX
	point.angular.y = angularY
	point.angular.z = angularZ
	grip.data = mode
		
	plan.points.append(point)
	plan.modes.append(grip)

# Gets sphere raw data
def get_sphere(data):	
	global sphere_x
	global sphere_y
	global sphere_z
	global sphere_radius
	global recieved_sphere_data
	global ball_avil
	
	if not ball_avil:
		sphere_x = data.xc
		sphere_y = data.yc
		sphere_z = data.zc
		sphere_radius = data.radius
	ball_avil = True

def get_pos(data):
	global curr_pos
	global rob_avil 
	
	if not rob_avil:
		curr_pos[0] = data.linear.x
		curr_pos[1] = data.linear.y
		curr_pos[2] = data.linear.z
		curr_pos[3] = data.angular.x
		curr_pos[4] = data.angular.y
		curr_pos[5] = data.angular.z
	rob_avil = True 
	
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
	rqt_toggle = rospy.Subscriber('/rqt_toggle', Bool, rqt_listener)
	# Subscriber to pause plan 
	pause_toggle = rospy.Subscriber('/pause_toggle', Bool, pause_listener)	
	rospy.Subscriber('/ur5e/toolpose', Twist, get_pos)
	# Set a 10Hz frequency
	loop_rate = rospy.Rate(10)
	planned = False
	plan = Plan()
	while not rospy.is_shutdown():
		# add a ros transform listener
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		if not planned and ball_avil and rob_avil:
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
				
			# Define plan
			
			open1 = 1
			close = 2
			stay = 0
			y_offset = -0.01
			roll, pitch, yaw = curr_pos[3], curr_pos[4], curr_pos[5]
			# Starting position 
			add_point(curr_pos[0],curr_pos[1], curr_pos[2], roll, pitch, yaw, stay, plan)
			add_point(x, y+y_offset, z+.1, roll, pitch, yaw, stay, plan)
			# Position with x, y, z + radius and stop to grip ball
			add_point(x, y+y_offset, z+.02, roll, pitch, yaw, stay, plan)
			add_point(x, y+y_offset, z+.02, roll, pitch, yaw, close, plan)
			add_point(x, y+y_offset, z+.1, roll, pitch, yaw, stay, plan)
			# Turn right 
			add_point(0.3, -0.408, 0.274, roll, pitch, yaw, stay, plan)
			add_point(0.3, -0.408, z+.02, roll, pitch, yaw, stay, plan)
			# Decrease z to drop ball 
			add_point(0.3, -0.408, z+.02, roll, pitch, yaw, open1, plan)
			# open1 gripper
			#add_point(0.3, -0.408, z+.02, roll, pitch, yaw, open1, plan)
			# Back to Start
			add_point(-0.014, -0.408, 0.274, roll, pitch, yaw, stay, plan)
			planned = True
		
		plan_pub.publish(plan)
					
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
				
	
	
	
	
	

	
	
	
	

