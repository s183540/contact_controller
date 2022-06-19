# The posix_sitl.launch should be changed so the drone start in (0,0,0) and with 0 yaw

#!/usr/bin/env python

import rospy


from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3, TwistStamped, Twist, Pose
from std_msgs.msg import Header
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, euler_matrix, euler_from_matrix
import math
import numpy as np
from std_msgs.msg import UInt8



class fcuModes:
	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException as e:
			print("Service arming call failed: %s"%e)

	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException as e:
			   print("service set_mode call failed: %s. Autoland Mode could not be set."%e)

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. Offboard Mode could not be set."%e)



class Controller:
	# initialization method
	def __init__(self):
		self.k=0 # flag for time

		# Setpoint publisher
		self.sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

		self.att_setpoint_pub = rospy.Publisher(
			'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

		self.dronestate_pub = rospy.Publisher(
			'dronestate', UInt8, queue_size=1)

		# Drone state
		self.state = State()
		# Instantiate a setpoints message
		self.sp = PositionTarget()
		# set the flag to use position setpoints and yaw angle
		self.sp.type_mask  = 3064
		# LOCAL_NED
		self.sp.coordinate_frame = 1

		# Altitude setpoint
		self.ALT_SP = 0.5
		# update the setpoint message with the required altitude
		self.sp.position.z = self.ALT_SP
		# initial values for setpoints
		self.sp.position.x = 0.0
		self.sp.position.y = 0.0
		self.sp.yaw=0.0


		# A Message for the current local position of the drone
		self.local_pos = Point(0.0, 0.0, 0.0)

		#Init orientation and velocity
		self.local_ori = Quaternion(0.0, 0.0, 0.0, 0.0)
		self.local_vel_lin = Vector3(0.0, 0.0, 0.0)
		self.local_vel_ang = Vector3(0.0, 0.0, 0.0)

		# Attitude setpoint initialization
		self.att = AttitudeTarget()
		self.att.header = Header()
		self.att.type_mask = 0  # ignore body rate
		self.att.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
		self.att.thrust = 0.0
		self.att.body_rate = Vector3() # (0.0,0.0,0.0)
		self.att.header.frame_id = "base_footprint"

		# Current thrust of drone
		self.target_thrust=0


	# Callbacks

	# local position callback
	def posCb(self, msg):
		self.local_pos = msg.pose.position # x y z Update position
		self.local_ori = msg.pose.orientation # x y z w


	# Drone State callback
	def stateCb(self, msg):
		self.state = msg #Update state

	# Drone velocity callback
	def velCb(self, msg):
		self.local_vel_lin = msg.twist.linear #x y z
		self.local_vel_ang = msg.twist.angular #x y z

	# Drone thrust callback
	def thrustCb(self,msg):
		self.target_thrust=msg.thrust



	def pos_reached(self):
		if ((abs(self.sp.position.x-(self.local_pos.x))<0.05)) and ((abs(self.sp.position.y-(self.local_pos.y))) < 0.05) and ((abs(self.sp.position.z-(self.local_pos.z))) < 0.05):
			if (self.k==0):
				self.told = rospy.Time.now()
				self.k=1
		else:
			self.k=0
		if (self.k==1) and (rospy.Time.now()-self.told)>rospy.Duration(secs=3):
			return True
		return False

	def count_time(self):
		if (self.k==0):
			self.told = rospy.Time.now()
			self.k=1
		if  (rospy.Time.now()-self.told)>rospy.Duration(secs=3):
			#self.k=0
			return True
		#rospy.loginfo("now-told  | {0}".format(rospy.Time.now()-self.told))
		return False



# Main function
def main():

	# initiate node
	rospy.init_node('setpoint_node', anonymous=True)

	# flight mode object
	modes = fcuModes()

	# controller object
	cnt = Controller()

	# ROS loop rate
	rate = rospy.Rate(100.0)

	# Subscribe to drone state
	rospy.Subscriber('mavros/state', State, cnt.stateCb)

	# Subscribe to drone's local position
	rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

	# Subscribe to drone's local velocity
	rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, cnt.velCb)

	# Subscribe to drone's Thrust
	rospy.Subscriber('mavros/setpoint_raw/target_attitude', AttitudeTarget, cnt.thrustCb)

	# Make sure the drone is armed
	# TURN OFF WHEN TESTING ON REAL DRONE
	while not cnt.state.armed:
		modes.setArm()
		rate.sleep()


	# Flags
	state=0
	set_setp=0
	state_string="NOT_STARTET"
	print_state=0
	modes.setOffboardMode()


	# ROS main loop
	while not rospy.is_shutdown():
		if (state==0):
			state_string="TAKEOFF"
			if print_state==0:
				rospy.loginfo(state_string)
				print_state=1
			cnt.dronestate_pub.publish(state)
			cnt.sp_pub.publish(cnt.sp)
			if cnt.pos_reached():
				SETP_THRUST = cnt.target_thrust
				rospy.loginfo("SETP_THRUST  | {0}".format(SETP_THRUST))
				set_setp=0
				print_state=0
				cnt.k=0
				state+=1
		if (state==1):
			state_string="ATTITUDE FORWARD"
			if print_state==0:
				rospy.loginfo(state_string)
				print_state=1
			if set_setp==0:
				cnt.att.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0349,
                                                                 0.0)) # approx 2 deg pitch
				cnt.att.thrust = SETP_THRUST # approx 0.71 in Gazebo
				set_setp=1
			cnt.dronestate_pub.publish(state)
			cnt.att_setpoint_pub.publish(cnt.att)
			if cnt.count_time():
				set_setp=0
				print_state=0
				cnt.k=0
				state+=1
		if (state==2):
			state_string="ATTITUDE BACKWARD"
			if print_state==0:
				rospy.loginfo(state_string)
				print_state=1
			if set_setp==0:
				cnt.att.orientation = Quaternion(*quaternion_from_euler(0.0, -0.0349,
                                                                 0.0)) # approx -2 deg pitch
				cnt.att.thrust = SETP_THRUST
				set_setp=1
			cnt.dronestate_pub.publish(state)
			cnt.att_setpoint_pub.publish(cnt.att)
			if cnt.count_time():
				set_setp=0
				print_state=0
				cnt.k=0
				state+=1
		if (state==3):
			state_string="POS BEFORE LAND"
			if print_state==0:
				rospy.loginfo(state_string)
				print_state=1
			cnt.dronestate_pub.publish(state)
			cnt.sp_pub.publish(cnt.sp)
			if cnt.pos_reached():
				set_setp=0
				print_state=0
				cnt.k=0
				state+=1
		if (state==4):
			state_string="LAND"
			if print_state==0:
				rospy.loginfo(state_string)
				print_state=1
			cnt.dronestate_pub.publish(state)
			modes.setAutoLandMode()
		rate.sleep()




if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
