# The posix_sitl.launch should be changed so the drone start in (0,8,0) and with 1.57 yaw


#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3, TwistStamped, Twist, Pose
from std_msgs.msg import Header
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, euler_matrix, euler_from_matrix
import math
import numpy as np
from std_msgs.msg import UInt8


# Flight modes class
class FcuModes:
	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException as e:
			print("Service arming call failed: %s"%e)

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. Offboard Mode could not be set."%e)


	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException as e:
			   print("service set_mode call failed: %s. Autoland Mode could not be set."%e)


class Controller:
	# initialization method
	def __init__(self):

		self.k=0 #flag for time

		# Contact controller variables
		self.g = 9.80665
		self.contact_pitch = 0.34906585 #20 deg
		self.rod_length = 0.42 #from in iris.sdf
		self.position_gain = 17
		self.velocity_gain = 10
		self.mass_total = 1.5+0.015+(4*0.005)+0.016 #added together from iris.sdf
		self.contact_yaw = 1.570796326794897 #90 deg
		self.p1 = 13.090802733397977
		self.p2 = 1.20

		# Setpoint position publisher
		self.sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

		# Setpoint attitude publisher
		self.att_setpoint_pub = rospy.Publisher(
			'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

		self.dronestate_pub = rospy.Publisher(
			'dronestate', UInt8, queue_size=1)

		# Drone state
		self.state = State()
		self.sp = PositionTarget()
		self.sp.type_mask = 3064
		# LOCAL_NED - north, east, down
		self.sp.coordinate_frame = 1

		self.sp.header.frame_id = "base_link"


		# Altitude setpoint
		self.ALT_SP = 3.0
		self.sp.position.x = 0
		self.sp.position.y = 0
		self.sp.position.z = self.ALT_SP
		self.sp.yaw=1.57

		# A Message for the current local position of the drone
		self.local_pos = Point(0.0, 0.0, 0.0)

		#Init orientation and velocity
		self.local_ori = Quaternion(0.0, 0.0, 0.0, 0.0)
		self.local_vel_lin = Vector3(0.0, 0.0, 0.0)
		self.local_vel_ang = Vector3(0.0, 0.0, 0.0)



		#Attitude setup
		self.att = AttitudeTarget()
		self.att.header = Header()
		self.att.type_mask = 0  # ignore body rate
		self.att.body_rate = Vector3()
		self.att.header.frame_id = "base_footprint" #base_link


		# Current thrust of drone
		self.target_thrust=0

		# Current dronestate
		self.dronestate=0


	# Callbacks
	def posCb(self, msg):
		self.local_pos = msg.pose.position # x y z Update position
		self.local_ori = msg.pose.orientation # x y z w


	def stateCb(self, msg):
		self.state = msg #Update state

	def velCb(self, msg):
		self.local_vel_lin = msg.twist.linear #x y z
		self.local_vel_ang = msg.twist.angular #x y z


	# Drone thrust callback
	def thrustCb(self,msg):
		self.target_thrust=msg.thrust

	def dronestateCb(self,msg):
		self.dronestate=msg.data


	def pos_reached(self):
		if ((abs(self.sp.position.x-(self.local_pos.x))<0.1)) and ((abs(self.sp.position.y-(self.local_pos.y))) < 0.1) and ((abs(self.sp.position.z-(self.local_pos.z))) < 0.1):
			if (self.k==0):
				self.told = rospy.Time.now()
				self.k=1
		else:
			self.k=0
		if (self.k==1) and (rospy.Time.now()-self.told)>rospy.Duration(secs=2):
			self.k=0
			return True
		return False


	def hit_plane(self):
		if ((-1)*((self.local_pos.y)-1) < 0) and ((self.local_pos.y)>0):
			return True
		return False


	def away_plane(self):
		if ((-1)*((abs(self.local_pos.y))-0.5) >=0) and ((self.local_pos.y)>0) or ((self.local_pos.y)<0):
			return True
		return False


	def count_time(self):
		if (self.k==0):
			self.told = rospy.Time.now()
			self.k=1
		if  (self.k==1) and (rospy.Time.now()-self.told)>rospy.Duration(secs=10):
			self.k=0
			return True
		return False


	def contact_controller(self):
		#Set output pitch and and yaw to input
		qB = [self.local_ori.x, self.local_ori.y, self.local_ori.z, self.local_ori.w]


		self.eB = euler_from_quaternion(qB, 'rzyx')
		setp_pitch = self.eB[1]
		setp_yaw = self.eB[0]

		#Create new contact frame.
		RB = quaternion_matrix(qB)
		RB = RB[0:3,0:3]
		RCB = euler_matrix(0, (-1)*self.contact_pitch, 0,'rzyx')
		RCB = RCB[0:3,0:3]
		RC = np.dot(RB,RCB)

		#pitch and yaw from contact frame
		eC = euler_from_matrix(RC,'rzyx')
		rzC = eC[0]
		ryC = eC[1]

		#Transform angular vel in horizontal contact frame
		wC = np.array([self.local_vel_ang.x, self.local_vel_ang.y, self.local_vel_ang.z])
		RP = euler_matrix(rzC, 0, 0,'rzyx')
		RP = RP[0:3,0:3]

		wCP = np.dot(np.transpose(RP),wC)

		kp_yaw = 0.3
		kd_yaw = 0.3

		#Pd controller yaw
		setp_roll = kp_yaw*(self.contact_yaw - rzC) - kd_yaw*wC[2]

		#Pd controller pitch
		thrust = -self.rod_length*(self.position_gain*ryC + self.velocity_gain*wCP[1]) + self.mass_total*self.g #we want the pitch of contact frame to become 0


		#Thrust divided with R33
		if (np.linalg.norm(qB, 2) > 0.5):
			R = quaternion_matrix(qB)
			R33 = R[2,2]
		else:
			R33=1


		thrust = thrust/R33
		acc = thrust/self.mass_total

		acc_map = (acc-self.p2)/self.p1


		#Limitations
		if (acc_map > 0.8):
			acc_map = 0.8
		elif (acc_map < 0.1):
			acc_map = 0.1

		if (setp_roll > 0.2618): # 15 deg
			setp_roll = 0.2618
		elif (setp_roll < -0.2618):
			setp_roll = -0.2618

		if (setp_pitch > 0.524): # 30 deg
			setp_pitch = 0.524
		elif (setp_pitch < -0.524):
			setp_pitch = -0.524

		if (setp_yaw > 3.1416):
			setp_yaw = 3.1416
		elif (setp_yaw < -3.1416):
			setp_yaw = -3.1416

		q = quaternion_from_euler(setp_yaw, setp_pitch, setp_roll,'rzyx')
		self.att.orientation = Quaternion(*q)
		self.att.thrust = acc_map




# Main function
def main():
	# initiate node
	rospy.init_node('controller_node', anonymous=True)

	# flight mode object
	modes = FcuModes()

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

	# Subscribe to drone's Thrust
	rospy.Subscriber('dronestate', UInt8, cnt.dronestateCb)


	# Make sure the drone is armed
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
			if print_state==0:
				state_string="TAKEOFF"
				rospy.loginfo(state_string)
				print_state=1
			if set_setp==0:
				cnt.sp.position.x=0.0
				cnt.sp.position.y=0.0
				cnt.sp.position.z=cnt.ALT_SP
				set_setp=1
			cnt.dronestate_pub.publish(state)
			cnt.sp_pub.publish(cnt.sp)
			if cnt.pos_reached():
				SETP_THRUST = cnt.target_thrust
				set_setp=0
				print_state=0
				state+=1
		if (state==1):
			if print_state==0:
				state_string="ATTITUDE RUNNING"
				rospy.loginfo(state_string)
				print_state=1
			if set_setp==0:
				cnt.att.orientation = Quaternion(*quaternion_from_euler(0, 0.05, 1.57))
				cnt.att.thrust = SETP_THRUST
				set_setp=1
			cnt.dronestate_pub.publish(state)
			cnt.att_setpoint_pub.publish(cnt.att)
			if (cnt.hit_plane()):
				rospy.loginfo("HIT PLANE!!!")
				set_setp=0
				print_state=0
				state+=1
		if (state==2):
			if print_state==0:
				state_string="CONTROLLER RUNNING"
				rospy.loginfo(state_string)
				print_state=1
			cnt.dronestate_pub.publish(state)
			cnt.contact_controller()
			cnt.att_setpoint_pub.publish(cnt.att)
			if (cnt.away_plane()) or (abs(cnt.sp.position.z-abs(cnt.local_pos.z)) > 1) or (abs(cnt.sp.position.x-abs(cnt.local_pos.x)) > 0.5) or (abs(cnt.eB[1])>0.56) or (abs(cnt.eB[2])>0.34) or (abs(cnt.eB[0])>3.1416) or (abs(cnt.target_thrust)>0.9) or (cnt.count_time()):
				set_setp=0
				print_state=0
				state+=1
		if (state==3):
			if print_state==0:
				state_string="POS BEFORE LAND"
				rospy.loginfo(state_string)
				print_state=1
			if set_setp==0:
				cnt.sp.position.x=cnt.local_pos.x
				cnt.sp.position.y=0.0
				cnt.sp.position.z=cnt.local_pos.z
				set_setp=1
			cnt.dronestate_pub.publish(state)
			cnt.sp_pub.publish(cnt.sp)
			if cnt.pos_reached():
				set_setp=0
				print_state=0
				state+=1
		if (state==4):
			if print_state==0:
				state_string="LAND"
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
