class Follower:
	def __init__(self, P, I, D, rate):
		self.P = P
		self.I = I
		self.D = D
		self.rate = rate
		self.integral_prior = 0
		self.error_prior = 0

	def control(self, target, current):
		"""
		Parameters
		----------
		target 	- float - represents the x value of the goal location
		current - float	- represents the x value of the current location

		Returns
		-------
		theta - float - represents what the steering angle should be
		"""

		# Calculate the new desired angle
		error = target - current
		integral = self.integral_prior + error * self.rate
		derivative = (error - self.error_prior) / self.rate
		theta = (self.P * error) + (self.D * derivative) + (self.I * integral)

		# Adjust integral and derivative values
		self.error_prior = error
		self.integral_prior = integral

		# Limit checks
		# Deprecated because I chnged the range of the servos
		# theta = min(1000, max(0, theta))

		# TODO: Firgure out how to manage speed with depth input
		return theta


# Previous example of PID control

# class robot_follower:
#     def __init__(self):
#         drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
#         drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

#         listener = tf.TransformListener()
# 		trans = []
# 		rot = []
# 		integral_prior = 0
# 		error_prior = 0
# 		kp = 0.5
# 		kd = 0.05
# 		ki = 0
# 		rate = 0.01
#         while not rospy.is_shutdown():
# 			try:
# 				(trans, rot) = listener.lookupTransform('/tag_0', 'camera_color_optical_frame', rospy.Time(0))
# 			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
# 				continue
# 			print(trans)
# 			x = float("{0:.2f}".format(trans[0]))
# 			angle  = rot[2]
# 			error  = 0 - angle
# 			integral = integral_prior + error *rate
# 			derivative = (error - error_prior)/rate
# 			theta = kp*error + kd*derivative + ki*integral
# 			error_prior = error
# 			integral_prior = integral

# 			if trans[2] >= 0.4:
# 				#curv = ((2*x)/(x*x+z*z))
# 				velocity = 0.5
# 				theta  = theta
				
# 			else:
# 				velocity = 0
# 				theta = 0
# 			print(theta)
# 			drive_msg = AckermannDriveStamped()
# 			drive_msg.header.stamp = rospy.Time.now()
# 			drive_msg.header.frame_id = "camera_color_optical_frame"
# 			drive_msg.drive.steering_angle = theta
# 			drive_msg.drive.speed = velocity
# 			drive_pub.publish(drive_msg)


    

# def main(args):
#     rospy.init_node('robot_follower')
#     rf = robot_follower()
#     rospy.spin()
# if __name__ == '__main__':
#     main(sys.argv)
