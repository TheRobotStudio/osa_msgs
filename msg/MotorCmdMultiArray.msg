# This custom message allows to define a multi array of motor commands.

#std_msgs/String	ros_node_name		# Name of the ROS node that publish this motor command array.
std_msgs/MultiArrayLayout layout    # Specification of data layout.
MotorCmd[] motor_cmd				# Array of MotorCmd.
