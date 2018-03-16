# This custom message allows to define a multi array of motor commands.
# Example : If your robot has one Master Board, you can define it as a 6*16 matrix, 6 Salve Boards with 16 nodes each. 

std_msgs/String			   rosNodeName	 # name of the ROS node that publish this motor command array.
std_msgs/MultiArrayLayout  layout        # specification of data layout.
MotorCmd[] 		  		   motorCmd		 # array of MotorCmd.
