# This custom message defines a multi array of extended motor data.
# Example : If your robot has one Master Board, you can define it as a 6*16 matrix, 6 Salve Boards with 16 nodes each.

std_msgs/MultiArrayLayout  layout        		# specification of data layout
MotorDataExtended[] 	   motorDataExtended	# array of MotorDataExtended, standard is 16
