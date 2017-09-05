# This custom message defines a variable that has motor_data attributes plus some extended ones, calculated from others.

MotorData motor_data				# Motor data type.
int32 velocity						# Velocity, calculated as the derivative of position over time.
int32 acceleration					# Acceleration, calculated as the derivative of velocity over time.
int16 current_derivative 			# Derivative of the current over time.
int16 current_second_derivative 	# Second derivative of the current over time.
