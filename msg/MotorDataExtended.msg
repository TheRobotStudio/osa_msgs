# This custom message defines a variable that has motorData attributes plus some extended ones, calculated from others.

MotorData motorData				# motor data type
int32 velocity					# velocity, calculated as the derivative of position over time.
int32 acceleration				# acceleration, calculated as the derivative of velocity over time.
int16 currentDerivative 		# derivative of the current over time.
int16 currentSecondDerivative 	# second derivative of the current over time.
