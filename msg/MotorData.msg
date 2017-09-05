# This custom message holds the data for a specific motor.

int32 position			# Motor encoder position actual value in encoder steps.
int32 velocity			# Motor velocity in rpm.
int16 current			# Current actual value averaged (mA).
int16 following_error	# Following error actual value (encoder steps), not for EPOS4.
uint16 status			# Statusword of the motor controller.
int8 mode_of_operation	# Mode of operation display.
