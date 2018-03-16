# This custom message allows to define a specific command for a specific motor.
# Chose the motor you want to control by setting the nodeID, chose the mode of control like current, position, velocity, force...
# Chose a value according to the mode, position in encoder tick, current in mA...
#
# Example: slaveBoardID 2, nodeID 3, command 1, value 1000: This will send a target velocity command of 1000 rpm to the motor with a nodeID 3 on the CAN bus of the SlaveBoard number 2.
# 											(Assuming the activated Mode of Operation is Profile Velocity Mode)

uint8 slaveBoardID	# the command will be redirected to this SlaveBoard ID by the MasterBoard
uint8 nodeID		# node ID on the CAN bus
uint8 command		# command like set target position, set profile acceleration, set controlword...
int32 value		# command value
