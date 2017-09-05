# This custom message allows to define a specific command for a specific motor.
# Chose the motor you want to control by setting the node Id, then chose the type of command and its value.
#
# Example: node_id 3, command 1, value 1000: This will send a target velocity command of 1000 rpm to the motor with a node Id 3 on the CAN bus.

uint8 node_id		# Node Id on the CAN bus.
uint8 command		# Command like set target position, set profile acceleration, set controlword...
int32 value			# Command value (position are in encoder tick, velocity in rpm, current in mA...).
