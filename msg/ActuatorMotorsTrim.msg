# Motor trims, added to the allocated motor outputs
#
# Normalised offset per motor, added to actuator_motors.control by the control allocator before
# the actuator limits are applied. The intended use is a steady trim that the allocator itself
# cannot produce because it is deliberately kept out of the effectiveness matrix, e.g. a
# differential on a symmetric pair of thrusters that is being used to null an unmodelled yaw
# disturbance.
#
# Publishers must keep the sum of the trims at zero for any pair they drive differentially,
# otherwise the trim also changes the collective thrust.
uint64 timestamp			# time since system start (microseconds)

uint8 NUM_CONTROLS = 12
float32[12] trim    # range: [-1, 1]
