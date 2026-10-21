uint64 timestamp	# time since system start (microseconds)

# Per-phase permission for the forward-thrust assist (MYM_FWD_THR_EN).
#
# false = run the plain position controller: no nose-down pitch clamp and no
# forward thrust. The mission uses this to stop the assist once a transit leg
# starts decelerating, where the attitude is nose-up and the clamp has nothing
# to hand the thrusters.
#
# Consumers must treat a stale message (no update for ~0.5 s) as "false": the
# assist only runs while the mission is actually managing it. The price is that
# the feature becomes my_mission-only -- flying offboard from MAVSDK or ROS 2
# will not get the assist.
bool enabled
