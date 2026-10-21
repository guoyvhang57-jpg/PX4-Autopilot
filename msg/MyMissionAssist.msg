uint64 timestamp	# time since system start (microseconds)

# Per-phase permission for the forward-thrust assist (MYM_FWD_THR_EN).
#
# false = run the plain position controller: no nose-down pitch clamp and no
# forward thrust. The mission uses this to stop the assist once a transit leg
# starts decelerating, where the attitude is nose-up and the clamp has nothing
# to hand the thrusters.
#
# Consumers must treat a stale message (no update for ~0.5 s) as "true", so a
# mission that dies mid-flight cannot silently disable the assist.
bool enabled
