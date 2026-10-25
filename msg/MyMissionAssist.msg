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

# "缺掉的水平力 -> 归一化前推力"的换算系数，由任务模块从机型的物理参数算出来
# （MPC_THR_HOVER、CA_ROTOR0_CT、CA_ROTOR4_AX/AZ），而不是让用户另填一个可能
# 和它们矛盾的值。控制器只在消息新鲜时用它，过期退回参数 MYM_FWD_THR_SC。
#
#   0 = 不覆盖，控制器用参数里的值（兼容旧配置）
float32 fwd_thrust_sc
