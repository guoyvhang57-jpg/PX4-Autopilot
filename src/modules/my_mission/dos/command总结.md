landDetectorUpdate();
读取_vehicle_land_detected话题，主要是land
写入_vehicle_status.takeoff_time 起飞时间
_have_taken_off_since_arming  第一次起飞标志位
_home_position 向此对象写入home点


handleAutoDisarm
通过disarm，判断上锁后
_vehicle_status.armed_time = 0;//解锁时间置0
_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;//解锁状态为上锁。影响isArmed为0
_vehicle_status.latest_disarming_reason = (uint8_t)calling_reason;//上一次的上锁理由
_vehicle_status.takeoff_time = 0;//起飞时间置-

_have_taken_off_since_arming = false;//

_last_disarmed_timestamp = hrt_absolute_time();

_user_mode_intention.onDisarm();//发布上锁意图，会过fail，但是由于_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;不起作用

checkForMissionUpdate
在自动任务结束后判断该发什么意图

offboardControlCheck
丢失后给_status_changed为真

_failure_detector.update(_vehicle_status, _vehicle_control_mode)) {
			_status_changed = true;
		}
_failure_detector._failure_detector_status.flags.roll
_failure_detector._failure_detector_status.flags.pitch
_failure_detector._failure_detector_status.flags.alt
同时出问题给state_change=1
