#gimbal module data,include pitch and yaw.
########### id of publisher
uint8 id
#control type
#  0x00 relative
#  0x01 absolute
#  0x02 local scan
#  0x03 global scan
#  other custom usage
uint8 gimbal_type
########
float32 yaw_angle
float32 pitch_angle
########
uint8 yaw_velocity_percent
uint8 pitch_velocity_percent
########
float32 yaw_scan_min
float32 yaw_scan_max

