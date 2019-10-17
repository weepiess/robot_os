#chassis module data.
########### id of publisher
uint8 id
#chassis_type: 
# 0x00  location
# 0x01  swing range
# 0x02  rand range
# others define by developer
uint8 chassis_type
########
float32 x
float32 y
float32 angle
float32 v_x
float32 v_y
float32 a_x
float32 a_y
#########
float32 x_range_min
float32 x_range_max
float32 y_range_min
float32 y_range_max