std_msgs/Header header

float32 roll
float32 pitch
float32 yaw
float32 gx
float32 gy
float32 gz
float32 ax
float32 ay
float32 az
float32 temperature
float32  time
uint8[13]   gps_message
uint8   gps_heading_status
uint8   ptype
uint16  pdata
float32  ver_pos
float32  ver_vel
uint16  info_byte

#0x00:deinit
#0x01:动中通版本数据帧
#0x02:VG版本数据帧
#0x03:升沉版本数据帧
uint8   ag041_type
