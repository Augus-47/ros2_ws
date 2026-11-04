std_msgs/Header header
float64 gnss_lon     
float64 gnss_lat
# gnss_高度
float64 gnss_alt
# gnss_经度精度
float32 gnss_lon_sigma
# gnss_纬度精度
float32 gnss_lat_sigma
# gnss_高度精度
float32 gnss_alt_sigma
# gnss_ve
float32 gnss_ve
# gnss_vn
float32 gnss_vn
# gnss_vu
float32 gnss_vu
# gnss_ve精度
float32 gnss_ve_sigma
# gnss_vn精度
float32 gnss_vn_sigma
# gnss_vu精度
float32 gnss_vu_sigma
# RTKAge
uint16 gnss_age
# HDOP
float32 gnss_hdop
# VDOP
float32 gnss_vdop
# NumSV used
uint8 gnss_mas_antusedsatnum
# NumSV  Visible
uint8 gnss_satnum
# NumSV sn0>40db
uint8 gnss_mas_anthighqualitysatnum
# gnss_ Time
uint64 gnss_time
# gnss_ Trk
float32 gnss_trk
# gnss_ Status
uint8 gnss_stat

#ENU pos
float32 pos_up
float32 pos_east
float32 pos_north
