通过监听底盘自定义的消息接口/orin_pose获得当前位置
然后通过转换得到x,y,yaw

直接运行即可：
chmod +x ~/ros2_ws/get_xyyaw/print_orin_xyyaw_once.py
python3 ~/ros2_ws/get_xyyaw/print_orin_xyyaw_once.py

------------------------------------------------------------------

sample_waypoint_pose介绍
1、手动把车停到要巡检的位置
2、车保持静止等待小车定位稳定
3、运行这个脚本
4、得到打印出来的x/y/yaw并写进咱们的配置文件里

脚本直接运行即可
chmod +x sample_waypoint_pose.py
python3 sample_waypoint_pose.py

