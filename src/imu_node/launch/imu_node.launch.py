"""Launch a talker and a listener in a component container."""
 
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
 
 
def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            package="imu_node",
            executable="imu_pub",
            namespace='',
            name="imu_node",
            output="screen",

            #Imu_node 节点运行配置参数
            parameters=[
				#串口设备串
                {"UART_Port": "/dev/ttyUSB0"},
				
				#串口波特率
                {"UART_Baudrate": 230400},
				
				#AG041协议类型
				#0x00:deinit
				#0x01:动中通版本数据帧
				#0x02:VG版本数据帧
				#0x03:升沉版本数据帧
                {"AG041_Type": 1},
				
				#AG041协议：陀螺量程
                {"AG041_Grange": 250.0},
				
				#AG041协议：加表量程
                {"AG041_Arange": 4.0},
                
				#570D协议：陀螺量程
				{"570D_Grange": 4.0},
                
				#570D协议：加表量程
				{"570D_Arange": 4.0},

                #设置日志文件路径和文件名
                #default: close
               # {"LogInfo":"./debug.log"},

                #设置日志打印等级：DEBUG:0,INFO:1,WARNING:2,ERROR:3,FATAL:4
                #default:INFO
                #{"LogLevel":1}
            ],
    )

    return launch.LaunchDescription([container])
