#include <chrono>
#include <memory>
#include <serial/serial.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "imu_node/imu_pub.hpp"
#include "imu_msgs/msg/asensing.hpp"
#include "imu_msgs/msg/imu.hpp"
#include "imu_msgs/msg/gnss.hpp"
#include <utility>
#include "imu_node/protocol_asensing.h"
#include "imu_node/TinyLog.h"
#include <algorithm>

using namespace std;

#define E_NOT_OK -1
#define E_OK 0
// #define BAUDRATE B38400
#define BAUDRATE B230400
#define HEADER_SIZE 3
#define MIN(x,y) (x)>(y)?(y):(x)

// constexpr int Length_0B = 63;//according to pdf
// constexpr int Length_0A = 34;//according to pdf
// constexpr int Length_10 = 83;//according to pdf
// auto message = std::make_unique<imu_msgs::msg::Imu>();
// auto message_gnss = std::make_unique<imu_msgs::msg::Gnss>();
imu_msgs::msg::Imu message;
imu_msgs::msg::Gnss message_gnss;
imu_msgs::msg::AG041 message_AG041;
imu_msgs::msg::AG072 message_AG072;
imu_msgs::msg::AG051 message_AG051;


ProtocolAsensing connect_manager;

void Imu_Pub::updateInsData(void *origin_id)
{

  auto imu_msg = sensor_msgs::msg::Imu();
  LooselyDataStruct *pLoosely = (LooselyDataStruct*)origin_id;
  if(pLoosely->type == 0x04)
  {
    updateAG041Data(origin_id);
  }  
  else if(pLoosely->type == 0x0A)
  {
    updateAG072Data(origin_id);
  }
  else if(pLoosely->type == 0x8B)
  {
    updateAG051Data(origin_id);
  }
  else if(pLoosely->type == 0x0B)
  {
    message.imu_msg.latitude = pLoosely->lat;
    message.imu_msg.longitude = pLoosely->lon;
    message.imu_msg.altitude = pLoosely->h;

    message.imu_msg.north_velocity = pLoosely->vn;
    message.imu_msg.east_velocity = pLoosely->ve;
    message.imu_msg.ground_velocity = pLoosely->vd;

    message.imu_msg.roll = pLoosely->roll;
    message.imu_msg.pitch = pLoosely->pitch;
    message.imu_msg.azimuth = pLoosely->yaw;

    message.imu_msg.x_angular_velocity = pLoosely->gx;
    message.imu_msg.y_angular_velocity = pLoosely->gy;
    message.imu_msg.z_angular_velocity = pLoosely->gz;


    message.imu_msg.x_acc = pLoosely->ax;
    message.imu_msg.y_acc = pLoosely->ay;
    message.imu_msg.z_acc = pLoosely->az;

    message.imu_msg.latitude_std = pLoosely->xlat;
    message.imu_msg.longitude_std = pLoosely->xlon;
    message.imu_msg.altitude_std = pLoosely->xh;

    message.imu_msg.north_velocity_std = pLoosely->xvn;
    message.imu_msg.east_velocity_std = pLoosely->xve;
    message.imu_msg.ground_velocity_std = pLoosely->xvd;

    message.imu_msg.roll_std = pLoosely->xroll;
    message.imu_msg.pitch_std = pLoosely->xpitch;
    message.imu_msg.azimuth_std = pLoosely->xyaw;

    message.imu_msg.sec_of_week = pLoosely->gps_second_of_week;
    message.imu_msg.gps_week_number = pLoosely->gps_week;

    message.imu_msg.temperature = pLoosely->temp;
    message.imu_msg.wheel_speed_status = pLoosely->wheelselectr;
    message.imu_msg.ins_status = pLoosely->IMUState;
    message.imu_msg.position_type = pLoosely->stater;
    message.imu_msg.heading_type = pLoosely->stater;
    message.imu_msg.numsv = pLoosely->numSV;

    message.header.stamp = rclcpp::Clock().now();
    publisher_->publish(std::move(message));



    //消息转化为ros imu msg
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.header.frame_id = "imu_link";
    
    // 填充orientation，使用RPY角转换为四元数
    //double roll = pLoosely->roll / 57.3;
    //double pitch = pLoosely->pitch / 57.3;
    //double yaw = pLoosely->yaw / 57.3;
    //const double G_INS570 = 9.7883105; // 依据说明书第9页注1 
    const double DEG_TO_RAD = M_PI / 180.0;

    // --- 1. 姿态角转换 (Orientation) ---
    double roll = pLoosely->roll * DEG_TO_RAD;   // 右倾为正 
    double pitch = -pLoosely->pitch * DEG_TO_RAD; // 抬头为负

    // 航向角修正：将“正北为0、顺时针为正”转换为“正东为0、逆时针为正”
    double yaw_ins = pLoosely->yaw * DEG_TO_RAD;
    double yaw_ros = (M_PI / 2.0) - yaw_ins;   // 处理90度偏置与方向翻转
    // 归一化 yaw 到 [-PI, PI] (推荐增加)
    while (yaw_ros > M_PI)  yaw_ros -= 2.0 * M_PI;
    while (yaw_ros < -M_PI) yaw_ros += 2.0 * M_PI;
    
    tf2::Quaternion q;
    tf2::Matrix3x3 m;
    //m.setRPY(roll, pitch, yaw);
    m.setRPY(roll, pitch, yaw_ros);
    m.getRotation(q);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    // 填充angular_velocity
    //imu_msg.angular_velocity.x = pLoosely->gx / 57.3;
    //imu_msg.angular_velocity.y = pLoosely->gy / -57.3;
    //imu_msg.angular_velocity.z = pLoosely->gz / -57.3;
    imu_msg.angular_velocity.x = pLoosely->gx *M_PI / 180.0;
    imu_msg.angular_velocity.y = pLoosely->gy *-M_PI / 180.0;
    imu_msg.angular_velocity.z = pLoosely->gz *-M_PI / 180.0;


    // 填充linear_acceleration
    imu_msg.linear_acceleration.x = pLoosely->ax * 9.7883105;
    imu_msg.linear_acceleration.y = pLoosely->ay * -9.7883105;
    imu_msg.linear_acceleration.z = pLoosely->az * -9.7883105;

    imu_publisher_->publish(std::move(imu_msg));

  }
}

void Imu_Pub::updateGnssData(void *origin_id)
{
  GnssStruct *pGnss = (GnssStruct *)origin_id;
  //if(pLoosely->protocol == 0x04)
  {
    // GNSS经度
    message_gnss.gnss_lon = pGnss->lon;
    // GNSS纬度
    message_gnss.gnss_lat = pGnss->lat;
    cout.precision(10);
    std::cout << "经度: " << message_gnss.gnss_lon << "\t" <<  "纬度: " << message_gnss.gnss_lat << std::endl;
    // GNSS高度
    message_gnss.gnss_alt = pGnss->h; 
    // GNSS经度精度
    message_gnss.gnss_lon_sigma=pGnss->lonstd;
    // GNSS纬度精度
    message_gnss.gnss_lat_sigma=pGnss->latstd;
    // GNSS高度精度
    message_gnss.gnss_alt_sigma=pGnss->hMSLstd;
    // GnssVe
    message_gnss.gnss_ve=pGnss->ve;
    // GnssVn
    message_gnss.gnss_vn=pGnss->vn;
    // GnssVu
    message_gnss.gnss_vu=pGnss->vd;
    // GnssVe精度
    message_gnss.gnss_ve_sigma=0;
    // GnssVn精度
    message_gnss.gnss_vn_sigma=0;
    // GnssVu精度
    message_gnss.gnss_vu_sigma=0;
    // RTKAge
    message_gnss.gnss_age=pGnss->age;
    // HDOP
    message_gnss.gnss_hdop=pGnss->hDOP;
    // VDOP
    message_gnss.gnss_vdop=pGnss->vDOP;
    // NumSV used
    //message_gnss.gnss_mas_antusedsatnum=((uint8_t)GnssMasAntUsedSatNum);
    // NumSV  Visible
    message_gnss.gnss_satnum=pGnss->numSV;
    // NumSV sn0>40db
    //message_gnss.gnss_mas_anthighqualitysatnum=((uint8_t)GnssMasAntHighQualitySatNum);
    // GNSS Time
    message_gnss.gnss_time=pGnss->ts;
    // GNSS Trk
    message_gnss.gnss_trk=pGnss->Trk;
    // GNSS Status
    //message_gnss.gnss_stat=((uint8_t)GnssStat);
    message.header.stamp = rclcpp::Clock().now();
    gnss_publisher_->publish(std::move(message_gnss));
  }
}

void Imu_Pub::updateAG041Data(void *origin_id)
{
  LooselyDataStruct *pLoosely = (LooselyDataStruct *)origin_id;
  message_AG041.roll = pLoosely->roll;
  message_AG041.pitch = pLoosely->pitch;
  message_AG041.yaw = pLoosely->yaw;
  message_AG041.gx = pLoosely->gx;
  message_AG041.gy = pLoosely->gy;
  message_AG041.gz = pLoosely->gz;
  message_AG041.ax = pLoosely->ax;
  message_AG041.ay = pLoosely->ay;
  message_AG041.az = pLoosely->az;
  message_AG041.temperature = pLoosely->temp;
  message_AG041.time = pLoosely->ts;
  memcpy((uint8_t*)&message_AG041.gps_message,(uint8_t*)pLoosely->gps_message,sizeof(message_AG041.gps_message));
  message_AG041.gps_heading_status = pLoosely->gps_heading_status;
  message_AG041.ptype = pLoosely->ptype;
  message_AG041.pdata = pLoosely->pdata;
  message_AG041.ver_pos = pLoosely->ver_pos;
  message_AG041.ver_vel = pLoosely->ver_vel;
  message_AG041.info_byte = pLoosely->info_byte;
  message_AG041.ag041_type = pLoosely->ag041_type;

  message_AG041.header.stamp = rclcpp::Clock().now();
  ag041_publisher_->publish(std::move(message_AG041));
}

void Imu_Pub::updateAG072Data(void *orignin_id)
{
  LooselyDataStruct *pLoosely = (LooselyDataStruct*)orignin_id;
  message_AG072.gx = pLoosely->gx;
  message_AG072.gy = pLoosely->gy;
  message_AG072.gz = pLoosely->gz;
  message_AG072.ax = pLoosely->ax;
  message_AG072.ay = pLoosely->ay;
  message_AG072.az = pLoosely->az;
  message_AG072.temperature = pLoosely->temp;
  message_AG072.status = pLoosely->ptype;
  message_AG072.frame_count = pLoosely->frame_count;

  ag072_publisher_->publish(std::move(message_AG072));
}
void Imu_Pub::updateAG051Data(void *orignin_id)
{
  LooselyDataStruct *pLoosely = (LooselyDataStruct*)orignin_id;
  message_AG051.type = pLoosely->type;
  message_AG051.data_length = pLoosely->pdata;
  message_AG051.frame_count = pLoosely->gps_second_of_week;
  message_AG051.serial_number = pLoosely->ptype;
  message_AG051.gx = pLoosely->gx;
  message_AG051.gy = pLoosely->gy;
  message_AG051.gz = pLoosely->gz;
  message_AG051.ax = pLoosely->ax;
  message_AG051.ay = pLoosely->ay;
  message_AG051.az = pLoosely->az;
  message_AG051.temperature = pLoosely->temp;
  message_AG051.status = pLoosely->IMUState;
 
  ag051_publisher_->publish(std::move(message_AG051));
}

Imu_Pub::Imu_Pub() 
:Node("Imu_Pub_Node"), port(UART_PORT), frameid("imu_link")
{
  std::flush(std::cout);
  //std::cout<<"进入Imu_Pub_Node构造函数\n";
  //exit(0);

  this->declare_parameter<std::string>("UART_Port", UART_PORT);
  this->declare_parameter<int>("UART_Baudrate", UART_BAUDRATE);
  this->declare_parameter<int>("AG041_Type", 0);
  this->declare_parameter<float>("AG041_Grange", 250);
  this->declare_parameter<float>("AG041_Arange", 4);
  this->declare_parameter<float>("570D_Grange", 250);
  this->declare_parameter<float>("570D_Arange", 4);
  this->declare_parameter<std::string>("LogInfo", "");
  this->declare_parameter<int>("LogLevel", 1);

  this->get_parameter("UART_Port",port);
  this->get_parameter("UART_Baudrate",baud);
  this->get_parameter("AG041_Type",connect_manager.ag041_param_dat.AG041_Type);
  this->get_parameter("AG041_Grange",connect_manager.ag041_param_dat.Grange);
  this->get_parameter("AG041_Arange",connect_manager.ag041_param_dat.Arange);
  this->get_parameter("570D_Grange",connect_manager.grange_scale);
  this->get_parameter("570D_Arange",connect_manager.arange_scale);
  /* set log config */
  std::string logInfo = "";
  int32_t logLevel = 1;
  this->get_parameter("LogInfo",logInfo);
  this->get_parameter("LogLevel",logLevel);
  TinyLog::setStorageDir(logInfo.c_str());
  TinyLog::setStorageLevel(logLevel);

  imu_ser.setPort(port);
  imu_ser.setBaudrate(baud);
  serial::Timeout to = serial::Timeout::simpleTimeout(0);
  imu_ser.setTimeout(to);
  imu_ser.open();
  if (imu_ser.isOpen())
  {
    TinyLog::info("imu_serial opened successfully!");
  }
  else 
  {
    TinyLog::error("imu_serial opened failed!");
    exit(0);
  }
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_msg", 100);//500

  publisher_ = this->create_publisher<imu_msgs::msg::Imu>("Imu_Pub", 100);//500
  gnss_publisher_ = this->create_publisher<imu_msgs::msg::Gnss>("Gnss_Pub", 100);//500
  ag041_publisher_ = this->create_publisher<imu_msgs::msg::AG041>("AG041_Pub", 100);//500
  ag072_publisher_ = this->create_publisher<imu_msgs::msg::AG072>("AG072_Pub", 100);//500
  ag051_publisher_ = this->create_publisher<imu_msgs::msg::AG051>("AG051_Pub", 100);//500
  timer_ = this->create_wall_timer(50ms, std::bind(&Imu_Pub::timer_callback, this));

  _register_ins_fun = std::make_shared<std::function<void(void*)>>(std::bind(&Imu_Pub::updateInsData, this, std::placeholders::_1));
  
  connect_manager.addAnalysisLooselyDataCallback (_register_ins_fun);

   _register_gnss_fun = std::make_shared<std::function<void(void*)>>(std::bind(&Imu_Pub::updateGnssData, this, std::placeholders::_1));
  connect_manager.addAnalysisGnssDataCallback (_register_gnss_fun); 
}

void Imu_Pub::timer_callback()
{

    try
    {
      if (imu_ser.isOpen())
      {
        if(imu_ser.available())
        {
          
          read = imu_ser.read(imu_ser.available());
          input += read;
          connect_manager.addData(input);
          input.erase(0,input.size());
          #if 0
          while (input.size() >= .HEADER_SIZE)
          {
            auto pos_0B = input.find(BDDB0B,0,HEADER_SIZE);
            auto pos_10 = input.find(BDDB10,0,HEADER_SIZE);
            if(pos_0B==std::string::npos &&  pos_10==std::string::npos)//no header is found
            {
              input.erase(0,input.size());
            }else//at least one header is found
            {
              auto pos_min=MIN(pos_0B,pos_10);//check who come first

        
              /************************************************************/
              /*0B process*/
              /************************************************************/
              if(pos_min==pos_0B)//0B first
              {
                if(pos_0B!=0)//there are surplus data
                {
                  input.erase(0,pos_0B);//delete surplus data
                  continue;
                }
                else//pos 0B=0, start parsing 0B
                {
                  if (input.size() < Length_0B)
                  {
                    break;//not enough data
                  }
                  if(crc_check(Length_0B,input)==0)//crc check
                  {
                    parsing_0B(input);
                    message.header.stamp = rclcpp::Clock().now();
                    publisher_->publish(std::move(message));
                  }
                  else{
                    //std::cout<<"0B XOR check wrong! \n"<<std::endl;
                  }
                  input.erase(0,Length_0B);
                  continue;
                }
              }
              /**************************************************************/

              /************************************************************/
              /*0A process*/
              /************************************************************/
              if(pos_min==pos_10)//0A first
              {
                if(pos_10!=0)//there are surplus data
                  {
                    input.erase(0,pos_10);//delete surplus data
                    continue;
                  }
                  else//pos 0A=0, start parsing 0A
                  {
                    if(input.size() < Length_10)
                    {
                      break;//not enough data
                    }

                    if(crc_check(Length_10,input)==0)//crc check
                    {
                      parsing_10(input);

                      message_gnss.header.stamp = rclcpp::Clock().now();
                      gnss_publisher_->publish(std::move(message_gnss));
                    }
                    else {
                      //std::cout<<"10 XOR check wrong! \n"<<std::endl;
                    }
                    input.erase(0,Length_10);
                    continue;
                  }
              }
            }
          }
          #endif 
        }
      }
      else
      {
        try
        {
          imu_ser.setPort(port);
          imu_ser.setBaudrate(baud);//115200  230400
          serial::Timeout to = serial::Timeout::simpleTimeout(0);
          imu_ser.setTimeout(to);
          imu_ser.open();
        }
        catch (serial::IOException& e)
        {
          //std::cout << "Unable to open serial port " << imu_ser.getPort() << ". Trying again in 5 seconds.";
        //   ros::Duration(5).sleep();need rebuild
        }

        if( imu_ser.isOpen() )// imu_ser.SerialImpl::open()
        {
        }
      }
    }
    catch (serial::IOException& e)
    {
      imu_ser.close();
    }
}



int main(int argc, char** argv)
{

    //初始化节点
    rclcpp::init(argc, argv);
    auto ppp = std::make_shared<Imu_Pub>();
    rclcpp::spin(ppp);
    rclcpp::shutdown();
    return 0;
}
