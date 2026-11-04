#ifndef __PROTOCOL_ASENSING_H__
#define __PROTOCOL_ASENSING_H__

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <map>
#include "common.h"
#include <iostream>
#include <algorithm>

struct LooselyDataStruct {
	uint8_t type = 0;//数据来源id 0x0B

	float roll = 0, pitch = 0, yaw = 0;//横滚角,俯仰角,方位角
	float yawDelay = 0;

	float gx = 0, gy = 0, gz = 0;//陀螺x,y,z轴
	float ax = 0, ay = 0, az = 0;//加表x,y,z轴
	double  lat = 0, lon = 0, h = 0;//纬度,经度,高度
	float vn = 0, ve = 0, vd = 0;//北向,东向,地向速度

	uint8_t		IMUState = 0;//状态
	uint32_t	LEKFState = 0;//卡尔曼滤波
	uint16_t	gpsState = 0;//gps状态

	float plat = 0, plon = 0, ph = 0;//轮询数据1,2,3  纬度,经度,高度	 polling_type=0时有效
	float pvn = 0, pve = 0, pvd = 0;//轮询数据1,2,3   北向,东向,地向速度 polling_type=1时有效
	float proll = 0, ppitch = 0, pyaw = 0;//轮询数据1,2,3	横滚角,俯仰角,方位角 polling_type=2时有效

	float temp = 0; //轮询数据1  温度， polling_type为22才有效
	float ts = 0;//时间
	uint8_t polling_type = 0;//轮询类型，0:定位信息精度（lat，lon，h），1：定速信息精度(vn, ve, vd) 
						 //，2：姿态信息精度(rollstd, pitchstd, yawstd)，22设备内部温度（data1温度）
						 //，32GPS状态（定位状态，收星数，定向状态），33轮速状态（data2 轮速标志）

	float bgx = 0, bgy = 0, bgz = 0;
	float bax = 0, bay = 0, baz = 0;
	float pbgx = 0, pbgy = 0, pbgz = 0;
	float pbax = 0, pbay = 0, pbaz = 0;

	float xlat = 0, xlon = 0, xh = 0;
	float xvn = 0, xve = 0, xvd = 0;
	float xroll = 0, xpitch = 0, xyaw = 0;

	float xbgx = 0, xbgy = 0, xbgz = 0;
	float xbax = 0, xbay = 0, xbaz = 0;

	float aroll = 0, apitch = 0, ayaw = 0;
	float paroll = 0, papitch = 0, payaw = 0;
	float xaroll = 0, xapitch = 0, xayaw = 0;

	float lbbgx = 0, lbbgy = 0, lbbgz = 0;
	float xlbbgx = 0, xlbbgy = 0, xlbbgz = 0;
	float plbbgx = 0, plbbgy = 0, plbbgz = 0;

	float lbbcx = 0, lbbcy = 0, lbbcz = 0;
	float plbbcx = 0, plbbcy = 0, plbbcz = 0;
	float xlbbcx = 0, xlbbcy = 0, xlbbcz = 0;

	float kws0 = 0, kws1 = 0;
	float pkws0 = 0, pkws1 = 0;
	float xkws0 = 0, xkws1 = 0;

	float tsRaw = 0;
	float timeIMU = 0;
	float timeLEKF = 0;
	float timeGPS2IMU = 0;

	double TimeSyncIMU = 0;
	double TimeSyncGPS = 0;

	bool   updata = 0;

	float atbxr = 0, atbyr = 0, atbzr = 0;
	float stdatbxr = 0, stdatbyr = 0, stdatbzr = 0;

	float atgxr = 0, atgyr = 0, atgzr = 0;
	float stdatgxr = 0, stdatgyr = 0, stdatgzr = 0;

	float ltgxr = 0, ltgyr = 0, ltgzr = 0;
	float stdltgxr = 0, stdltgyr = 0, stdltgzr = 0;


	float ltcxr = 0, ltcyr = 0, ltczr = 0;
	float stdltcxr = 0, stdltcyr = 0, stdltczr = 0;

	float kws1r = 0, kws2r = 0, kws3r = 0;
	float stdkws1r = 0, stdkws2r = 0, stdkws3r = 0;

	int32_t wheelselectr = 0;
	int32_t stater = 0;

	uint8_t numSV = 0;//导航星数

	float _pos_config_value = 0.0;//置信度
	uint8_t _pos_config_level = 0.0;//置信度等级

	//0x54 蔚来项目增加 2023/4/17
	uint16_t status_flag;//自检状态标志
	uint16_t frame_count;//帧计数
	uint16_t gps_week;//gps周
	uint32_t gps_second_of_week;//gps周内秒
	uint16_t gps_year = 0;
	uint8_t gps_month = 0, gps_day = 0;
	uint8_t	gps_hour = 0, gps_min = 0, gps_sec = 0;
	uint16_t gps_msec = 0;
	//update for AG041 protocol
	
	uint8_t	gps_heading_status;
	uint8_t   ptype;
	uint16_t  pdata;
	float ver_pos;
	float ver_vel;
	uint16_t  info_byte;
	uint8_t   ag041_type;
	uint8_t	gps_message[14];
	/// <summary>
	/// 转成明文字符串
	/// </summary>
	/// <returns></returns>
	std::string toString(const std::string& split);
};

struct GnssStruct {
	uint8_t type = 0;//0x10

	double lon = 0;  //经度
	float lonstd = 0;//经度精度
	double lat = 0;  //纬度
	float latstd = 0;//纬度精度
	double h = 0;	 //高程
	float hMSLstd = 0;//高程精度

	double vDOP = 0;
	uint32_t	gpsfixNovAtel = 0;
	uint32_t	flagsNovAtel = 0;

	float Hspd = 0, Trk = 0, VSpd = 0;//水平速度,航迹角,垂直速度
	float LatencyVel = 0;

	float BaseLineLength = 0;//基线长度
	float 	heading = 0;
	double cDOP = 0;

	float pitch = 0;//俯仰角
	float pitchStd = 0;//俯仰角精度
	uint16_t year = 0;//UTC年
	uint8_t month = 0;//月
	uint8_t day = 0;//日
	uint8_t hour = 0;//小时
	uint8_t min = 0;//分钟
	float sec = 0;//秒

	double itow_pos = 0, itow_vel = 0, itow_heading = 0;//

	uint8_t msg = 0;//
	uint8_t numSV = 0;//导航星数
	float ts = 0;//时间

	bool isNovAtel = false;

	double hDOP = 0;// '40~43
	double sDOP = 0;// '44~47
	float pDOP = 0;//  '55~56

	float  vn = 0, ve = 0, vd = 0;//北向，东向，地向速度
	float 	Speed3D = 0, Speed2D = 0;//

	uint8_t gpsfix = 0;// GPSfix Type,0x00 = No Fix; 0x01 = Dead Reckoning only; 0x02 = 2D-Fix; 
					   // 0x03 = 3D-Fix; 0x04 = GPS + dead reckoning combined;0x05 = Time only fix
	uint8_t flags = 0; // 0:GPSfixOK; 1:DiffSoln; 2:WKNSET; 3:TOWSET

	double iTowPosllh = 0, iTowVelned = 0;
	uint8_t age = 0;

	/// <summary>
	/// 转成明文字符串
	/// </summary>
	/// <returns></returns>
	std::string toString(const std::string& split);
};

struct PollingStruct
{
	float LFS = 0;
	float RFS = 0;
	float LRS = 0;
	float RRS = 0;
	uint8_t Shift = 0;
	bool error = 0;

	/// <summary>
	/// 转成明文字符串
	/// </summary>
	/// <returns></returns>
	std::string toString(const std::string& split);
};


struct AG041_Param
{
	//0x00:deinit
	//0x01:动中通版本数据帧
	//0x02:VG版本数据帧
	//0x03:升沉版本数据帧
	uint8_t AG041_Type;		
	
	//陀螺量程范围
	float 	Grange;
	//加表量程范围
	float	Arange;
};
/// <summary>
/// 解析导远内部协议
/// </summary>
class ProtocolAsensing
{
public:
	explicit ProtocolAsensing();
	~ProtocolAsensing();

	AG041_Param ag041_param_dat;

	float grange_scale = 300.0;
	float arange_scale = 3.0;

	/// <summary>
	/// 添加接收的数据，进行解析，自动处理粘包问题
	/// </summary>
	/// <param name="data"></param>
	void addData(const std::string& data);

	/// <summary>
	/// 外部主动刷新，主要用在回放
	/// </summary>
	/// <param name=""></param>
	void updateGnss(void*);
	void updateIns(void*);

	/// <summary>
	///最近一次收到的gnss数据
	/// </summary>
	/// <returns></returns>
	void* lastGnssData() const {
		return (void*)(&_gnss_data);
	}
	/// <summary>
	/// 最近一次收到的loosely数据
	/// </summary>
	/// <returns></returns>
	void* lastLooselyData() const {
		return (void*)(&_loosely_data);
	}
	/// <summary>
	/// 最近一次收到的车轮数据
	/// </summary>
	/// <returns></returns>
	void* lastPollData() const {
		return (void*)(&_poll);
	}

	/// <summary>
	/// 轮数
	/// </summary>
	/// <returns></returns>
	uint16_t wheelNum() const {
		return _wheel_num;
	}
	/// <summary>
	/// RTKAGE
	/// </summary>
	/// <returns></returns>
	uint16_t rtkAge() const {
		return _rtk_age;
	}
	/// <summary>
	/// 姿态发散
	/// </summary>
	/// <returns></returns>
	uint16_t angleDiffuse() const {
		return _angle_diffuse_num;
	}/// <summary>
	/// RTKAGE
	/// </summary>
	/// <returns></returns>
	uint16_t biasDiffuse() const {
		return _bias_diffuse_num;
	}

	/// <summary>
	/// 增加接收gnss数据的回调函数,每次正确解析后会调用（数据频率较高时影响界面显示效率）
	/// </summary>
	/// <param name="fun"></param>
	int addAnalysisGnssDataCallback(std::shared_ptr<std::function<void(void*)>> fun) {
		_gnss_funs.emplace_back(fun);
		return (int)_gnss_funs.size() - 1;
	}

	/// <summary>
	/// 移除接收gnss数据的回调函数
	/// </summary>
	/// <param name="fun"></param>
	void removeAnalysisGnssDataCallback(std::shared_ptr<std::function<void(void*)>> fun) {
		auto it = std::find(_gnss_funs.begin(), _gnss_funs.end(), fun);
		if (it != _gnss_funs.end()) {
			_gnss_funs.erase(it);
		}
	}

	/// <summary>
	/// 增加接收loosely数据的回调函数（数据频率较高时影响界面显示效率）
	/// </summary>
	/// <param name="fun"></param>
	int addAnalysisLooselyDataCallback(std::shared_ptr<std::function<void(void*)>> fun) {
		//std::cout << "进入addAnalysisLooselyDataCallback" << std::endl;
		_loosely_funs.emplace_back(fun);
		return (int)_loosely_funs.size() - 1;
	}

	/// <summary>
	/// 移除接收loosely数据的回调函数
	/// </summary>
	/// <param name="fun"></param>
	void removeAnalysisLooselyDataCallback(std::shared_ptr<std::function<void(void*)>> fun) {
		auto it = std::find(_loosely_funs.begin(), _loosely_funs.end(), fun);
		if (it != _loosely_funs.end()) {
			_loosely_funs.erase(it);
		}
	}

	/// <summary>
	/// 增加接收loosely数据的回调函数（数据频率较高时影响界面显示效率）
	/// </summary>
	/// <param name="fun"></param>
	int addAnalysisPollCallback(std::shared_ptr<std::function<void(void*)>> fun) {
		_poll_funs.emplace_back(fun);
		return (int)_poll_funs.size() - 1;
	}

	/// <summary>
	/// 移除接收loosely数据的回调函数
	/// </summary>
	/// <param name="fun"></param>
	void removeAnalysisPollCallback(std::shared_ptr<std::function<void(void*)>> fun) {
		auto it = std::find(_poll_funs.begin(), _poll_funs.end(), fun);
		if (it != _poll_funs.end()) {
			_poll_funs.erase(it);
		}
	}

	/// <summary>
	/// 设置状态栏显示
	/// </summary>
	/// <param name="f"></param>
	int addStatusCallback(std::shared_ptr<std::function<void(StatusUi, StatusType)>> fun) {
		_status_funs.emplace_back(fun);
		return (int)_status_funs.size() - 1;
	}

	/// <summary>
	/// 移除接收数据的回调函数
	/// </summary>
	/// <param name="index"></param>
	void removeStatusCallback(std::shared_ptr < std::function<void(StatusUi, StatusType)>> fun) {
		auto it = std::find(_status_funs.begin(), _status_funs.end(), fun);
		if (it != _status_funs.end()) {
			_status_funs.erase(it);
		}
	}

	void clearCache();

private:
	/// <summary>
	/// 解析串口发送的数据
	/// </summary>
	/// <param name="data"></param>
	/// <returns></returns>
	bool analysisData(const std::string& data);
	void sub00(const uint8_t* sub_address, int& index);
	void sub03(const uint8_t* sub_address, int& index);
	int sub04(const uint8_t* sub_address, int& index, int len);
	void sub05(const uint8_t* sub_address, int& index);
	void sub06(const uint8_t* sub_address, int& index);
	void sub09(const uint8_t* sub_address, int& index);
	void sub0A(const uint8_t* sub_address, int& index);
	void sub0B(const uint8_t* sub_address, int& index);
	void sub0C(const uint8_t* sub_address, int& index);
	void sub0D_10(const uint8_t* sub_address, int& index);
	void sub20(const uint8_t* sub_address, int& index);
	void sub54(const uint8_t* sub_address, int& index);
	void sub90(const uint8_t* sub_address, int& index);
	void subE5(const uint8_t* sub_address, int& index);
	void subF0(const uint8_t* sub_address, int& index);
	void AG041TypeVG(const uint8_t* data_address);
	void AG041TypeZT(const uint8_t* data_address);
	void AG041TypeSC(const uint8_t* data_address);
	uint32_t CheckProtocol(const uint8_t *dat, uint32_t len);
	/* add AB 54 8B protocol */
	uint16_t checkCrc16(const uint8_t* data , const uint32_t len);
	void subAB548B(const uint8_t* sub_address, int& index);
private:
	std::vector<std::shared_ptr<std::function<void(void*)>>> _gnss_funs;//考虑安全问题，暂时给出副本(避免在回调函数内部被修改)
	std::vector<std::shared_ptr<std::function<void(void*)>>> _loosely_funs;//考虑安全问题，暂时给出副本(避免在回调函数内部被修改)
	std::vector<std::shared_ptr<std::function<void(void*)>>> _poll_funs;//考虑安全问题，暂时给出副本(避免在回调函数内部被修改)
	std::vector<std::shared_ptr<std::function<void(StatusUi, StatusType)>>> _status_funs;//状态回调函数
	std::string _receive_data;//解析自定义数据
	std::map<uint8_t, int> _type_datalen;//解析数据type对应单条数据长度映射

	GnssStruct _gnss_data;//GNSS数据
	LooselyDataStruct _loosely_data;//LOOSELY数据
	PollingStruct _poll;//车轮相关
	uint16_t _wheel_num = 0;//车轮基数
	uint16_t _rtk_age = 0;//RTKAGE
	uint16_t _angle_diffuse_num = 0;//姿态发散
	uint16_t _bias_diffuse_num = 0;//零偏发散
	int _num54 = 0;
	int _num54_all = 0;
	std::string _datas_src;
};

#endif
