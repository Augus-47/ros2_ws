#include "imu_node/protocol_asensing.h"
// #include <sstream>
// #include <iomanip>
#include "imu_node/common.h"
// #include "log.h"
#include <math.h>
#include <time.h>
#include <iostream>
#include "imu_node/TinyLog.h"
using namespace std;

enum ERROR_CODE
{
	NO_ERR,
	POLL_TIMEOUT,            //Polling toggle bit failed
	VERIFY_WRITE,            // Verifying write to flash failed
	INVALID_SECTOR,           //Invalid Sector
	INVALID_BLOCK,           //Invalid Block
	UNKNOWN_COMMAND,     //Unknown Command
	PROCESS_COMMAND_ERR, // Processing command
	NOT_READ_ERROR,          //Could not read memory from target
	DRV_NOTAT_BREAK,     //The drive was not at AFP_BreakReady
	BUFFER_IS_NULL,          //Could not allocate storage for the buffer
	NO_ACCESS_SECTOR,        //Cannot access the sector( could be locked or something is stored there that should not be touched )
	NUM_ERROR_CODES,
	UART_RX_FULL,            //Lost one byte when the RX circular buffer is full.
	UART_TX_FULL,            //When TX circular buffer is full, push one byte will be lost
	UART_RX_BUF_EMPTY,      //Try to pop one byte from empty RX circular buffer
	OUT_OF_FLASH_NUM,       //Out of flash number (0-2)
	FLASH_GET_CODE_FAIL,    //Check codes of flash devices failed
	FLASH_COMPARE_ERROR,     //Compare read/write flash error
	UNDEFINED_URX_STAT,      //Undefined uart rx buf status
	UNDEFINED_INS,           //Undefined instruction
	FAIL_ON_MALLOC,          //Failed on malloc()
	CHECK_SUM_ERR,           //Check sum error, VR101 protocol
	TOO_MUCH_DATA,           //Too much data to print in a data rate period
	OUT_OF_16BIT_RANGE,      //Out of 16bit range
	MATRIX_NO_INVERSE,       //Matrix no inverse
	WRITE_BACK_FLASH,        //Try to write to a Flash address that have been written
	SDRAM_TEST_ERR,          //SDRAM self test error
	COMP_SELFCHECK_FAIL,     //Compass self check fail
	UNFINISHED_HMC5883SEQ,    //Unfinished HMC5883 continous read sequence while starting a new one
	SENSOR_SATURATION,       //Sensor saturation
	GYRO_BIT_ERR,             //Gyro Built-in-test error
	CALIB_MAG2D_LARGE_INCLIN, //too large pitch or roll angle during 2D magnetometer calib
	ACCEL_INITIAL_CHECK_ERR,  //Accel initial check error
	GYRO_INITIAL_CHECK_ERR    //gyroscope initial check error
};

ProtocolAsensing::ProtocolAsensing() :_gnss_data(GnssStruct())
, _loosely_data(LooselyDataStruct()), _poll(PollingStruct())
{
	_type_datalen.insert(std::pair<uint8_t, int>(0, 9));
	_type_datalen.insert(std::pair<uint8_t, int>(3, 71));
	_type_datalen.insert(std::pair<uint8_t, int>(4, 30));
	_type_datalen.insert(std::pair<uint8_t, int>(5, 39));
	_type_datalen.insert(std::pair<uint8_t, int>(6, 21));
	_type_datalen.insert(std::pair<uint8_t, int>(9, 76));
	_type_datalen.insert(std::pair<uint8_t, int>(0x0a, 34));
	_type_datalen.insert(std::pair<uint8_t, int>(0x0b, 63));
	_type_datalen.insert(std::pair<uint8_t, int>(0x0c, 11));
	_type_datalen.insert(std::pair<uint8_t, int>(0x0d, 69));
	_type_datalen.insert(std::pair<uint8_t, int>(0x10, 69));
	_type_datalen.insert(std::pair<uint8_t, int>(0x20, 34));
	_type_datalen.insert(std::pair<uint8_t, int>(0x54, 38));
	/* 2023.06.12 ligao:add AB 54 8B protocol length:41 */
	_type_datalen.insert(std::pair<uint8_t, int>(0x8B, 41));
	_type_datalen.insert(std::pair<uint8_t, int>(0x90, 4));
	_type_datalen.insert(std::pair<uint8_t, int>(0xE5, 13));
	_type_datalen.insert(std::pair<uint8_t, int>(0xF0, 115));
}


ProtocolAsensing::~ProtocolAsensing()
{
	//AGLOG("num:({0}),data:{1}", _datas_src.size(), _datas_src);
}

void ProtocolAsensing::addData(const std::string& data)
{
	analysisData(data);
}

void ProtocolAsensing::updateGnss(void* gnss)
{
	_gnss_data = *((GnssStruct*)gnss);
	for (auto fun : _gnss_funs)
	{
		if (nullptr != fun.get())
		{
			(*(fun.get()))(gnss);
		}
	}
}

void ProtocolAsensing::updateIns(void* loosely)
{
	_loosely_data = *((LooselyDataStruct*)loosely);
	for (auto fun : _loosely_funs)
	{
		if (nullptr != fun.get())
		{
			(*(fun.get()))(loosely);
		}
	}
}
uint16_t ProtocolAsensing::checkCrc16(const uint8_t* data , const uint32_t len)
{
	uint16_t wCRCin = 0xFFFFu;
	uint16_t wCPoly = 0x1021u;
	uint8_t wChar = 0u;
	uint32_t i = 0u;
	uint32_t j = 0u;

	for( i=0u; i < len; i++)
	{
		wChar = data[i];
		wCRCin ^= (wChar << 8);

		for( j = 0u; j <8; j++)
		{
			if(wCRCin & (uint16_t)0x8000u)
			{
				wCRCin = (uint16_t)((uint16_t)(wCRCin << 1) ^ wCPoly);
			}else
			{
				wCRCin = (uint16_t)(wCRCin << 1);
			}
		}
	}

	return (wCRCin);
}

uint32_t ProtocolAsensing::CheckProtocol(const uint8_t *dat, uint32_t len)
{
	uint8_t check_sum = 0;
	static uint32_t protocolLen = 0;
	int i;
	for (i = 0; i < (len-1); ++i)
	{
		check_sum ^= dat[i];
		if(check_sum == dat[i+1])
		{
			if(i >= 30)
			{
				static uint32_t protocolCnt = 0;
				static bool getLength = false;
				if((i != 30) && (i != 32) && (i != 42))
				{
					/*is not right protocol length*/
					continue;
				}
				else
				{
					/* witch protocol is? DZT:44 VG:34 SC:32?*/
					if(!getLength || (protocolCnt > 10))
					{
						/* first time get protocol length */
						getLength = true;	
						protocolLen = i + 2;
						/* change protocol length */
						if(_type_datalen.find(4) != _type_datalen.end())
						{
							_type_datalen[4] = protocolLen;
						}
						protocolCnt = 0;
					}

					if(protocolLen != (i + 2) )
					{
						/* data check in middle pos,
						 * if protocolCnt > 10 ,change protocol length
						 * else ,clean protocolCnt , return protocolLen*/
						protocolCnt++;
						continue;
					}
					else{
						protocolCnt = 0;
						return protocolLen;
					}
				}
			}
			else{
				/*less than 30 , continue*/
				continue;
			}
		}
	}
	return 0;
}
void ProtocolAsensing::AG041TypeZT(const uint8_t* data_address)
{
	int16_t middle;
	uint16_t Info = 0;
	int start_index = 0;

	memset(_loosely_data.gps_message,0,sizeof(_loosely_data.gps_message));
	_loosely_data.ver_pos = 0x00;
	_loosely_data.ver_vel = 0x00;
	_loosely_data.info_byte = 0x00;
	_loosely_data.ag041_type = 0x01;
	_loosely_data.gps_heading_status = 0x00;
	_loosely_data.ptype = 0x00;
	_loosely_data.pdata = 0x00;
	//cout<<"BDDB041"<<endl;
	_loosely_data.type = data_address[start_index];

	start_index++;

	//DataValidLED.BackColor = Color.Lime
	//G_IS_START_ANALYSIS_SUB = true;
	middle = toValue<int16_t>(data_address, start_index);// toValue<int16_t>(data_address, start_index);
	_loosely_data.roll = middle * 360.0 / 32768;
	//VRRollTextBox.Text = Roll
	middle = toValue<int16_t>(data_address, start_index);//toValue<int16_t>(data_address, start_index);
	_loosely_data.pitch = middle * 360.0 / 32768;
	//VRPitchTextBox.Text = Pitch
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.yaw = middle * 360.0 / 32768;
	//VRYawTextBox.Text = Yaw
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gx = middle * ag041_param_dat.Grange / 32768;
	//VRGxTextBox.Text = Gx
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gy = middle * ag041_param_dat.Grange / 32768;
	//VRGyTextBox.Text = Gy
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gz = middle * ag041_param_dat.Grange / 32768;
	//VRGzTextBox.Text = Gz
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ax = middle * ag041_param_dat.Arange / 32768;
	//VRAxTextBox.Text = Ax
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ay = middle * ag041_param_dat.Arange / 32768;
	//VRAyTextBox.Text = Ay
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.az = middle * ag041_param_dat.Arange / 32768;
	//VRAzTextBox.Text = Az

	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.temp = middle * 200.0 / 32768;
	//TimeStamp
	uint32_t middle2;
	middle2 = toValue<uint32_t>(data_address, start_index); //toValue<uint32_t>(data_address, start_index);//
	_loosely_data.ts = middle2 / 100.0;

	memcpy(_loosely_data.gps_message,(uint8_t*)&data_address[start_index],14);
	start_index+=14;

	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.info_byte = middle;

	for (auto fun : _loosely_funs)
	{
		if (nullptr != fun.get())
		{
			(*fun.get())((void*)(&_loosely_data));
		}
	}
}
void ProtocolAsensing::AG041TypeVG(const uint8_t* data_address)
{
	int16_t middle;
	uint16_t Info = 0;
	int start_index = 0;

	memset(_loosely_data.gps_message,0,sizeof(_loosely_data.gps_message));
	_loosely_data.ver_pos = 0x00;
	_loosely_data.ver_vel = 0x00;
	_loosely_data.info_byte = 0x00;
	_loosely_data.ag041_type = 0x02;

	//cout<<"BDDB041"<<endl;
	_loosely_data.type = data_address[start_index];

	start_index++;

	//DataValidLED.BackColor = Color.Lime
	//G_IS_START_ANALYSIS_SUB = true;
	middle = toValue<int16_t>(data_address, start_index);// toValue<int16_t>(data_address, start_index);
	_loosely_data.roll = middle * 360.0 / 32768;
	//VRRollTextBox.Text = Roll
	middle = toValue<int16_t>(data_address, start_index);//toValue<int16_t>(data_address, start_index);
	_loosely_data.pitch = middle * 360.0 / 32768;
	//VRPitchTextBox.Text = Pitch
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.yaw = middle * 360.0 / 32768;
	//VRYawTextBox.Text = Yaw
	middle = toValue<int8_t>(data_address, start_index);
	_loosely_data.gps_heading_status = middle;
	//if (middle > 0)
	//	Debug0 = middle * 10 / 32768;

	middle = toValue<int8_t>(data_address, start_index);
	_loosely_data.ptype = middle;

	//Debug1
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.pdata = middle;
	//if (middle > 0)
	//	Debug1 = middle * 20 / 32768;

	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gx = middle * ag041_param_dat.Grange / 32768;
	//VRGxTextBox.Text = Gx
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gy = middle * ag041_param_dat.Grange / 32768;
	//VRGyTextBox.Text = Gy
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gz = middle * ag041_param_dat.Grange / 32768;
	//VRGzTextBox.Text = Gz
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ax = middle * ag041_param_dat.Arange / 32768;
	//VRAxTextBox.Text = Ax
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ay = middle * ag041_param_dat.Arange / 32768;
	//VRAyTextBox.Text = Ay
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.az = middle * ag041_param_dat.Arange / 32768;
	//VRAzTextBox.Text = Az

	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.temp = middle * 200.0 / 32768;
	//TimeStamp
	uint32_t middle2;
	middle2 = toValue<uint32_t>(data_address, start_index); //toValue<uint32_t>(data_address, start_index);//
	_loosely_data.ts = middle2 / 100.0;
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.info_byte = middle2;

	for (auto fun : _loosely_funs)
	{
		if (nullptr != fun.get())
		{
			(*fun.get())((void*)(&_loosely_data));
		}
	}
}
void ProtocolAsensing::AG041TypeSC(const uint8_t* data_address)
{
	int16_t middle;
	uint16_t Info = 0;
	int start_index = 0;

	memset(_loosely_data.gps_message,0,sizeof(_loosely_data.gps_message));
	_loosely_data.ver_pos = 0x00;
	_loosely_data.ver_vel = 0x00;
	_loosely_data.info_byte = 0x00;
	_loosely_data.ag041_type = 0x03;
	_loosely_data.gps_heading_status = 0x00;
	_loosely_data.ptype = 0x00;
	_loosely_data.pdata = 0x00;
	//cout<<"BDDB041"<<endl;
	_loosely_data.type = data_address[start_index];

	start_index++;

	//DataValidLED.BackColor = Color.Lime
	//G_IS_START_ANALYSIS_SUB = true;
	middle = toValue<int16_t>(data_address, start_index);// toValue<int16_t>(data_address, start_index);
	_loosely_data.roll = middle * 360.0 / 32768;
	//VRRollTextBox.Text = Roll
	middle = toValue<int16_t>(data_address, start_index);//toValue<int16_t>(data_address, start_index);
	_loosely_data.pitch = middle * 360.0 / 32768;
	//VRPitchTextBox.Text = Pitch
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.yaw = middle * 360.0 / 32768;
	//VRYawTextBox.Text = Yaw
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ver_pos = middle * 20.0 / 32768;

	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ver_vel = middle * 10.0 / 32768;

	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gx = middle * ag041_param_dat.Grange / 32768;
	//VRGxTextBox.Text = Gx
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gy = middle * ag041_param_dat.Grange / 32768;
	//VRGyTextBox.Text = Gy
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.gz = middle * ag041_param_dat.Grange / 32768;
	//VRGzTextBox.Text = Gz
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ax = middle * ag041_param_dat.Arange / 32768;
	//VRAxTextBox.Text = Ax
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.ay = middle * ag041_param_dat.Arange / 32768;
	//VRAyTextBox.Text = Ay
	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.az = middle * ag041_param_dat.Arange / 32768;
	//VRAzTextBox.Text = Az

	middle = toValue<int16_t>(data_address, start_index);
	_loosely_data.temp = middle * 200.0 / 32768;
	//TimeStamp
	uint32_t middle2;
	middle2 = toValue<uint32_t>(data_address, start_index); //toValue<uint32_t>(data_address, start_index);//
	_loosely_data.ts = middle2 / 100.0;

	for (auto fun : _loosely_funs)
	{
		if (nullptr != fun.get())
		{
			(*fun.get())((void*)(&_loosely_data));
		}
	}
}
void ProtocolAsensing::clearCache()
{
	_receive_data.erase(_receive_data.begin(), _receive_data.end());
	memset(&_loosely_data, 0, sizeof(_loosely_data));
	memset(&_gnss_data, 0, sizeof(_gnss_data));
}

bool ProtocolAsensing::analysisData(const std::string& data)
{
	_receive_data += data;
	//解析自定义协议数据
	int start_index = 0;//当前读取到的字节下标
	const uint8_t* data_address = (uint8_t*)_receive_data.c_str();
	bool is_exit_while = false;

	while (start_index < (int)_receive_data.size() - 11 && !is_exit_while)
	{
		if (0xDB == data_address[start_index + 1] && 0xBD == data_address[start_index])
		{
			const uint8_t* sub_address = data_address + start_index;
			uint8_t packet_type = sub_address[2];

			if (_type_datalen.find(packet_type) == _type_datalen.end())
			{
				if (start_index < _receive_data.size() - 2)
				{
					start_index += 2;
				}
				else
				{
					is_exit_while = true;
				}
			}
			else
			{
				_loosely_data.type = packet_type;
				_gnss_data.type = packet_type;

				if (_receive_data.size() - start_index - 1 > _type_datalen[packet_type])
				{
					switch (packet_type)
					{
					case 0:
						sub00(sub_address, start_index);
						break;
					case 0x03:
						sub03(sub_address, start_index);
						break;
					case 0x04:
						if(sub04(sub_address, start_index,(int)_receive_data.size() - (start_index))!=0)
						{
							is_exit_while = true;
						}
						break;
					case 0x05:
						sub05(sub_address, start_index);
						break;
					case 0x06:
						sub06(sub_address, start_index);
						break;
					case 0x09:
						sub09(sub_address, start_index);
						break;
					case 0x0A:
						sub0A(sub_address, start_index);
						break;
					case 0x0B:
						sub0B(sub_address, start_index);
						break;
					case 0x0C:
						sub0C(sub_address, start_index);
						break;
					case 0x0D:
					case 0x10:
						sub0D_10(sub_address, start_index);
						break;
					case 0x20:
						sub20(sub_address, start_index);
						break;
					case 0x54: //蔚来协议
						sub54(sub_address, start_index);
						break;
					case 0x90:
						sub90(sub_address, start_index);
						break;
					case 0xE5:
						subE5(sub_address, start_index);
						break;
					case 0xF0:
						sub0D_10(sub_address, start_index);
						break;
					default:
						if (start_index < _receive_data.size() - 1)
						{
							start_index++;
						}
						break;
					}
				}
				else
				{
					is_exit_while = true;
				}
			}
		}
		else if(0x54 == data_address[start_index + 1] && 0xAB == data_address[start_index])
		{
			const uint8_t* sub_address = data_address + start_index;
			uint8_t packet_type = sub_address[2];

			if (_type_datalen.find(packet_type) == _type_datalen.end())
			{
				if (start_index < _receive_data.size() - 2)
				{
					start_index += 2;
				}
				else
				{
					is_exit_while = true;
				}
			}
			else
			{
				_loosely_data.type = packet_type;
				if (_receive_data.size() - start_index - 1 > _type_datalen[packet_type])
				{
					subAB548B(sub_address, start_index);
				}
				else
				{
					is_exit_while = true;
				}
			}
		}
		else
		{
			if (start_index < _receive_data.size() - 1)
			{
				start_index++;
			}
			else
			{
				is_exit_while = true;
			}
		}
	}//end while

	if (start_index > 0 && _receive_data.size() > start_index)
	{
		std::string hex_data_rec;
		for (int k = 0; k < start_index; ++k)
		{
			char buf[3];
			sprintf(buf, "%02X", (uint8_t)_receive_data[k]);
			hex_data_rec += buf;
		}
		//去掉已遍历过的数据
		_receive_data.erase(_receive_data.begin(), _receive_data.begin() + start_index);
	}
	else if (start_index > 0 && _receive_data.size() == start_index)
	{
		_receive_data.clear();
	}

	return true;
}

void ProtocolAsensing::sub00(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	//校验数据正确性
	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		int16_t middle;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.roll = middle * 360.0 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.pitch = middle * 360.0 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.yaw = middle * 360.0 / 32768;

		index += sub_index + 1;
		//更新roll, pitch, yaw
		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub03(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < 71; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[71])
	{
		int16_t middle, temp[3];
		int32_t middle2;
		uint16_t middleu;
		uint32_t middleu2;

		sub_index = 3;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.roll = middle * 360.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.pitch = middle * 360.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.yaw = middle * 360.0 / 32768;

		// 陀螺仪x,y,z轴
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gx = middle * 300.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gy = middle * 300.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gz = middle * 300.0 / 32768;

		// 加速度x,y,z轴
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ax = middle * 2.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ay = middle * 2.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.az = middle * 2.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.temp = middle * 200.0 / 32768;

		//bgx,y,z
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.bgx = middle * 1000.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.bgy = middle * 1000.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.bgz = middle * 1000.0 / 32768;

		//bax,y,z
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.bax = middle * 50.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.bay = middle * 50.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.baz = middle * 50.0 / 32768;

		//lat,lon,h
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_loosely_data.lat = middle2 * 0.0000001;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_loosely_data.lon = middle2 * 0.0000001;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_loosely_data.h = middle2 * 0.001;

		//vn, ve, vd
		middle2 = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.vn = middle * 100.0 / 32768;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_loosely_data.ve = middle * 100.0 / 32768;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_loosely_data.vd = middle * 100.0 / 32768;

		_loosely_data.IMUState = toValue<uint8_t>(sub_address, sub_index);

		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_loosely_data.LEKFState = middleu2;

		middleu = toValue<uint16_t>(sub_address, sub_index);
		_loosely_data.gpsState = middleu;

		temp[0] = toValue<int16_t>(sub_address, sub_index);
		temp[1] = toValue<int16_t>(sub_address, sub_index);
		temp[2] = toValue<int16_t>(sub_address, sub_index);

		middle2 = toValue<int32_t>(sub_address, sub_index);
		_loosely_data.ts = middle2 / 4000.0;

		uint8_t sub_type = toValue<uint8_t>(sub_address, sub_index);
		switch (sub_type)
		{
		case 0:
			_loosely_data.plat = exp(temp[0] * 0.01);
			_loosely_data.plon = exp(temp[1] * 0.01);
			_loosely_data.ph = exp(temp[2] * 0.01);
			break;
		case 1:
			_loosely_data.plat = exp(temp[0] * 0.01);
			_loosely_data.plon = exp(temp[1] * 0.01);
			_loosely_data.ph = exp(temp[2] * 0.01);
			break;
		case 2:
			_loosely_data.plat = exp(temp[0] * 0.01);
			_loosely_data.plon = exp(temp[1] * 0.01);
			_loosely_data.ph = exp(temp[2] * 0.01);
			break;
		case 3:
			_loosely_data.plat = exp(temp[0] * 0.01);
			_loosely_data.plon = exp(temp[1] * 0.01);
			_loosely_data.ph = exp(temp[2] * 0.01);
			break;
		default:
			break;
		}

		index += sub_index + 1;

		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

int ProtocolAsensing::sub04(const uint8_t* sub_address, int& index , int len)
{
#if 0
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i) //63？
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		int16_t middle;
		uint16_t debug0, debug1, info = 0;
		uint32_t middle2;

		sub_index = 3;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.roll = middle * 360.0 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.pitch = middle * 360.0 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.yaw = middle * 360.0 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		if (middle > 0)
			debug0 = middle * 10 / 32768;

		//Debug1
		middle = toValue<int16_t>(sub_address, sub_index);
		if (middle > 0)
			debug1 = middle * 20 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gx = middle * grange_scale / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gy = middle * grange_scale / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gz = middle * grange_scale / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ax = middle * arange_scale / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ay = middle * arange_scale / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.az = middle * arange_scale / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.temp = middle * 200.0 / 32768;
		//TimeStamp

		middle2 = toValue<uint32_t>(sub_address, sub_index);
		_loosely_data.ts = middle2 / 1000.0;

		middle = toValue<int16_t>(sub_address, sub_index);

		index += sub_index + 1;

		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}

		if (middle > 0)
		{
			info = middle;
		}

		//Analyse InfoByte
		std::string imu_dev_model_type[5] = { "IMU800", "IMU610", "WS601", "MiniIMU", "Unknown Model" };
		std::string imu_dev_model_now, soft_version, hw_version;

		uint16_t granges[6] = { 75, 150, 200, 300, 573, 900 };
		uint16_t aranges[5] = { 2, 3, 5, 300, 12 };
		std::string grange, arange;
		std::string sys_warn, sys_error, sys_snr_sat;

		uint8_t model_segment, soft_segment, hw_segment;
		uint8_t grange_segment, arange_segment;
		uint8_t sys_warn_segment, sys_err_segment, sys_snr_segment;

		model_segment = info >> 13;
		soft_segment = (info & 0x1C00) >> 10;
		hw_segment = (info & 0x300) >> 8;
		grange_segment = (info & 0xE0) >> 5;
		arange_segment = (info & 0x18) >> 3;
		sys_warn_segment = (info & 0x4) >> 2;
		sys_err_segment = (info & 0x2) >> 1;
		sys_snr_segment = (info & 0x1);

		imu_dev_model_now = model_segment > 4 ? imu_dev_model_type[4] : imu_dev_model_type[model_segment];

		soft_version = "Ver " + std::to_string(soft_segment);

		if (hw_segment < 4)
		{
			std::string s(1, ('A' + hw_segment));
			hw_version = "Rev. " + s;
		}

		if (grange_segment < 5)
		{
			grange_scale = granges[grange_segment];
			grange = std::to_string(grange_scale) + "deg/s";
		}

		if (arange_segment < 6)
		{
			arange_scale = aranges[arange_segment];
			arange = std::to_string(arange_scale) + "g";
		}

		sys_warn = sys_warn_segment == 1 ? "R" : "G";
		sys_error = sys_err_segment == 1 ? "R" : "G";
		sys_snr_sat = sys_snr_segment == 1 ? "R" : "G";
	}
	else
	{
		index += 4;
	}
#else
	static uint8_t	ag041_type = 0, ag041_type_tmp = 0;
	static uint8_t	ag_type_cnt = 0;
	//if(ag041_type==0)
	{
		uint8_t ag041_type_get = 0;
		uint32_t Protocol_len = 0;
		static uint8_t exit_cnt = 0;
		//check protocol
		Protocol_len = CheckProtocol((const uint8_t*)&sub_address[index],len);
		//printf("%d,%x,%d\r\n",len,sub_address[index],Protocol_len);
		if(Protocol_len==44)
		{
			ag041_type_get = 0x01;
			AG041TypeZT((const uint8_t*)&sub_address[2]);
		}
		else if(Protocol_len==34)
		{
			ag041_type_get = 0x02;
			AG041TypeVG((const uint8_t*)&sub_address[2]);
		}
		else if(Protocol_len==32)
		{
			ag041_type_get = 0x03;
			AG041TypeSC((const uint8_t*)&sub_address[2]);
		}
		//printf("Protocol_len:%d\r\n",Protocol_len);
		if(Protocol_len!=0)
		{
			exit_cnt = 0;
			// if(ag041_type_tmp == ag041_type_get)
			// {
			// 	ag_type_cnt++;
			// }
			// else
			// {
			// 	ag_type_cnt = 0;
			// }
			// ag041_type_tmp = ag041_type_get;

			// if(ag_type_cnt>10)
			// {
			// 	ag041_type = ag041_type_tmp;
			// }
			index += Protocol_len;
		}
		else
		{
			exit_cnt++;
			if(exit_cnt >= 3)
			{
				 index += 1;
				 exit_cnt = 0;
				 static uint64_t errcnt = 0;
				 errcnt++;
			}
			return 1;
		}
	}
#endif
	return 0;
}

void ProtocolAsensing::sub05(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		int16_t middle;
		float roll, pitch, yaw;
		float gx, gy, gz;
		float ax, ay, az;

		//Following items are added for imu610
		int16_t magx = 0, magy = 0, magz = 0;
		uint16_t debug0, debug1 = 0, info = 0;
		float Temperature;
		float TimeStamp;

		sub_index = 3;

		middle = toValue<int16_t>(sub_address, sub_index);
		roll = middle * 360.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		pitch = middle * 360.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		yaw = middle * 360.0 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		if (middle > 0)
			debug0 = middle * 10.0 / 32768;

		middle = toValue<int16_t>(sub_address, sub_index);
		if (middle > 1)
			debug1 = middle * 20.0 / 32768;

		//gx,gy,gz
		middle = toValue<int16_t>(sub_address, sub_index);
		gx = middle * grange_scale / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		gy = middle * grange_scale / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		gz = middle * grange_scale / 32768;

		//ax,ay,az
		middle = toValue<int16_t>(sub_address, sub_index);
		ax = middle * arange_scale / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		ay = middle * arange_scale / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		az = middle * arange_scale / 32768;

		//magx, magy, magz
		magx = toValue<int16_t>(sub_address, sub_index);
		magy = toValue<int16_t>(sub_address, sub_index);
		magz = toValue<int16_t>(sub_address, sub_index);

		//Temperature
		middle = toValue<int16_t>(sub_address, sub_index);
		Temperature = middle * 200.0 / 32768.0;
		//TimeStamp
		uint32_t middle2;
		middle2 = toValue<uint32_t>(sub_address, sub_index);
		TimeStamp = middle2 / 1000.0;

		middle = toValue<int16_t>(sub_address, sub_index);
		if (middle > 0)
			info = middle;

		index += sub_index + 1;

		float abs_mag_2d = sqrt(magx ^ 2 + magy ^ 2);

		//Analyse InfoByte
		std::string imu_dev_model_type[5] = { "IMU800", "IMU610", "WS601", "MiniIMU", "Unknown Model" };
		std::string imu_dev_model_now, soft_version, hw_version;

		uint16_t granges[6] = { 75, 150, 200, 300, 573, 900 };
		uint16_t aranges[5] = { 2, 3, 5, 300, 12 };
		std::string grange, arange;
		std::string sys_warn, sys_error, sys_snr_sat;

		uint8_t model_segment, soft_segment, hw_segment;
		uint8_t grange_segment, arange_segment;
		uint8_t sys_warn_segment, sys_err_segment, sys_snr_segment;

		model_segment = info >> 13;
		soft_segment = (info & 0x1C00) >> 10;
		hw_segment = (info & 0x300) >> 8;
		grange_segment = (info & 0xE0) >> 5;
		arange_segment = (info & 0x18) >> 3;
		sys_warn_segment = (info & 0x4) >> 2;
		sys_err_segment = (info & 0x2) >> 1;
		sys_snr_segment = (info & 0x1);

		imu_dev_model_now = model_segment > 4 ? imu_dev_model_type[4] : imu_dev_model_type[model_segment];

		soft_version = "Ver " + std::to_string(soft_segment);

		if (hw_segment < 4)
		{
			std::string s(1, ('A' + hw_segment));
			hw_version = "Rev. " + s;
		}

		if (grange_segment < 5)
		{
			grange_scale = granges[grange_segment];
			grange = std::to_string(grange_scale) + "deg/s";
		}

		if (arange_segment < 6)
		{
			arange_scale = aranges[arange_segment];
			arange = std::to_string(arange_scale) + "g";
		}

		sys_warn = sys_warn_segment == 1 ? "R" : "G";
		sys_error = sys_err_segment == 1 ? "R" : "G";
		sys_snr_sat = sys_snr_segment == 1 ? "R" : "G";
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub06(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		uint16_t gx, gy, gz;
		uint16_t ax, ay, az;
		uint16_t temp[3];

		sub_index = 3;

		gx = toValue<uint16_t>(sub_address, sub_index);
		gy = toValue<uint16_t>(sub_address, sub_index);
		gz = toValue<uint16_t>(sub_address, sub_index);

		ax = toValue<uint16_t>(sub_address, sub_index);
		ay = toValue<uint16_t>(sub_address, sub_index);
		az = toValue<uint16_t>(sub_address, sub_index);

		temp[0] = toValue<uint16_t>(sub_address, sub_index);
		temp[1] = toValue<uint16_t>(sub_address, sub_index);
		temp[2] = toValue<uint16_t>(sub_address, sub_index);

		index += sub_index + 1;
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub09(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}
	if (check_sum == sub_address[sub_data_len])
	{
		int32_t middle2;
		uint16_t middleu;
		uint32_t middleu2;

		sub_index = 3;
		//lon, lat, h
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.lon = middle2 * 0.0000001;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.lat = middle2 * 0.0000001;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.h = middle2 * 0.001;
		//vn, ve, vd
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.vn = middle2 * 0.01;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.ve = middle2 * 0.01;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.vd = middle2 * 0.01;
		//speed3d, speed2d, heading
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.Speed3D = middle2 * 0.01;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.Speed2D = middle2 * 0.01;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.heading = middle2 * 0.00001;

		//hdop, vdop
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.hDOP = middle2 * 0.001;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.vDOP = middle2 * 0.001;

		//sdop, cdop
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.sDOP = middle2 * 0.01;
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.cDOP = middle2 * 0.00001;

		_gnss_data.gpsfix = sub_address[sub_index++];
		_gnss_data.gpsfix = _gnss_data.gpsfix & 0x07;
		_gnss_data.flags = sub_address[sub_index++];
		_gnss_data.flags = _gnss_data.flags & 0x03;
		_gnss_data.numSV = sub_address[sub_index++];
		middleu = toValue<uint16_t>(sub_address, sub_index);
		_gnss_data.pDOP = middleu * 0.01;

		middleu = toValue<uint16_t>(sub_address, sub_index);
		_gnss_data.year = middleu;
		_gnss_data.month = sub_address[sub_index++];
		_gnss_data.day = sub_address[sub_index++];
		_gnss_data.hour = sub_address[sub_index++];

		// 北京时间
		if (_gnss_data.hour < 16)
			_gnss_data.hour = _gnss_data.hour + 8;
		else if (_gnss_data.hour < 24)
			_gnss_data.hour = _gnss_data.hour + 8 - 24;
		_gnss_data.min = sub_address[sub_index++];
		_gnss_data.sec = sub_address[sub_index++];

		_gnss_data.msg = sub_address[sub_index++];

		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_gnss_data.iTowPosllh = middleu2 * 0.001;
		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_gnss_data.iTowVelned = middleu2 * 0.001;

		_gnss_data.ts = _loosely_data.ts;

		index += sub_index + 1;

		for (auto fun : _gnss_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub0A(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];
	_loosely_data.type = packet_type;

	for (int i = 0; i < sub_data_len - 1; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len - 1])
	{
		int16_t middle;

		sub_index = 3;

		// gyro
		_loosely_data.gx = toValue<float>(sub_address, sub_index);
		_loosely_data.gy = toValue<float>(sub_address, sub_index);
		_loosely_data.gz = toValue<float>(sub_address, sub_index);

		// acc
		_loosely_data.ax = toValue<float>(sub_address, sub_index);
		_loosely_data.ay = toValue<float>(sub_address, sub_index);
		_loosely_data.az = toValue<float>(sub_address, sub_index);

		// temp
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.temp = middle * 200.0 / 32768;

		sub_index++;
		_loosely_data.ptype = toValue<uint8_t>(sub_address, sub_index);
		_loosely_data.frame_count = toValue<uint16_t>(sub_address, sub_index);

		index += sub_data_len;
		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub0B(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len - 6; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len - 6])
	{
		int16_t middle;
		int32_t middle2;
		uint32_t middleu2;
		uint16_t middleu;
		int16_t temp[3];

		sub_index = 3;
		// roll pitch yaw
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.roll = middle * 360.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.pitch = middle * 360.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.yaw = middle * 360.0 / 32768;

		// gyro
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gx = middle * 300.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gy = middle * 300.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gz = middle * 300.0 / 32768;

		// acc
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ax = middle * 12.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ay = middle * 12.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.az = middle * 12.0 / 32768;

		// lat lon h
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_loosely_data.lat = middle2 * 0.0000001;
		middle2 = toValue<int32_t>(sub_address, sub_index);//
		_loosely_data.lon = middle2 * 0.0000001;
		middle2 = toValue<int32_t>(sub_address, sub_index);//
		_loosely_data.h = middle2 * 0.001;

		// speed
		middle = toValue<int16_t>(sub_address, sub_index);//
		_loosely_data.vn = middle * 100.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);//
		_loosely_data.ve = middle * 100.0 / 32768;
		middle = toValue<int16_t>(sub_address, sub_index);//
		_loosely_data.vd = middle * 100.0 / 32768;

		// imustate
		_loosely_data.IMUState = sub_address[sub_index++];

		//LooselyEKF.state
		middleu2 = toValue<uint32_t>(sub_address, sub_index);//toValue<uint32_t>(sub_address, sub_index);//
		_loosely_data.LEKFState = middleu2;

		//gps state
		middleu = toValue<uint16_t>(sub_address, sub_index);//System.BitConverter.ToUInt16(inBytes, iProAnalysis)
		_loosely_data.gpsState = middleu;

		//p
		temp[0] = toValue<int16_t>(sub_address, sub_index);//
		temp[1] = toValue<int16_t>(sub_address, sub_index);//
		temp[2] = toValue<int16_t>(sub_address, sub_index);//

		//ts
		middle2 = toValue<uint32_t>(sub_address, sub_index);//
		_loosely_data.ts = middle2 / 4000.0;
		if (fabs(_loosely_data.ts - 374824.843750) < 1e-7)
		{
			int m = 0;
		}

		//PrnData3_PType
		switch (sub_address[sub_index++]) {
		case 0:
			_loosely_data.plat = exp(temp[0] * 0.01);
			_loosely_data.plon = exp(temp[1] * 0.01);
			_loosely_data.ph = exp(temp[2] * 0.01);
			break;
		case 1:
			_loosely_data.pvn = exp(temp[0] * 0.01);
			_loosely_data.pve = exp(temp[1] * 0.01);
			_loosely_data.pvd = exp(temp[2] * 0.01);
			break;
		case 2:
			_loosely_data.proll = exp(temp[0] * 0.01);
			_loosely_data.ppitch = exp(temp[1] * 0.01);
			_loosely_data.pyaw = exp(temp[2] * 0.01);
			break;
		case 3:
			_loosely_data.pbax = exp(temp[0] * 0.01);
			_loosely_data.pbay = exp(temp[1] * 0.01);
			_loosely_data.pbaz = exp(temp[2] * 0.01);
			break;
		case 4:
			_loosely_data.pbgx = exp(temp[0] * 0.01);
			_loosely_data.pbgy = exp(temp[1] * 0.01);
			_loosely_data.pbgz = exp(temp[2] * 0.01);
			break;
		case 5:
			_loosely_data.xlat = temp[0] * 0.001;
			_loosely_data.xlon = temp[1] * 0.001;
			_loosely_data.xh = temp[2] * 0.001;
			break;
		case 6:
			_loosely_data.xvn = temp[0] / 32768.0;
			_loosely_data.xve = temp[1] / 32768.0;
			_loosely_data.xvd = temp[2] / 32768.0;
			break;
		case 7:
			_loosely_data.xroll = temp[0] / 32768.0 * 360 / 100;
			_loosely_data.xpitch = temp[1] / 32768.0 * 360 / 100;
			_loosely_data.xyaw = temp[2] / 32768.0 * 360 / 100;
			break;
		case 8:
			_loosely_data.xbax = temp[0] / 32768.0 / 100 * 0.1;
			_loosely_data.xbay = temp[1] / 32768.0 / 100 * 0.1;
			_loosely_data.xbaz = temp[2] / 32768.0 / 100 * 0.1;
			break;
		case 9:
			_loosely_data.xbgx = temp[0] / 32768.0 / 100;
			_loosely_data.xbgy = temp[1] / 32768.0 / 100;
			_loosely_data.xbgz = temp[2] / 32768.0 / 100;
			break;
		case 10:
			_loosely_data.aroll = temp[0] * 360.0 / 32768;
			_loosely_data.apitch = temp[1] * 360.0 / 32768;
			_loosely_data.ayaw = temp[2] * 360.0 / 32768;
			break;
		case 11:
			_loosely_data.paroll = exp(temp[0] * 0.01);
			_loosely_data.papitch = exp(temp[1] * 0.01);
			_loosely_data.payaw = exp(temp[2] * 0.01);
			break;
		case 12:
			_loosely_data.xaroll = temp[0] / 32768.0 * 360 / 100;
			_loosely_data.xapitch = temp[1] / 32768.0 * 360 / 100;
			_loosely_data.xayaw = temp[2] / 32768.0 * 360 / 100;
			break;
		case 13:
			//_loosely_data.yawDelay = temp[0]/ 32768.0 * 360
			_loosely_data.timeGPS2IMU = temp[0];
			_loosely_data.timeIMU = temp[1] / 100.0;   // unit:ms
			_loosely_data.timeLEKF = temp[2] / 100.0; // unit:ms
			break;
		case 14:
			_loosely_data.lbbgx = temp[0] * 0.001;
			_loosely_data.lbbgy = temp[1] * 0.001;
			_loosely_data.lbbgz = temp[2] * 0.001;
			break;
		case 15:
			_loosely_data.plbbgx = exp(temp[0] * 0.01);
			_loosely_data.plbbgy = exp(temp[1] * 0.01);
			_loosely_data.plbbgz = exp(temp[2] * 0.01);
			break;
		case 16:
			_loosely_data.xlbbgx = temp[0] * 0.0001;
			_loosely_data.xlbbgy = temp[1] * 0.0001;
			_loosely_data.xlbbgz = temp[2] * 0.0001;
			break;
		case 17:
			_loosely_data.lbbcx = temp[0] * 0.001;
			_loosely_data.lbbcy = temp[1] * 0.001;
			_loosely_data.lbbcz = temp[2] * 0.001;
			break;
		case 18:
			_loosely_data.plbbcx = exp(temp[0] * 0.01);
			_loosely_data.plbbcy = exp(temp[1] * 0.01);
			_loosely_data.plbbcz = exp(temp[2] * 0.01);
			break;
		case 19:
			_loosely_data.xlbbcx = temp[0] * 0.0001;
			_loosely_data.xlbbcy = temp[1] * 0.0001;
			_loosely_data.xlbbcz = temp[2] * 0.0001;
			break;
		case 20:
			_loosely_data.bgx = temp[0] * 1000.0 / 32768;
			_loosely_data.bgy = temp[1] * 1000.0 / 32768;
			_loosely_data.bgz = temp[2] * 1000.0 / 32768;
			break;
		case 21:
			_loosely_data.bax = temp[0] * 100.0 / 32768;
			_loosely_data.bay = temp[1] * 100.0 / 32768;
			_loosely_data.baz = temp[2] * 100.0 / 32768;
			break;
		case 22:
			_loosely_data.temp = temp[0] * 200.0 / 32768;
			break;
		case 27:
			_loosely_data.kws0 = temp[0] * 0.00001;
			_loosely_data.kws1 = temp[1] * 0.00001;
			break;
		case 28:
			_loosely_data.pkws0 = exp(temp[0] * 0.01);
			_loosely_data.pkws1 = exp(temp[1] * 0.01);
			break;
		case 29:
			_loosely_data.xkws0 = temp[0] * 0.00001;
			_loosely_data.xkws1 = temp[1] * 0.00001;
			break;
		}

		if (fabs(_loosely_data.roll) > 30 || fabs(_loosely_data.pitch) > 30)
		{
			_angle_diffuse_num = (ushort)(_angle_diffuse_num + 1);
			if (_angle_diffuse_num == 50000)
			{
				_angle_diffuse_num = 100;
			}
		}
		else
		{
			_angle_diffuse_num = 0;
		}
		if (fabs(_loosely_data.bax) > 10 || fabs(_loosely_data.bay) > 10
			|| fabs(_loosely_data.baz) > 10 || fabs(_loosely_data.bgx) > 600
			|| fabs(_loosely_data.bgy) > 600 || fabs(_loosely_data.bgz) > 600)
		{
			_bias_diffuse_num = (ushort)(_bias_diffuse_num + 1);
		}
		else
		{
			_bias_diffuse_num = 0;
		}

		//新版，置信度值
		check_sum = 0;
		for (int i = 0; i < sub_data_len; ++i)
		{
			check_sum ^= sub_address[i];
		}

		uint8_t test_num = sub_address[sub_data_len];
		if (check_sum == test_num)
		{
			sub_index++;
			int32_t pos_config_value = toValue<int32_t>(sub_address, sub_index);
			_loosely_data._pos_config_value = pos_config_value / 1000.0;
			_loosely_data._pos_config_level = sub_address[sub_index++];
		}

		index += sub_index + 1;

		//AGLOG("type:{0}{1}{2}", _loosely_data.type, _loosely_data.ts, _loosely_data.lon);

		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub0C(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		uint32_t middle2;

		sub_index = 3;

		// TimeSyncIMU
		middle2 = toValue<uint32_t>(sub_address, sub_index);
		_loosely_data.TimeSyncIMU = middle2 * 0.001;

		// TimeSyncGPS
		middle2 = toValue<uint32_t>(sub_address, sub_index);
		_loosely_data.TimeSyncGPS = middle2 * 0.001;

		index += sub_index + 1;

		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub0D_10(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	// 校验数据正确性
	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		uint32_t middleu2;
		int32_t	middle2;
		int16_t middle;
		uint16_t middleu;

		sub_index = 3;

		// Lon, LonStd, Lat, LatStd, hMSL, hMSLStd
		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.lon = middle2 * 0.0000001;
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.lonstd = middle * 0.001;

		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.lat = middle2 * 0.0000001;
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.latstd = middle * 0.001;

		middle2 = toValue<int32_t>(sub_address, sub_index);
		_gnss_data.h = middle2 * 0.001;

		//   19~20
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.vDOP = middle * 0.001;

		// gpsFix  21~24
		_rtk_age = toValue<uint16_t>(sub_address, sub_index);
		sub_index -= sizeof(uint16_t);
		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_gnss_data.gpsfixNovAtel = middleu2;

		// flags  25~28
		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_gnss_data.flagsNovAtel = middleu2;

		// HSpd  29~30
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.Hspd = middle * 0.01;

		// Trk  31~32
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.Trk = middle * 0.01;

		// VSpd  33~34
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.VSpd = middle * 0.01;

		// LatencyVel  35~36
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.LatencyVel = middle * 0.001;

		// BaseLineLength  37~38
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.BaseLineLength = middle * 0.001;

		// heading  39~40
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.heading = middle * 0.01;

		// cAcc  41~42
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.cDOP = middle * 0.001;

		// pitch  43~44
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.pitch = middle * 0.001;

		// pitchStd  45~46
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.pitchStd = middle * 0.001;

		// time  47~54
		middle = toValue<int16_t>(sub_address, sub_index);
		_gnss_data.year = (middle & 63) + 2000;
		_gnss_data.month = sub_address[sub_index++];
		_gnss_data.day = sub_address[sub_index++];
		_gnss_data.hour = sub_address[sub_index++];
		_gnss_data.min = sub_address[sub_index++];
		middleu = toValue<uint16_t>(sub_address, sub_index);
		_gnss_data.sec = middleu * 0.001;

		// itow_pos  55~58
		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_gnss_data.itow_pos = middleu2 * 0.001;

		// itow_vel  59~62
		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_gnss_data.itow_vel = middleu2 * 0.001;

		// itow_heading  63~66
		middleu2 = toValue<uint32_t>(sub_address, sub_index);
		_gnss_data.itow_heading = middleu2 * 0.001;

		// gs.RecMsg
		_gnss_data.msg = sub_address[sub_index++];

		// numSV
		_gnss_data.numSV = sub_address[sub_index];
		_loosely_data.numSV = _gnss_data.numSV ;
		_gnss_data.ts = _loosely_data.ts;
		_gnss_data.isNovAtel = true;

		index += sub_index + 1;

		for (auto fun : _gnss_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_gnss_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub20(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		_poll.Shift = sub_address[21];
		sub_index = 3;

		_wheel_num = (ushort)(_wheel_num + 1);
		if (_wheel_num == 50000)
		{
			_wheel_num = 0;
		}

		if (_poll.Shift == 0x05 || _poll.Shift == 0x06
			|| _poll.Shift == 0x07 || _poll.Shift == 0x08)
		{
			//左前轮轮速 3
			uint16_t middle = toValue<uint16_t>(sub_address, sub_index);
			_poll.LFS = (float)(middle * 0.00863);
			//右前轮轮速 5
			middle = toValue<uint16_t>(sub_address, sub_index);
			_poll.RFS = (float)(middle * 0.00863);
			//左后轮轮速 12
			middle = toValue<uint16_t>(sub_address, sub_index);
			_poll.LRS = (float)(middle * 0.00863);
			//右后轮轮速 14
			middle = toValue<uint16_t>(sub_address, sub_index);
			_poll.RRS = (float)(middle * 0.00863);
		}
		else if (_poll.Shift == 0x01 || _poll.Shift == 0x02
			|| _poll.Shift == 0x03 || _poll.Shift == 0x04)
		{
			//左前轮轮速 3
			uint16_t middle = serializeData<uint16_t>(sub_address, sub_index);
			_poll.LFS = (float)(middle * 1.0 / 0.00863);
			//右前轮轮速 5
			middle = serializeData<uint16_t>(sub_address, sub_index);
			_poll.RFS = (float)(middle * 1.0 / 0.00863);
			sub_index += 6;
			//左后轮轮速 12
			middle = serializeData<uint16_t>(sub_address, sub_index);
			_poll.LRS = (float)(middle * 1.0 / 0.00863);
			//右后轮轮速 14
			middle = serializeData<uint16_t>(sub_address, sub_index);
			_poll.RRS = (float)(middle * 1.0 / 0.00863);
		}
		_poll.error = true;

		index += sub_data_len;
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub54(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len - 1; ++i)
	{
		check_sum ^= sub_address[i];
	}
	//36 37均为前面所有数异或，即37恒为0
	if (check_sum == sub_address[sub_data_len - 1] && 0 == sub_address[sub_data_len])
	{
		short temp = 0;
		uint16_t u16_temp;
		uint32_t u32_temp;
		sub_index = 3;

		temp = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gx = temp * 300.0 / 32768.0;
		temp = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gy = temp * 300.0 / 32768.0;
		temp = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.gz = temp * 300.0 / 32768.0;
		temp = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ax = temp * 12.0 / 32768.0;
		temp = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.ay = temp * 12.0 / 32768.0;
		temp = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.az = temp * 12.0 / 32768.0;
		temp = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.temp = temp * 200.0 / 32768.0;

		sub_index++; //固定值，占位
		u16_temp = toValue<uint8_t>(sub_address, sub_index);
		u16_temp |= toValue<uint8_t>(sub_address, sub_index);
		_loosely_data.status_flag = u16_temp;
		_loosely_data.frame_count = toValue<uint16_t>(sub_address, sub_index);
		_loosely_data.gps_week = toValue<uint16_t>(sub_address, sub_index);
		u32_temp = toValue<uint32_t>(sub_address, sub_index);
		_loosely_data.gps_second_of_week = u32_temp / 1000.0;

		_loosely_data.gps_year = toValue<uint16_t>(sub_address, sub_index);
		_loosely_data.gps_month = toValue<uint8_t>(sub_address, sub_index);
		_loosely_data.gps_day = toValue<uint8_t>(sub_address, sub_index);
		_loosely_data.gps_hour = toValue<uint8_t>(sub_address, sub_index);
		_loosely_data.gps_min = toValue<uint8_t>(sub_address, sub_index);
		_loosely_data.gps_sec = toValue<uint8_t>(sub_address, sub_index);
		_loosely_data.gps_msec = toValue<uint16_t>(sub_address, sub_index);

		index += sub_index + 2;	//有2个校验位所以要+2

		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
		_num54++;
		//AGLOG("type:{0}, count:{1}, check_sum:{2}", _loosely_data.type, _num54, check_sum);
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::sub90(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		sub_index = 3;
		index += sub_index + 1;
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::subE5(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}

	if (check_sum == sub_address[sub_data_len])
	{
		int32_t iCheck = 0;
		uint8_t id = 0;

		sub_index = 3;
		id = sub_address[sub_index++];
		switch (id) {
		case 0:
			iCheck = toValue<int16_t>(sub_address, sub_index);
			_loosely_data.atbxr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);
			_loosely_data.atbyr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);
			_loosely_data.atbzr = iCheck * 180.0 / 32768.0;
			break;
		case 1:
			iCheck = toValue<int16_t>(sub_address, sub_index);
			_loosely_data.ltgxr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);
			_loosely_data.ltgyr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.ltgzr = iCheck * 0.001;
			break;
		case 2:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.ltcxr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.ltcyr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.ltczr = iCheck * 0.001;
			break;
		case 3:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.atgxr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.atgyr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.atgzr = iCheck * 180.0 / 32768.0;
			break;
		case 4:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.kws1r = iCheck * 0.00001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.kws2r = iCheck * 0.00001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.kws3r = iCheck * 0.001;
			break;
		case 10:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.wheelselectr = iCheck;
			if (_loosely_data.wheelselectr == 0) {
				//SettingForm.wheelselectr.Text = "从动轮"
			}
			else if (_loosely_data.wheelselectr == 1) {
				//SettingForm.wheelselectr.Text = "驱动轮"
			}
			else {
				//SettingForm.wheelselectr.Text = _loosely_data.wheelselectr
			}
			break;

		case 11:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stater = iCheck;
			if (_loosely_data.stater == 0) {
				//SettingForm.stater.Text = "加载出厂配置"
			}
			else if (_loosely_data.stater == 6) {
				//SettingForm.stater.Text = "加载保存配置"
			}
			else if (_loosely_data.stater == 7) {
				//SettingForm.stater.Text = "加载保存配置(无轮速)"
			}
			else {
				//SettingForm.stater.Text = _loosely_data.stater
			}
		case 5:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdatbxr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdatbyr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdatbzr = iCheck * 180.0 / 32768.0;

			break;
		case 6:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdltgxr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdltgyr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdltgzr = iCheck * 0.001;

			break;
		case 7:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdltcxr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdltcyr = iCheck * 0.001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdltczr = iCheck * 0.001;

			break;
		case 8:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdatgxr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdatgyr = iCheck * 180.0 / 32768.0;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdatgzr = iCheck * 180.0 / 32768.0;

			break;
		case 9:
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdkws1r = iCheck * 0.000001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdkws2r = iCheck * 0.000001;
			iCheck = toValue<int16_t>(sub_address, sub_index);//S
			_loosely_data.stdkws3r = iCheck * 0.001;

			break;
		}

		index += sub_index + 1;

		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

void ProtocolAsensing::subF0(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t check_sum = 0;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];

	//校验数据正确性
	for (int i = 0; i < sub_data_len; ++i)
	{
		check_sum ^= sub_address[i];
	}
	if (check_sum == sub_address[sub_data_len])
	{
		uint8_t CurrentPtr;
		uint32_t Sec[16];
		uint32_t SubMs[16];
		ERROR_CODE ErrMsg[16];

		sub_index = 3;

		CurrentPtr = sub_address[sub_index++];

		for (int iPrnErrMsg = 0; iPrnErrMsg <= 15; ++iPrnErrMsg)
		{
			Sec[iPrnErrMsg] = toValue<uint32_t>(sub_address, sub_index);
			SubMs[iPrnErrMsg] = toValue<uint16_t>(sub_address, sub_index);
			ErrMsg[iPrnErrMsg] = (ERROR_CODE)sub_address[sub_index++];
		}

		std::string IMUErrLog;
		IMUErrLog = (/*vbCrLf + */"Next error: " + std::to_string(CurrentPtr));
		for (int iPrnErrMsg = 0; iPrnErrMsg <= 15; ++iPrnErrMsg) {
			IMUErrLog += (/*vbCrLf + */"Log " + std::to_string(iPrnErrMsg) + ": Sec: " + std::to_string(Sec[iPrnErrMsg])
				+ " Ticks (0.1ms): " + std::to_string(SubMs[iPrnErrMsg]) + " Error Msg: "
				+ std::to_string(ErrMsg[iPrnErrMsg]));
		}
		//AGERROR(IMUErrLog);

		index += sub_index + 1;
	}
	else
	{
		index += 4;
	}
}
void ProtocolAsensing::subAB548B(const uint8_t* sub_address, int& index)
{
	int sub_index = 3;
	uint8_t	packet_type = sub_address[2];
	int sub_data_len = _type_datalen[packet_type];//41
	_loosely_data.type = packet_type;

	uint16_t crc16Result = checkCrc16(sub_address , sub_data_len -2);
	uint16_t getcrc16Result = 0; 
	memcpy(&getcrc16Result , &sub_address[sub_data_len - 2] , 2);

	if (crc16Result == getcrc16Result)
	{
		int16_t middle;
		sub_index = 3;
		_loosely_data.type = toValue<uint8_t>(sub_address , sub_index);
		_loosely_data.pdata = toValue<uint16_t>(sub_address , sub_index);
		_loosely_data.gps_second_of_week = toValue<uint32_t>(sub_address , sub_index);
		_loosely_data.ptype = toValue<uint8_t>(sub_address , sub_index);

		// gyro
		_loosely_data.gx = toValue<float>(sub_address, sub_index);
		_loosely_data.gy = toValue<float>(sub_address, sub_index);
		_loosely_data.gz = toValue<float>(sub_address, sub_index);

		// acc
		_loosely_data.ax = toValue<float>(sub_address, sub_index);
		_loosely_data.ay = toValue<float>(sub_address, sub_index);
		_loosely_data.az = toValue<float>(sub_address, sub_index);

		// temp
		middle = toValue<int16_t>(sub_address, sub_index);
		_loosely_data.temp = middle * 200.0 / 32768;

		sub_index++;
		_loosely_data.IMUState = toValue<uint8_t>(sub_address , sub_index);
		index += sub_data_len;
		for (auto fun : _loosely_funs)
		{
			if (nullptr != fun.get())
			{
				(*fun.get())((void*)(&_loosely_data));
			}
		}
	}
	else
	{
		index += 4;
	}
}

std::string LooselyDataStruct::toString(const std::string& split)
{
	std::string result_string;

	// result_string += std::to_string(roll) + split;
	// result_string += std::to_string(pitch) + split;
	// result_string += std::to_string(yaw) + split;
	// result_string += std::to_string(yawDelay) + split;
	// result_string += std::to_string(gx) + split;
	// result_string += std::to_string(gy) + split;
	// result_string += std::to_string(gz) + split;
	// result_string += std::to_string(ax) + split;
	// result_string += std::to_string(ay) + split;
	// result_string += std::to_string(az) + split;
	// result_string += std::to_string(temp) + split;

	// result_string += std::to_string(bgx) + split;
	// result_string += std::to_string(bgy) + split;
	// result_string += std::to_string(bgz) + split;
	// result_string += std::to_string(bax) + split;
	// result_string += std::to_string(bay) + split;
	// result_string += std::to_string(baz) + split;
	// std::stringstream ss;
	// ss << std::setprecision(15) << lat;
	// result_string += ss.str() + split;
	// ss.str(std::string());
	// ss.clear();
	// ss << std::setprecision(15) << lon;
	// result_string += ss.str() + split;
	// ss.str(std::string());
	// ss.clear();
	// ss << std::setprecision(15) << h;
	// result_string += ss.str() + split;
	// result_string += std::to_string(vn) + split;
	// result_string += std::to_string(ve) + split;
	// result_string += std::to_string(vd) + split;
	// result_string += std::to_string(IMUState) + split;
	// result_string += std::to_string(LEKFState) + split;
	// result_string += std::to_string(gpsState) + split;

	// result_string += std::to_string(plat) + split;
	// result_string += std::to_string(plon) + split;
	// result_string += std::to_string(ph) + split;

	// result_string += std::to_string(pvn) + split;
	// result_string += std::to_string(pve) + split;
	// result_string += std::to_string(pvd) + split;

	// result_string += std::to_string(proll) + split;
	// result_string += std::to_string(ppitch) + split;
	// result_string += std::to_string(pyaw) + split;

	// result_string += std::to_string(pbgx) + split;
	// result_string += std::to_string(pbgy) + split;
	// result_string += std::to_string(pbgz) + split;

	// result_string += std::to_string(pbax) + split;
	// result_string += std::to_string(pbay) + split;
	// result_string += std::to_string(pbaz) + split;

	// result_string += std::to_string(xlat) + split;
	// result_string += std::to_string(xlon) + split;
	// result_string += std::to_string(xh) + split;

	// result_string += std::to_string(xvn) + split;
	// result_string += std::to_string(xve) + split;
	// result_string += std::to_string(xvd) + split;

	// result_string += std::to_string(xroll) + split;
	// result_string += std::to_string(xpitch) + split;
	// result_string += std::to_string(xyaw) + split;

	// result_string += std::to_string(xbgx) + split;
	// result_string += std::to_string(xbgy) + split;
	// result_string += std::to_string(xbgz) + split;

	// result_string += std::to_string(xbax) + split;
	// result_string += std::to_string(xbay) + split;
	// result_string += std::to_string(xbaz) + split;

	// result_string += std::to_string(aroll) + split;
	// result_string += std::to_string(apitch) + split;
	// result_string += std::to_string(ayaw) + split;

	// result_string += std::to_string(paroll) + split;
	// result_string += std::to_string(papitch) + split;
	// result_string += std::to_string(payaw) + split;

	// result_string += std::to_string(xaroll) + split;
	// result_string += std::to_string(xapitch) + split;
	// result_string += std::to_string(xayaw) + split;

	// result_string += std::to_string(lbbgx) + split;
	// result_string += std::to_string(lbbgy) + split;
	// result_string += std::to_string(lbbgz) + split;

	// result_string += std::to_string(xlbbgx) + split;
	// result_string += std::to_string(xlbbgy) + split;
	// result_string += std::to_string(xlbbgz) + split;

	// result_string += std::to_string(plbbgx) + split;
	// result_string += std::to_string(plbbgy) + split;
	// result_string += std::to_string(plbbgz) + split;

	// result_string += std::to_string(lbbcx) + split;
	// result_string += std::to_string(lbbcy) + split;
	// result_string += std::to_string(lbbcz) + split;

	// result_string += std::to_string(plbbcx) + split;
	// result_string += std::to_string(plbbcy) + split;
	// result_string += std::to_string(plbbcz) + split;

	// result_string += std::to_string(xlbbcx) + split;
	// result_string += std::to_string(xlbbcy) + split;
	// result_string += std::to_string(xlbbcz) + split;

	// result_string += std::to_string(kws0) + split;
	// result_string += std::to_string(kws1) + split;
	// result_string += std::to_string(pkws0) + split;
	// result_string += std::to_string(pkws1) + split;
	// result_string += std::to_string(xkws0) + split;
	// result_string += std::to_string(xkws1) + split;

	// result_string += std::to_string(ts) + split;
	// result_string += std::to_string(tsRaw) + split;
	// result_string += std::to_string(timeIMU) + split;
	// result_string += std::to_string(timeLEKF) + split;
	// result_string += std::to_string(timeGPS2IMU) + split;

	// ss.str(std::string());
	// ss.clear();
	// ss << std::setprecision(15) << TimeSyncIMU;
	// result_string += ss.str() + split;
	// ss.str(std::string());
	// ss.clear();
	// ss << std::setprecision(15) << TimeSyncGPS;
	// result_string += ss.str() + split;
	// result_string += updata ? "True" : "False" + split;

	// result_string += std::to_string(atbxr) + split;
	// result_string += std::to_string(atbyr) + split;
	// result_string += std::to_string(atbzr) + split;
	// result_string += std::to_string(stdatbxr) + split;
	// result_string += std::to_string(stdatbyr) + split;
	// result_string += std::to_string(stdatbzr) + split;

	// result_string += std::to_string(atgxr) + split;
	// result_string += std::to_string(atgyr) + split;
	// result_string += std::to_string(atgzr) + split;
	// result_string += std::to_string(stdatgxr) + split;
	// result_string += std::to_string(stdatgyr) + split;
	// result_string += std::to_string(stdatgzr) + split;

	// result_string += std::to_string(ltgxr) + split;
	// result_string += std::to_string(ltgyr) + split;
	// result_string += std::to_string(ltgzr) + split;
	// result_string += std::to_string(stdltgxr) + split;
	// result_string += std::to_string(stdltgyr) + split;
	// result_string += std::to_string(stdltgzr) + split;

	// result_string += std::to_string(ltcxr) + split;
	// result_string += std::to_string(ltcyr) + split;
	// result_string += std::to_string(ltczr) + split;
	// result_string += std::to_string(stdltcxr) + split;
	// result_string += std::to_string(stdltcyr) + split;
	// result_string += std::to_string(stdltczr) + split;

	// result_string += std::to_string(kws1r) + split;
	// result_string += std::to_string(kws2r) + split;
	// result_string += std::to_string(kws3r) + split;
	// result_string += std::to_string(stdkws1r) + split;
	// result_string += std::to_string(stdkws2r) + split;
	// result_string += std::to_string(stdkws3r) + split;

	// result_string += std::to_string(wheelselectr) + split;
	// result_string += std::to_string(stater) + split;
	return result_string;
}

std::string GnssStruct::toString(const std::string& split)
{
	std::string result_string;
// 	std::stringstream ss;
// 	ss << std::setprecision(15) << lat;
// 	result_string += ss.str() + split;
// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << lon;
// 	result_string += ss.str() + split;
// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << h;
// 	result_string += ss.str() + split;
// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << vDOP;
// 	result_string += ss.str() + split;
// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << hDOP;
// 	result_string += ss.str() + split;

// 	result_string += std::to_string(vn) + split;
// 	result_string += std::to_string(ve) + split;
// 	result_string += std::to_string(vd) + split;

// 	result_string += std::to_string(Speed3D) + split;
// 	result_string += std::to_string(Speed2D) + split;
// 	result_string += std::to_string(heading) + split;

// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << sDOP;
// 	result_string += ss.str() + split;
// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << cDOP;
// 	result_string += ss.str() + split;

// 	result_string += std::to_string(gpsfix) + split;
// 	result_string += std::to_string(flags) + split;
// 	result_string += std::to_string(numSV) + split;
// 	result_string += std::to_string(pDOP) + split;

// 	result_string += std::to_string(year) + split;
// 	result_string += std::to_string(month) + split;
// 	result_string += std::to_string(day) + split;

// 	result_string += std::to_string(hour) + split;
// 	result_string += std::to_string(min) + split;
// 	result_string += std::to_string(sec) + split;
// 	result_string += std::to_string(msg) + split;

// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << iTowPosllh;
// 	result_string += ss.str() + split;
// 	ss.str(std::string());
// 	ss.clear();
// 	ss << std::setprecision(15) << iTowVelned;
// 	result_string += ss.str() + split;
// 	result_string += std::to_string(ts) + split;
// 	result_string += isNovAtel ? "True" : "False" + split;

// 	result_string += std::to_string(latstd) + split;
// 	result_string += std::to_string(lonstd) + split;
// 	result_string += std::to_string(hMSLstd) + split;
// 	result_string += std::to_string(gpsfixNovAtel) + split;
// 	result_string += std::to_string(flagsNovAtel) + split;

// 	result_string += std::to_string(Hspd) + split;
// 	result_string += std::to_string(Trk) + split;
// 	result_string += std::to_string(VSpd) + split;
// 	result_string += std::to_string(LatencyVel) + split;
// 	result_string += std::to_string(BaseLineLength) + split;

// 	result_string += std::to_string(pitch) + split;
// 	result_string += std::to_string(pitchStd) + split;
// 	result_string += std::to_string(itow_pos) + split;
// 	result_string += std::to_string(itow_vel) + split;
// 	result_string += std::to_string(itow_heading) + split;
// 	return result_string;
// }

// std::string PollingStruct::toString(const std::string& split)
// {
// 	std::string result_string;
// 	result_string += std::to_string(LFS) + split;
// 	result_string += std::to_string(RFS) + split;
// 	result_string += std::to_string(LRS) + split;

// 	result_string += std::to_string(RRS) + split;
// 	result_string += std::to_string(Shift) + split;
// 	result_string += error ? "True" : "False" + split;
	return result_string;
}