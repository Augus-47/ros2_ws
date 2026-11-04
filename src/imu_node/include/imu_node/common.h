#ifndef __COMMON_H__
#define __COMMON_H__

// #include <qglobal.h>
#include <string>
// #include <QString>
// #include <QColor>
#include <string.h>
#ifndef PI
#define PI          3.1415926535897932  /* pi */
#endif

#ifndef D2R
#define D2R         (PI/180.0)          /* deg to rad */
#endif

const int G_INTERVAL_UPDATE_TIME = 100;//��������ˢ�¼��

const std::string G_DOWN_SOURCETABLE_FINISHED = "download source table finished";

/// <summary>
/// ��־��������
/// </summary>
enum class ConnectType
{
	eNone = 0,
	eTxt = 1,//�ı�
	eVideo = 2,//ý�����ݣ�����USB����ͷ
	eSerial = 3,//��������
	eCan = 4,//can����
	eUdp = 5,//��̫������
	eTcp = 6,//��̫������
	eSomeip = 7,//��̫������
	eDoip = 8,//��̫������
	eNtrip = 9
};

/// <summary>
/// ������ݵ�����
/// </summary>
enum class ProtocolType
{
	eNone = 0,
	eIns = 1,//ins
	eGnss = 2,//gnss
	ePoll = 3,//����
	eVideo = 4
};

enum class LogType {
	eNone = 0,
	eText,//��ͨ�ı�
	eVideoRecordStart,//��ʼ¼����Ƶ
	eVideoRecordStop,//����¼����Ƶ
	eSerialPortConnect,//����
	eSerialPort,//����
	eSerialPortClose,//����
	eCanConnect,//CAN
	eCan,//CAN
	eCanClose,//CAN
	eCanfdConnect,//CANFD
	eCanfd,//CANFD
	eCanfdClose,//CANFD
	eUdpListen,//udp
	eUdp,//udp
	eUdpClose,//udp
	eTcpConnect,//tcp
	eTcp,//tcp
	eTcpClose,//tcp
	eDoipConnectTcp,//Doip
	eDoipTcp,//Doip
	eDoipCloseTcp,//Doip
	eDoipConnectUdp,//Doip
	eDoipUdp,//Doip
	eDoipCloseUdp,//Doip
	eSomeipConnectTcp,//Some/Ip
	eSomeipTcp,//Some/Ip
	eSomeipCloseTcp,//Some/Ip
	eSomeipConnectUdp,//Some/Ip
	eSomeipUdp,//Some/Ip
	eSomeipCloseUdp,//Some/Ip
	eNtripConnect,//tcp
	eNtrip,//tcp
	eNtripClose//tcp
};

/// <summary>
/// ״̬�ռ�ö��ֵ
/// </summary>
enum class StatusUi
{
	eNone,
	eSerial,
	eCan,
	eUdp,
	eTcp,
	ePosLoss,//RTK
	eBiasDiffuse,//��ƫ��ɢ
	eSizeDiffuse,//�˱۷�ɢ
	eAngleDiffuse,//��̬��ɢ
	eRtkOff,//�����ʱ
	eWheelSpeedLoss,//���ٶ�ʧ
	eGnssLoss//GNSS��ʧ
};

/// <summary>
/// ����״ֵ̬
/// </summary>
enum class StatusType
{
	eNone,
	eOpenOrValid,
	eError,
	eCloseOrOver
};

/// <summary>
/// ������¼��������¼��key��ip��ַ+����
/// </summary>
struct ReceiveDataKey
{
	ReceiveDataKey() {}
	ReceiveDataKey(uint32_t ip, int type) :_ip(ip), _asc_type(type) {}
	uint32_t _ip = 0;
	int _asc_type = 0;//1:IMUInfoNotify,2:INSInfoNotify,3:INSStatusInfoNotify,4:GNSSInfoInfoNotify,5:PartNum
	bool operator<(const ReceiveDataKey& other) const {
		if (_ip < other._ip)
		{
			return true;
		}
		if (_ip == other._ip && _asc_type < other._asc_type)
		{
			return true;
		}
		return false;
	}
};

/// <summary>
/// ��ת���ݣ���С��ת����
/// </summary>
/// <param name="temp"></param>
/// <param name="length"></param>
void reverseData(uint8_t* temp, int length);

/// <summary>
/// ������תʵ������,�Դ���С��ת��
/// </summary>
/// <typeparam name="T"></typeparam>
/// <param name="add"></param>
/// <param name="index"></param>
/// <returns></returns>
template<typename T>
T serializeData(const uint8_t* add, int& index)
{
	if (sizeof(T) > 1)
	{
		uint8_t* data = (uint8_t*)malloc(sizeof(T));
		memcpy(data, add + index, sizeof(T));
		reverseData(data, sizeof(T));
		T result = *((T*)(data));
		free(data);
		index += sizeof(T);
		return result;
	}
	else
	{
		T result = *(T*)(add + index);
		index += sizeof(T);
		return result;
	}
}

/// <summary>
/// ������תʵ������
/// </summary>
/// <typeparam name="T"></typeparam>
/// <param name="add"></param>
/// <param name="index"></param>
/// <returns></returns>
template<typename T>
T toValue(const uint8_t* add, int& index)
{
	T result = *(T*)(add + index);
	index += sizeof(T);
	return result;
}

/// <summary>
/// ������תʵ������
/// </summary>
/// <typeparam name="T"></typeparam>
/// <param name="add"></param>
/// <param name="index"></param>
/// <returns></returns>
template<typename T>
T toValue(const char* add, int& index)
{
	T result = *(T*)(add + index);
	index += sizeof(T);
	return result;
}

template<typename T>
std::string toBytes(const T& data)
{
	return std::string((char*)&data, sizeof(T));
}

/// <summary>
/// �ַ���0~F/f)תʮ����
/// </summary>
/// <param name="data"></param>
/// <returns></returns>
char convertToBit(char data);

/// <summary>
/// ������ת16��������
/// </summary>
/// <param name="data"></param>
/// <returns></returns>
// QString convertToHex(const std::string& data);

/// <summary>
/// 16��������ת�����ƣ����硰FF"->255
/// </summary>
/// <param name="origin_data"></param>
/// <returns></returns>
// std::string convertToBin(const QString& origin_data);

// #define  COLOR_RGBToINT(r,g,b) COLOR_RGBAToINT(r,g,b,255)
// #define  COLOR_RGBAToINT(r,g,b,a) (b + g * 256 + r * 65536 + a * 16777216)
// #define  COLOR_R(c)  ((c & 0x00ff0000) >> 16)
// #define  COLOR_G(c)  ((c & 0x0000ff00) >> 8)
// #define  COLOR_B(c)  ((c & 0x000000ff))
// #define  COLOR_A(c)  ((c & 0xff000000) >> 24)

// static QColor IntToColor(int color)
// {
// 	return QColor(COLOR_R(color), COLOR_G(color), COLOR_B(color), COLOR_A(color));
// }

// static int ColorToInt(const QColor& color)
// {
// 	return COLOR_RGBAToINT(color.red(), color.green(), color.blue(), color.alpha());
// }

#endif
