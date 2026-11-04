#include "imu_node/common.h"

void reverseData(uint8_t* temp, int length)
{
	for (int i = 0; i < length / 2; i++)
	{
		std::swap(temp[i], temp[length - 1 - i]);
	}
}

char convertToBit(char data)
{
	if (data >= 48 && data <= 57)
	{//0-9
		return data - 48;
	}
	else if (data >= 65 && data <= 70)
	{
		return data - 55;
	}
	else
	{
		return data - 87;
	}
}

// QString convertToHex(const std::string& data)
// {
// 	QString result_data;
// 	char sub_data[4] = { 0 };
// 	uchar* address = (uchar*)data.c_str();
// 	for (size_t i = 0; i < data.size(); ++i)
// 	{
// 		sprintf(sub_data, "%02X ", address[i]);
// 		result_data += QString::fromLocal8Bit(QByteArray(sub_data, 3));
// 	}
// 	return result_data;
// }

// std::string convertToBin(const QString& origin_data)
// {
// 	QString data = origin_data;
// 	std::string result_data(data.size() / 2, 0);
// 	for (size_t i = 0; i < data.size(); i += 2)
// 	{
// 		uint8_t f = convertToBit(data.at(i).cell());
// 		uint8_t s = convertToBit(data.at(i + 1).cell());
// 		result_data[i / 2] = (f << 4) + s;
// 	}
// 	return result_data;
// }