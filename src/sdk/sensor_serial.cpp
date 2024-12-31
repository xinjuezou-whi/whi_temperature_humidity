/******************************************************************
temp driver instance for serial module

Features:
- temp state get 
- serial


Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_temperature_humidity/sensor_serial.h"

#include <sstream>
#include <iomanip>
#include <thread>
#include <bitset>

namespace whi_temperature_humidity
{
    static std::bitset<16> decimalToBinary2(uint16_t decimalNumber) 
    {
        std::bitset<16> binary(decimalNumber);
        //std::cout << "二进制表示: " << binary << std::endl;
        return binary;
    }

    static int complementToDecimal(std::bitset<16> comp) 
    {
        // 如果是正数，直接转换为十进制
        if (comp[15] == 0) {
            return static_cast<int>(comp.to_ulong());
        }
        // 如果是负数，先取反再加1，然后转换为十进制并加上负号
        std::bitset<16> original = ~comp;  // 取反
        original = original.to_ulong() + 1;  // 加1
        int value = static_cast<int>(original.to_ulong());
        return -value;
    }

    static uint16_t crc16(const uint8_t* Data, size_t Length)
    {
        uint16_t crc = 0xffff;
        uint16_t polynomial = 0xa001;

        for (size_t i = 0; i < Length; ++i)
        {
            crc ^= Data[i];
            for (int j = 0; j < 8; ++j)
            {
                if ((crc & 0x0001))
                {
                    crc = (crc >> 1) ^ polynomial;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

    static int hexToString(const std::vector<uint8_t> &vec) 
    {
        std::ostringstream oss;
        // 遍历vector并以16进制格式添加到sstream中
        for (size_t i = 0; i < vec.size(); ++i)
        {
            if (i != 0)
            {
                oss << " ";
            }
            // 设置为以16进制输出，填充0，确保至少两位数，以及设置宽度
            oss << "0x" << std::setw(2)
                << std::setfill('0')     
                << std::hex              
                << static_cast<int>(vec[i]);
        }
        std::string result = oss.str();
        //ROS_INFO("send data is : %s",result.c_str());
        return 0;
    }

    static char decimalToHexChar(int decimal)
    {
        std::stringstream ss;
        ss << std::hex << decimal;
        std::string hexStr = ss.str();
        int value = std::stoi(hexStr, 0, 16);
        return static_cast<char>(value);
    }

    SensorSerial::SensorSerial(std::shared_ptr<ros::NodeHandle> NodeHandle)
        : SensorBase(NodeHandle)
    {
        init();
    }

    SensorSerial::~SensorSerial()
    {
        if (serial_inst_)
	    {
		    serial_inst_->close();
	    }
    }

    void SensorSerial::init()
    {
        // params
        node_handle_->param("port", serial_port_, std::string("/dev/ttyUSB0"));
        node_handle_->param("baudrate", baudrate_, 9600);
        int device;
        node_handle_->param("device_addr", device, 1);
        device_addr_ = uint16_t(device);

        // serial
	    try
	    {
		    serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
            ROS_INFO_STREAM("init,  device:" << serial_port_);
	    }
	    catch (serial::IOException& e)
	    {
		    ROS_FATAL_STREAM("failed to open serial " << serial_port_);
	    }

    }

    void SensorSerial::parseProtocol(const std::string& ProtocolConfig)
    {
        ROS_INFO("parsing prototocol");
        protocol_ = std::make_unique<Protocol>();
        protocol_->parseProtocol(ProtocolConfig);
    }

    bool SensorSerial::getValues(double& Temperature, double& Humidity)
    {
        std::string Param = "request";
        std::vector<uint8_t> data;
        bool result = false;
        if (protocol_->static_commands_map_)
        {
            if (auto cmd_iter = protocol_->static_commands_map_->find(Param); cmd_iter == protocol_->static_commands_map_->end())
            {
                ROS_INFO_STREAM("request param is not exist, param: " << Param);
                return false;
            }

            data = protocol_->static_commands_map_->at(Param).data_;
            uint16_t crc = crc16(data.data(), data.size());
            data.push_back(crc);
            data.push_back(uint8_t(crc >> 8));
            hexToString(data);
            if (serial_inst_)
            {
                serial_inst_->write(data.data(), data.size());
            }

            if (protocol_->feedbacks_map_)
            {
                int tryCount = 0;
                const int MAX_TRY_COUNT = 3;
                size_t count = 0;
                if (serial_inst_)
                {
                    while ((count = serial_inst_->available()) <= 0 && tryCount++ < MAX_TRY_COUNT)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    }
                    
                    if (tryCount < MAX_TRY_COUNT)
                    {
                        unsigned char rbuff[count];
                        size_t readNum = serial_inst_->read(rbuff, count);
                        if (count > 2)
                        {
                            uint16_t crc = crc16(rbuff, readNum - 2);
                            uint16_t readCrc = rbuff[readNum - 2] | uint16_t(rbuff[readNum - 1] << 8);
                            if (crc == readCrc)
                            {
                                std::ostringstream oss;
                                // 遍历vector并以16进制格式添加到sstream中
                                for (size_t i = 0; i < count; ++i)
                                {
                                    if (i != 0)
                                    {
                                        oss << " ";
                                    }
                                    // 设置为以16进制输出，填充0，确保至少两位数，以及设置宽度
                                    oss << "0x" << std::setw(2)
                                        << std::setfill('0')     
                                        << std::hex              
                                        << static_cast<int>(rbuff[i]);
                                }
#ifdef DEBUG
                                std::string getstr = oss.str();     
                                ROS_INFO("getstr is %s",getstr.c_str());
#endif

                                uint16_t tempI, humidityI;
                                tempI = (rbuff[5] << 8) | rbuff[6];
                                humidityI = (rbuff[3] << 8) | rbuff[4];
                                auto tempBin = decimalToBinary2(tempI);
                                int gettemp = complementToDecimal(tempBin);
                                auto humidityBin = decimalToBinary2(humidityI);
                                int gethumidity = complementToDecimal(humidityBin);
                                Temperature = float(gettemp) / 10.0;
                                Humidity = float(gethumidity) / 10.0;
                                result = true;
                            }
                        }
                    }
                }
            }else
            {
                result = true;
            }
        }

        return result;
    }
} // namespace whi_temperature_humidity
