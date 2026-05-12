/******************************************************************
temp driver instance for serial module

Features:
- temp state get 
- serial


Written by Yue Zhou, sevendull@163.com
Refactored by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
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

    SensorSerial::SensorSerial(std::shared_ptr<rclcpp::Node> NodeHandle,
        const std::string& Port, int Baudrate, int DeviceAddr)
        : SensorBase(NodeHandle)
        , serial_port_(Port), baudrate_(Baudrate), device_addr_(DeviceAddr)
    {
        // serial
	    try
	    {
		    serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
            RCLCPP_INFO_STREAM(node_handle_->get_logger(), "init,  device: " << serial_port_);
	    }
	    catch (serial::IOException& e)
	    {
		    RCLCPP_FATAL_STREAM(node_handle_->get_logger(), "failed to open serial " << serial_port_);
	    }
    }

    SensorSerial::SensorSerial(std::shared_ptr<rclcpp::Node> NodeHandle,
        int DeviceAddr, const std::string& Service)
        : SensorBase(NodeHandle)
        , device_addr_(DeviceAddr)
    {
        modbus_client_ = node_handle_->create_client<whi_interfaces::srv::WhiSrvModBus>(Service);
    }

    SensorSerial::~SensorSerial()
    {
        if (serial_inst_)
	    {
		    serial_inst_->close();
	    }
    }

    void SensorSerial::parseProtocol(const std::string& ProtocolConfig)
    {
        RCLCPP_INFO(node_handle_->get_logger(), "parsing prototocol");
        protocol_ = std::make_unique<Protocol>();
        protocol_->parseProtocol(ProtocolConfig);
    }

    bool SensorSerial::request(const std::string& Param)
    {
        if (protocol_->static_commands_map_)
        {
            if (auto found = protocol_->static_commands_map_->find(Param); found == protocol_->static_commands_map_->end())
            {
                RCLCPP_INFO_STREAM(node_handle_->get_logger(), "request param is not exist, param: " << Param);
                return false;
            }

            std::vector<uint8_t> data = protocol_->static_commands_map_->at(Param).data_;

            if (serial_inst_)
            {
                if (protocol_->static_commands_map_->at(Param).variables_map_.empty())
                {
                    device_addr_ = data[0];
                    uint16_t crc = crc16(data.data(), data.size());
                    data.push_back(crc);
                    data.push_back(uint8_t(crc >> 8));
                }
                else
                {
                    for (const auto& variable : protocol_->static_commands_map_->at(Param).variables_map_)
                    {
                        if (variable.first == "device_addr")
                        {
                            data[variable.second] = device_addr_;
                        }
                    }
                    uint16_t crc = crc16(data.data(), data.size());
                    data.push_back(crc);
                    data.push_back(uint8_t(crc >> 8));
                }
#ifdef DEBUG
    std::cout << "write data:" << std::endl;
    for (const auto& it : data)
    {
        std::cout << std::hex << int(it) << ",";
    }
    std::cout << std::endl;
#endif

                return serial_inst_->write(data.data(), data.size()) > 0;
            }
            else if (modbus_client_)
            {
                if (!protocol_->static_commands_map_->at(Param).variables_map_.empty())
                {
                    for (const auto& variable : protocol_->static_commands_map_->at(Param).variables_map_)
                    {
                        if (variable.first == "device_addr")
                        {
                            data[variable.second] = device_addr_;
                        }
                    }
                }

                auto request = std::make_shared<whi_interfaces::srv::WhiSrvModBus::Request>();
                request->instance.device = data[0];
                request->instance.func = data[1];
                request->instance.crc_size = 0;
                request->instance.data = std::vector<uint8_t>(data.begin() + 2, data.end());
#ifdef DEBUG
    std::cout << "data send:" << std::endl;
    for (const auto& it : data)
    {
        std::cout << std::hex << int(it) << ",";
    }
    std::cout << std::endl;
#endif
                
                future_ = modbus_client_->async_send_request(request);

                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool SensorSerial::acquireValues()
    {
        if (serial_inst_)
        {
            int tryCount = 0;
            const int MAX_TRY_COUNT = 3;
            size_t count = 0;
            while ((count = serial_inst_->available()) <= 0 && tryCount++ < MAX_TRY_COUNT)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            if (tryCount < MAX_TRY_COUNT)
            {
                unsigned char rbuff[count];
                size_t readNum = serial_inst_->read(rbuff, count);
                if (count > 2 && device_addr_ == rbuff[0] && readNum >= 19)
                {
                    uint16_t crc = crc16(rbuff, readNum - 2);
                    uint16_t readCrc = rbuff[readNum - 2] | uint16_t(rbuff[readNum - 1] << 8);
                    if (crc == readCrc)
                    {
                        if (rbuff[0] == device_addr_)
                        {
                            std::vector<uint8_t> raw(rbuff, rbuff + count);
                            return parseValues(raw);
                        }
                        else
                        {
                            return false;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else if (modbus_client_)
        {
            auto result = future_.get();
            if (result->result)
            {
                return parseValues(future_.get()->data);
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    void SensorSerial::getValues(double& Temperature, double& Humidity, double& Pm25)
    {
        std::lock_guard lk(data_mtx_);
        Temperature = temperature_;
        Humidity = humidity_;
        Pm25 = pm25_;
    }

    bool SensorSerial::parseValues(const std::vector<uint8_t>& Raw)
    {
        if (Raw.size() > 18)
        {
            uint16_t tempI, humidityI, pm25I;
            tempI = (Raw[3] << 8) | Raw[4];
            humidityI = (Raw[5] << 8) | Raw[6];
            pm25I = (Raw[17] << 8) | Raw[18];  // PM2.5在第17-18字节
            auto tempBin = decimalToBinary2(tempI);
            int gettemp = complementToDecimal(tempBin);
            auto humidityBin = decimalToBinary2(humidityI);
            int gethumidity = complementToDecimal(humidityBin);
            // PM2.5数据转换：根据手册，PM2.5直接使用寄存器值，单位ug/m3
            auto pm25Bin = decimalToBinary2(pm25I);
            int getPm25 = complementToDecimal(pm25Bin);

            std::unique_lock lk(data_mtx_);
            temperature_ = float(gettemp - 2000) / 100.0;
            humidity_ = float(gethumidity) / 100.0;
            pm25_ = float(getPm25);  // PM2.5直接使用，单位ug/m3

            return true;
        }
        else
        {
            return false;
        }
    }

} // namespace whi_temperature_humidity
