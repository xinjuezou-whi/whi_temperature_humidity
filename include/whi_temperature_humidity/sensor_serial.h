/******************************************************************
thermometer driver instance for Serial module

Features:
- thermometer state control
- Serial
- xxx

Written by Yue Zhou, sevendull@163.com
Refactored by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-12-30: Initial version
2026-05-12: Refactor
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "sensor_base.h"
#include <serial/serial.h>
#include "protocol_def.h"
#include <whi_interfaces/srv/whi_srv_mod_bus.hpp>

#include <memory>
#include <mutex>
#include <thread>

namespace whi_temperature_humidity
{
	class SensorSerial : public SensorBase
	{
    public:
        SensorSerial() = delete;
        SensorSerial(std::shared_ptr<rclcpp::Node> NodeHandle,
            const std::string& Port, int Baudrate, int DeviceAddr);
        SensorSerial(std::shared_ptr<rclcpp::Node> NodeHandle,
            int DeviceAddr, const std::string& Service);
        virtual ~SensorSerial();

    public:
        void parseProtocol(const std::string& ProtocolConfig) override;
        bool request(const std::string& Param) override;
        bool acquireValues() override;
        void getValues(double& Temperature, double& Humidity, double& Pm25) override;

    protected:
        bool parseValues(const std::vector<uint8_t>& Raw);

    protected:
        std::unique_ptr<Protocol> protocol_{ nullptr };
	    std::string serial_port_;
	    int baudrate_{ 4800 };
        std::unique_ptr<serial::Serial> serial_inst_{ nullptr };
        double temperature_{ 0.0 };
        double humidity_{ 0.0 };
        double pm25_{ 0.0 };
        int device_addr_{ 0 };
        rclcpp::Client<whi_interfaces::srv::WhiSrvModBus>::SharedPtr modbus_client_{ nullptr };
        rclcpp::Client<whi_interfaces::srv::WhiSrvModBus>::SharedFuture future_;
        std::mutex data_mtx_;
	};
} // namespace whi_indicators
