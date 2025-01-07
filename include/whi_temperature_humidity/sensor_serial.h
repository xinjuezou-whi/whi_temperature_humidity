/******************************************************************
thermometer driver instance for Serial module

Features:
- thermometer state control
- Serial
- xxx

Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-12-30: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "sensor_base.h"
#include <serial/serial.h>
#include "protocol_def.h"

#include <memory>

namespace whi_temperature_humidity
{
	class SensorSerial : public SensorBase
	{
    public:
        SensorSerial() = delete;
        SensorSerial(std::shared_ptr<ros::NodeHandle> NodeHandle);
        virtual ~SensorSerial();

    public:
        void parseProtocol(const std::string& ProtocolConfig) override;
        bool getValues(double& Temperature, double& Humidity, std::string Param) override;
        bool getServiceValues(std::vector<double> & valuesVec, std::string Param) override;

    protected:
        void init();


    protected:
        std::unique_ptr<Protocol> protocol_{ nullptr };
	    std::string serial_port_;
	    int baudrate_{ 4800 };
        std::unique_ptr<serial::Serial> serial_inst_{ nullptr };
        double temp_noise_{ 0.0 };
        double humidity_{ 0.0 };
        std::map<uint8_t, std::vector<double>> values_map_;
        uint8_t addr_{ 0x01 };
	};
} // namespace whi_indicators
