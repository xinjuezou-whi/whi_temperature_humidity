/******************************************************************
base temp interface under ROS 1

Features:
- abstract temp interfaces
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
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace whi_temperature_humidity
{
	class SensorBase
	{
    public:
        SensorBase() = delete;
        SensorBase(std::shared_ptr<rclcpp::Node> NodeHandle)
            : node_handle_(NodeHandle){};
        virtual ~SensorBase() = default;

    public:
        virtual void parseProtocol(const std::string& ProtocolConfig) = 0;
        virtual bool request(const std::string& Param) = 0;
        virtual bool acquireValues() = 0;
        virtual void getValues(double& Temperature, double& Humidity, double& Pm25) = 0;

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
	};
} // namespace whi_temperature_humidity
