/******************************************************************
base temp interface under ROS 1

Features:
- abstract temp interfaces
- xxx

Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-12-30: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h>

#include <memory>

namespace whi_temperature_humidity
{
	class SensorBase
	{
    public:
        SensorBase() = delete;
        SensorBase(std::shared_ptr<ros::NodeHandle> NodeHandle)
            : node_handle_(NodeHandle){};
        virtual ~SensorBase() = default;

    public:
        virtual void parseProtocol(const std::string& ProtocolConfig) = 0;
        virtual bool getValues(double& Temperature, double& Humidity, std::string Param) = 0;
        virtual bool getServiceValues(std::vector<double> & valuesVec, std::string Param) = 0;

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
	};
} // namespace whi_temperature_humidity
