/******************************************************************
temp get interface under ROS 1

Features:
- abstract temp get interfaces
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
#include <whi_interfaces/WhiTemperatureHumidity.h>
#include <whi_interfaces/WhiSrvTemperatureHumidity.h>

#include <ros/ros.h>

#include <memory>
#include <map>

namespace whi_temperature_humidity
{
	class TemperatureHumidity
	{
    public:
        TemperatureHumidity(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~TemperatureHumidity() = default;

    protected:
        void init();
        void update(const ros::TimerEvent & Event);
        bool onService(whi_interfaces::WhiSrvTemperatureHumidity::Request& Request,
                whi_interfaces::WhiSrvTemperatureHumidity::Response& Response);         

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_duration_{ 10.0 };
        std::shared_ptr<SensorBase> sensor_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_temp_hum_{ nullptr };
        std::unique_ptr<ros::ServiceServer> service_{ nullptr };          
	};
} // namespace whi_get_temp
