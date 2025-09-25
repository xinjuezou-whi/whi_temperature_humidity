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
#include <whi_interfaces/msg/whi_temperature_humidity.hpp>
#include <whi_interfaces/srv/whi_srv_temperature_humidity.hpp>
#include <whi_interfaces/msg/whi_decibel.hpp>
#include <whi_interfaces/srv/whi_srv_decibel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <map>
#include <mutex>

namespace whi_temperature_humidity
{
	class TemperatureHumidity
	{
    public:
        TemperatureHumidity(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~TemperatureHumidity() = default;

    protected:
        void init();
        void update();
        void update_decibel();
        void onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Request> Request,
                std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Response> Response);
                
        void onServiceDecibel(const std::shared_ptr<whi_interfaces::srv::WhiSrvDecibel::Request> Request,
                std::shared_ptr<whi_interfaces::srv::WhiSrvDecibel::Response> Response);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        std::shared_ptr<rclcpp::TimerBase> non_temp_loop_{ nullptr };
        std::shared_ptr<rclcpp::TimerBase> non_decibel_loop_{ nullptr };
        rclcpp::Duration elapsed_time_;
        double loop_duration_{ 10.0 };
        double loop_duration_decibel_{ 10.0 };
        std::shared_ptr<SensorBase> sensor_{ nullptr };
        std::shared_ptr<rclcpp::Publisher<whi_interfaces::msg::WhiTemperatureHumidity>> pub_temp_hum_{ nullptr };
        std::shared_ptr<rclcpp::Service<whi_interfaces::srv::WhiSrvTemperatureHumidity>> service_{ nullptr };
           
        std::shared_ptr<rclcpp::Publisher<whi_interfaces::msg::WhiDecibel>> pub_decibel_hum_{ nullptr };
        std::shared_ptr<rclcpp::Service<whi_interfaces::srv::WhiSrvDecibel>> service_decibel_{ nullptr };
           
        std::mutex mutex_;
	};
} // namespace whi_get_temp
