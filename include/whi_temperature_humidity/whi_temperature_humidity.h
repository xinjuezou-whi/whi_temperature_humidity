/******************************************************************
temp get interface under ROS 1

Features:
- abstract temp get interfaces
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
#include <whi_interfaces/msg/whi_temperature_humidity.hpp>
#include <whi_interfaces/srv/whi_srv_temperature_humidity.hpp>
#include <whi_interfaces/msg/whi_state.hpp>
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
        ~TemperatureHumidity();

    protected:
        void init();
        void update();
        void onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Request> Request,
                std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Response> Response);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        rclcpp::Duration elapsed_time_;
        double loop_duration_{ 10.0 };
        std::shared_ptr<SensorBase> sensor_{ nullptr };
        std::shared_ptr<rclcpp::Publisher<whi_interfaces::msg::WhiTemperatureHumidity>> pub_temp_hum_{ nullptr };
        rclcpp::Publisher<whi_interfaces::msg::WhiState>::SharedPtr pub_state_{ nullptr };
        std::shared_ptr<rclcpp::Service<whi_interfaces::srv::WhiSrvTemperatureHumidity>> service_{ nullptr };
        std::thread th_loop_;
        std::atomic_bool terminated_{ false };
        std::string frame_id_{ "temperature_humidity_sensor" };
	};
} // namespace whi_get_temp
