/******************************************************************
Gettemp interface under ROS 1

Features:
- abstract Gettemp interfaces
- xxx

Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_temperature_humidity/whi_temperature_humidity.h"
#include "whi_temperature_humidity/sensor_serial.h"
#include <whi_interfaces/msg/whi_temperature_humidity.hpp>

namespace whi_temperature_humidity
{
    TemperatureHumidity::TemperatureHumidity(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle), elapsed_time_(rclcpp::Duration(0, 0))
    {
        init();
    }

    void TemperatureHumidity::init()
    {
        /// params
        std::string protocolConfig;
        node_handle_->declare_parameter("protocol_config", std::string());
        node_handle_->get_parameter("protocol_config", protocolConfig);
        RCLCPP_INFO(node_handle_->get_logger(), "protocol_config is %s", protocolConfig.c_str());
        sensor_ = std::make_shared<SensorSerial>(node_handle_);
        sensor_->parseProtocol(protocolConfig);

        node_handle_->declare_parameter("loop_duration_temp", 10.0);
        node_handle_->declare_parameter("loop_duration_decibel", 10.0);
        node_handle_->get_parameter("loop_duration_temp", loop_duration_);
        node_handle_->get_parameter("loop_duration_decibel", loop_duration_decibel_);
        
        auto updateFreq = std::chrono::duration<double>(loop_duration_);
        auto updateFreq_decibel = std::chrono::duration<double>(loop_duration_decibel_);
        
        non_temp_loop_ = node_handle_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(updateFreq),
            std::bind(&TemperatureHumidity::update, this));
        non_decibel_loop_ = node_handle_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(updateFreq_decibel),
            std::bind(&TemperatureHumidity::update_decibel, this));
            
        pub_temp_hum_ = node_handle_->create_publisher<whi_interfaces::msg::WhiTemperatureHumidity>(
            "temperature_humidity", 1);
        service_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvTemperatureHumidity>(
            "temperature_humidity", 
            std::bind(&TemperatureHumidity::onService, this, std::placeholders::_1, std::placeholders::_2));
        pub_decibel_hum_ = node_handle_->create_publisher<whi_interfaces::msg::WhiDecibel>(
            "decibel", 1);
        service_decibel_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvDecibel>(
            "decibel", 
            std::bind(&TemperatureHumidity::onServiceDecibel, this, std::placeholders::_1, std::placeholders::_2));
    }

    void TemperatureHumidity::update()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        static auto last_time = node_handle_->now();
        auto current_time = node_handle_->now();
        elapsed_time_ = current_time - last_time;
        last_time = current_time;

        whi_interfaces::msg::WhiTemperatureHumidity msg;
        double temperature = 0.0, humidity = 0.0, pm25 = 0.0; 
        bool result = sensor_->getValues(temperature, humidity, pm25, "request_temp");
        if (result)
        {
            msg.header.stamp = node_handle_->now();
            msg.temperature = temperature;
            msg.humidity = humidity;
            msg.pm25 = pm25;
        }

        pub_temp_hum_->publish(msg);
    }

    void TemperatureHumidity::update_decibel()
    {
        // 暂时不用这个分贝传感器
        return ;
        std::lock_guard<std::mutex> lock(mutex_);
        whi_interfaces::msg::WhiDecibel msg;
        double decibel = 0.0;
        double decibel_have = 0.0;
        double unused_pm25 = 0.0;  // 添加占位参数
        bool result = sensor_->getValues(decibel, decibel_have, unused_pm25, "request_decibel");
        if (result)
        {
            msg.header.stamp = node_handle_->now();
            msg.decibel = decibel;
        }

        pub_decibel_hum_->publish(msg);
    }

    void TemperatureHumidity::onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Response> Response)
    {
        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "request on service temp");
        std::vector<double> temp_humidity;
        temp_humidity.resize(3);
        bool result = sensor_->getServiceValues(temp_humidity, "temp");
        if (result)
        {
            Response->temperature_humidity.header.stamp = node_handle_->now();
            Response->temperature_humidity.temperature = temp_humidity[0];
            Response->temperature_humidity.humidity = temp_humidity[1];
            Response->temperature_humidity.pm25 = temp_humidity[2];
            Response->result = true;
        }
        else
        {
            RCLCPP_INFO(node_handle_->get_logger(), "requested error");
            Response->result = false;
        }
    }

    void TemperatureHumidity::onServiceDecibel(const std::shared_ptr<whi_interfaces::srv::WhiSrvDecibel::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvDecibel::Response> Response)
    {
        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "request on service decibel");
        std::vector<double> decibel;
        decibel.resize(1);
        bool result = sensor_->getServiceValues(decibel, "decibel");
        if (result)
        {
            Response->decibel_msg.header.stamp = node_handle_->now();
            Response->decibel_msg.decibel = decibel[0];
            Response->result = true;
        }
        else
        {
            RCLCPP_INFO(node_handle_->get_logger(), "requested error");
            Response->result = false;
        }
    }

} // namespace whi_temperature_humidity
