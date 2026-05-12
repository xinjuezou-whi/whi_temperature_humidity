/******************************************************************
Gettemp interface under ROS 1

Features:
- abstract Gettemp interfaces
- xxx

Written by Yue Zhou, sevendull@163.com
Refactored by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
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

    TemperatureHumidity::~TemperatureHumidity()
    {
        terminated_.store(true);
        if (th_loop_.joinable())
        {
            th_loop_.join();
        }
    }

    void TemperatureHumidity::init()
    {
        /// params
        node_handle_->declare_parameter("protocol_config", std::string());
        auto protocolConfig = node_handle_->get_parameter("protocol_config").as_string();
        RCLCPP_INFO(node_handle_->get_logger(), "protocol_config is %s", protocolConfig.c_str());
        node_handle_->declare_parameter("device_addr", 0);
        auto deviceAddr = node_handle_->get_parameter("device_addr").as_int();
        node_handle_->declare_parameter("modbus_instance", std::string());
        auto modbusInstance = node_handle_->get_parameter("modbus_instance").as_string();
        if (modbusInstance == "stand_alone")
        {
            node_handle_->declare_parameter(modbusInstance + ".port", std::string("/dev/ttyUSB0"));
            auto port = node_handle_->get_parameter(modbusInstance + ".port").as_string();
            node_handle_->declare_parameter(modbusInstance + ".baudrate", 9600);
            auto baudrate = node_handle_->get_parameter(modbusInstance + ".baudrate").as_int();

            sensor_ = std::make_shared<SensorSerial>(node_handle_, port, baudrate, deviceAddr);
        }
        else if (modbusInstance == "server_depend")
        {
            node_handle_->declare_parameter(modbusInstance + ".modbus_service", std::string("modbus_request"));
            auto service = node_handle_->get_parameter(modbusInstance + ".modbus_service").as_string();

            sensor_ = std::make_shared<SensorSerial>(node_handle_, deviceAddr, service);
        }
        sensor_->parseProtocol(protocolConfig);

        node_handle_->declare_parameter("loop_duration_temp", 10.0);
        node_handle_->get_parameter("loop_duration_temp", loop_duration_);

        th_loop_ = std::thread(std::bind(&TemperatureHumidity::update, this));
        
        // publisher
        pub_temp_hum_ = node_handle_->create_publisher<whi_interfaces::msg::WhiTemperatureHumidity>(
            "temperature_humidity", 1);
        // whi_state publisher
        pub_state_ = node_handle_->create_publisher<whi_interfaces::msg::WhiState>("whi_state", 10);
        // service
        service_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvTemperatureHumidity>(
            "temperature_humidity", 
            std::bind(&TemperatureHumidity::onService, this, std::placeholders::_1, std::placeholders::_2));
    }

    template <typename T>
    std::string toStringWithPrecision(const T Value, const int Digits = 6)
    {
        std::ostringstream out;
        out.precision(Digits);
        out << std::fixed << Value;
        return out.str();
    }

    void TemperatureHumidity::update()
    {
        while (!terminated_)
        {
            static auto lastTime = node_handle_->now();
            auto currentTime = node_handle_->now();
            elapsed_time_ = currentTime - lastTime;
            lastTime = currentTime;
            
            auto res = sensor_->request("request_temp");
            res = sensor_->acquireValues();
            whi_interfaces::msg::WhiTemperatureHumidity msg;
            if (res)
            {
                double temperature = 0.0, humidity = 0.0, pm25 = 0.0;
                sensor_->getValues(temperature, humidity, pm25);
                msg.header.stamp = currentTime;
                msg.header.frame_id = frame_id_;
                msg.temperature = temperature;
                msg.humidity = humidity;
                msg.pm25 = pm25;
            }

            pub_temp_hum_->publish(msg);

            whi_interfaces::msg::WhiState staMsg;
            staMsg.header.stamp = currentTime;
            staMsg.hardware_id = "whi_temperature_humidity";
            staMsg.level = whi_interfaces::msg::WhiState::INFO;
            diagnostic_msgs::msg::KeyValue value;
            value.key = "temperature";
            value.value = toStringWithPrecision(msg.temperature, 1);
            staMsg.values.push_back(value);
            value.key = "humidity";
            value.value = toStringWithPrecision(msg.humidity, 1);
            staMsg.values.push_back(value);
        
            pub_state_->publish(staMsg);

            std::this_thread::sleep_for(std::chrono::duration<double>(loop_duration_));
        }
    }

    void TemperatureHumidity::onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvTemperatureHumidity::Response> Response)
    {
        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "request on service temp");

        Response->temperature_humidity.header.stamp = node_handle_->now();
        Response->temperature_humidity.header.frame_id = frame_id_;
        double temperature, humidity, pm25;
        sensor_->getValues(temperature, humidity, pm25);
        Response->temperature_humidity.temperature = temperature;
        Response->temperature_humidity.humidity = humidity;
        Response->temperature_humidity.pm25 = pm25;
    }
} // namespace whi_temperature_humidity
