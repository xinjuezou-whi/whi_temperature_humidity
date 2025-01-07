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
#include <whi_interfaces/WhiTemperatureHumidity.h>

namespace whi_temperature_humidity
{
    TemperatureHumidity::TemperatureHumidity(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    void TemperatureHumidity::init()
    {
        /// params
        std::string protocolConfig;
        node_handle_->param("protocol_config", protocolConfig, std::string());
        ROS_INFO("protocol_config is %s",protocolConfig.c_str());
        sensor_ = std::make_shared<SensorSerial>(node_handle_);
        sensor_->parseProtocol(protocolConfig);

        node_handle_->param("loop_duration_temp", loop_duration_, 10.0);
        node_handle_->param("loop_duration_noise", loop_duration_noise_, 10.0);
        ros::Duration updateFreq = ros::Duration(loop_duration_);
        ros::Duration updateFreq_noise = ros::Duration(loop_duration_noise_);
        non_temp_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq,
            std::bind(&TemperatureHumidity::update, this, std::placeholders::_1)));
        non_noise_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq_noise,
            std::bind(&TemperatureHumidity::update_noise, this, std::placeholders::_1)));
        pub_temp_hum_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<whi_interfaces::WhiTemperatureHumidity>("temperature_humidity", 1));
        service_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("temperature_humidity", &TemperatureHumidity::onService, this));
        pub_noise_hum_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<whi_interfaces::WhiNoise>("noise", 1));
        service_noise_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("noise", &TemperatureHumidity::onServiceNoise, this));
    }

    void TemperatureHumidity::update(const ros::TimerEvent& Event)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);

        whi_interfaces::WhiTemperatureHumidity msg;
        double temperature = 0.0, humidity = 0.0;
        bool result = sensor_->getValues(temperature, humidity,"request_temp");
        if (result)
        {
            msg.header.stamp = ros::Time::now();
            msg.temperature = temperature;
            msg.humidity = humidity;
        }

        pub_temp_hum_->publish(msg);
    }

    void TemperatureHumidity::update_noise(const ros::TimerEvent& Event)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        whi_interfaces::WhiNoise msg;
        double noise = 0.0;
        double noise_have = 0.0;
        bool result = sensor_->getValues(noise,noise_have,"request_noise");
        if (result)
        {
            msg.header.stamp = ros::Time::now();
            msg.noise = noise;
        }

        pub_noise_hum_->publish(msg);
    }

    bool TemperatureHumidity::onService(whi_interfaces::WhiSrvTemperatureHumidity::Request& Request,
        whi_interfaces::WhiSrvTemperatureHumidity::Response& Response)
    {
        ROS_INFO_STREAM("request on service temp");
        std::vector<double> temp_humidity;
        temp_humidity.resize(2);
        bool result = sensor_->getServiceValues(temp_humidity, "temp");
        if (result)
        {
            Response.temperature_humidity.header.stamp = ros::Time::now();
            Response.temperature_humidity.temperature = temp_humidity[0];
            Response.temperature_humidity.humidity = temp_humidity[1];
            Response.result = true;
        }
        else
        {
            ROS_INFO("requested error");
            Response.result = false;
        }

        return Response.result;
    }

    bool TemperatureHumidity::onServiceNoise(whi_interfaces::WhiSrvNoise::Request& Request,
        whi_interfaces::WhiSrvNoise::Response& Response)
    {
        ROS_INFO_STREAM("request on service noise");
        std::vector<double> noise;
        noise.resize(1);
        bool result = sensor_->getServiceValues(noise, "noise");
        if (result)
        {
            Response.noise_msg.header.stamp = ros::Time::now();
            Response.noise_msg.noise = noise[0];
            Response.result = true;
        }
        else
        {
            ROS_INFO("requested error");
            Response.result = false;
        }

        return Response.result;
    }

} // namespace whi_temperature_humidity
