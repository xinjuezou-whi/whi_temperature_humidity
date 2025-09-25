/******************************************************************
node to handle the behavior of tempget

Features:
- triggered by messages
- xxx

Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-12-30: Initial version
2025-07-25: Migrate from ROS 1 by Yuhang Su
******************************************************************/
#include <iostream>
#include <signal.h>
#include <functional>

#include "whi_temperature_humidity/whi_temperature_humidity.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI temperature and humidity sensor VERSION 00.02.2" << std::endl;
	std::cout << "Copyright © 2023-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    const std::string nodeName("whi_temperature_humidity"); 
	rclcpp::init(argc, argv);
	auto nodeHandle = std::make_shared<rclcpp::Node>(nodeName);
	/// node logic
	auto instance = std::make_unique<whi_temperature_humidity::TemperatureHumidity>(nodeHandle);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		instance = nullptr;

		// all the default sigint handler does is call shutdown()
		rclcpp::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(nodeHandle);
	executor.spin();
#else
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(nodeHandle);
	executor.spin();
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
