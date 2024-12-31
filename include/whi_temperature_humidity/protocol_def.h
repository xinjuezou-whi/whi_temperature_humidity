/******************************************************************
general definition for serial protocol

Features:
- general definition
- xxx

Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-12-30: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct OperatingParams
{
    OperatingParams() = default;
    std::map<std::string, int64_t> params_map_;
};

class Protocol
{
public:
    const char* VERSION = "WHI generic protocol 00.1.t";

public:
    struct Shift
    {
        Shift() = default;
        enum { SHIFT_LEFT = 0, SHIFT_RIGHT };
        uint8_t bits_{ 0 };
        uint8_t dir_{ SHIFT_LEFT };
    };

    class StaticCommand
    {
    public:
        StaticCommand() = default;
        ~StaticCommand() = default;

    public:
        std::string name_;
        std::vector<uint8_t> data_;
        uint32_t base_addr_{ 0 };
        bool is_absolute_base_{ false };
        std::map<std::string, uint8_t> variables_map_;
        uint32_t delay_{ 0 };
    };

    class FeebackByte
    {
    public:
        FeebackByte() = default;
        ~FeebackByte() = default;

    public:
        uint8_t index_{ 0 };
        Shift shift_;
    };

    class FeedbackParam
    {
    public:
        FeedbackParam() = default;
        ~FeedbackParam() = default;

    public:
        uint32_t base_addr_{ 0 };
        bool is_absolute_base_{ false };
        std::vector<FeebackByte> bytes_;
        double multiplier_{ 1.0 };
        double divider_{ 1.0 };
        double additioner_{ 0.0 };
    };

    enum Type { FRAME_DATA = 0, FRAME_REMOTE, FRAME_NONE };

public:
	Protocol() = default;
	~Protocol() = default;

public:
    using StaticCommandsMap = std::map<std::string, StaticCommand>;
    using FeedbacksMap = std::map<std::string, FeedbackParam>;

public:
    void parseProtocol(const std::string& ProtocolConfig)
    {
        try
        {
            YAML::Node node = YAML::LoadFile(ProtocolConfig);

            // static commands
            loadStaticCommandsList(node, "static_commands", static_commands_map_);
            // feedbacks
            loadFeedbacksMap(node, "feedbacks", feedbacks_map_);
        }
        catch (const std::exception& e)
        {
            std::cout << "failed to load protocol config file " << ProtocolConfig << std::endl;
        }
    }

protected:
    static void loadStaticCommandsList(const YAML::Node& Node, const std::string& Key,
        std::unique_ptr<StaticCommandsMap>& Map)
    {
        const auto& commands = Node[Key];
        if (commands)
        {
            Map = std::make_unique<StaticCommandsMap>();

            for (const auto& command : commands)
            {
                std::string name = command["param"].as<std::string>();
                Map->emplace(name, StaticCommand());
                Map->at(name).name_= name;
                for (const auto& it : command["data"])
                {
                    Map->at(name).data_.push_back(uint8_t(it.as<int>()));
                }
                Map->at(name).base_addr_ = command["base_addr"].as<uint32_t>();
                Map->at(name).is_absolute_base_ = command["absolute_base"].as<bool>();
                const auto& variables = command["variables"];
                if (variables)
                {
                    for (const auto& it : variables)
                    {
                        Map->at(name).variables_map_.emplace(
                            std::make_pair(it.second.as<std::string>(), uint8_t(it.first.as<int>())));
                    }
                }
                const auto& delay = command["delay"];
                if (delay)
                {
                    Map->at(name).delay_ = 1000 * delay.as<int>();
                }
            }
        }
    }

    static void loadFeedbacksMap(const YAML::Node& Node, const std::string& Key,
        std::unique_ptr<FeedbacksMap>& Map)
    {
        const auto& feedbacks = Node["feedbacks"];
        if (feedbacks)
        {
            Map = std::make_unique<FeedbacksMap>();

            for (const auto& param : feedbacks)
            {
                auto name = param["param"].as<std::string>();
                Map->emplace(std::make_pair(name, FeedbackParam()));

                Map->at(name).base_addr_ = param["base_addr"].as<uint32_t>();
                Map->at(name).is_absolute_base_ = param["absolute_base"].as<bool>();

                const auto& multiplier = param["multiplier"];
                if (multiplier)
                {
                    Map->at(name).multiplier_ = multiplier.as<double>();
                }
                const auto& divider = param["divider"];
                if (divider)
                {
                    Map->at(name).divider_ = divider.as<double>();
                }
                const auto& additioner = param["additioner"];
                if (additioner)
                {
                    Map->at(name).additioner_ = additioner.as<double>();
                }

                auto bytes = param["bytes"];
                if (bytes)
                {
                    for (const auto& it : bytes)
                    {
                        FeebackByte feed;
                        feed.index_ = uint8_t(it.first.as<int>());
                        feed.shift_.bits_ = uint8_t(it.second.as<int>());
                        Map->at(name).bytes_.push_back(feed);
                    }
                    // sort bytes with ascending order for determining the sign of bit shift
                    std::sort(Map->at(name).bytes_.begin(), Map->at(name).bytes_.end(),
                        [](const FeebackByte& A, const FeebackByte& B)
                        {
                            return A.shift_.bits_ < B.shift_.bits_;
                        });
                }
            }
        }
    }

public:
    std::unique_ptr<StaticCommandsMap> static_commands_map_{ nullptr };
    std::unique_ptr<FeedbacksMap> feedbacks_map_{ nullptr };
};
