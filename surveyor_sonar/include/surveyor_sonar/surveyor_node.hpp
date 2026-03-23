#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ping_parser.hpp"
#include "yz_decoder.hpp"

/**
 * @brief ROS2 driver node for Cerulean Surveyor sonar
 *
 * Responsibilities:
 * - TCP connection to sonar
 * - packet parsing (Ping protocol)
 * - decoding sonar messages
 * - publishing PointCloud2
 */
class SurveyorNode : public rclcpp::Node
{
public:
    SurveyorNode();

private:
    void connectSocket();
    void readSocket();
    void publishYZ(const std::vector<YZPoint>& points);

    int sock_;
    std::string ip_;
    int port_;

    PingParser parser_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};