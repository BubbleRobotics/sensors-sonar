#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

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
    ~SurveyorNode() override;

private:
    void connectSocket();
    void closeSocket();
    void scheduleReconnect();
    bool updateConnectionState();
    bool sendPacket(uint16_t packet_id, const std::vector<uint8_t>& payload);
    bool sendPingParameters(bool ping_enable);
    void readSocket();
    void publishDistance(const std::vector<YZPoint>& points);
    void publishYZ(const std::vector<YZPoint>& points);

    int sock_;
    bool connected_;
    bool ping_config_sent_;
    std::string ip_;
    int port_;
    std::string frame_id_;
    std::chrono::steady_clock::time_point next_connect_attempt_;
    int start_mm_;
    int end_mm_;
    double sos_mps_;
    int gain_index_;
    int msec_per_ping_;
    bool enable_atof_data_;
    int n_range_steps_;
    double pulse_len_steps_;
    double distance_fov_deg_;
    size_t packets_seen_;
    size_t decode_failures_;
    size_t published_clouds_;
    size_t last_reported_checksum_failures_;
    size_t last_reported_sync_losses_;
    uint16_t last_msg_id_;
    uint16_t last_payload_len_;

    PingParser parser_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
