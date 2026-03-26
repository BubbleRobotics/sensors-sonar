#include "surveyor_sonar/surveyor_node.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
uint16_t readU16LE(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

void appendU8(std::vector<uint8_t>& payload, uint8_t value)
{
    payload.push_back(value);
}

void appendU16LE(std::vector<uint8_t>& payload, uint16_t value)
{
    payload.push_back(static_cast<uint8_t>(value & 0xffU));
    payload.push_back(static_cast<uint8_t>((value >> 8) & 0xffU));
}

void appendI16LE(std::vector<uint8_t>& payload, int16_t value)
{
    appendU16LE(payload, static_cast<uint16_t>(value));
}

void appendU32LE(std::vector<uint8_t>& payload, uint32_t value)
{
    payload.push_back(static_cast<uint8_t>(value & 0xffU));
    payload.push_back(static_cast<uint8_t>((value >> 8) & 0xffU));
    payload.push_back(static_cast<uint8_t>((value >> 16) & 0xffU));
    payload.push_back(static_cast<uint8_t>((value >> 24) & 0xffU));
}

void appendI32LE(std::vector<uint8_t>& payload, int32_t value)
{
    appendU32LE(payload, static_cast<uint32_t>(value));
}

void appendFloatLE(std::vector<uint8_t>& payload, float value)
{
    static_assert(sizeof(float) == sizeof(uint32_t), "Unexpected float size");
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    appendU32LE(payload, bits);
}

uint16_t computeChecksum(const uint8_t* data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i)
    {
        sum += data[i];
    }
    return static_cast<uint16_t>(sum & 0xffffU);
}

constexpr uint16_t kNopPacketId = 0;
constexpr uint16_t kAckPacketId = 1;
constexpr uint16_t kNackPacketId = 2;
constexpr uint16_t kSetPingParametersPacketId = 3023;
}  // namespace

SurveyorNode::SurveyorNode()
: Node("surveyor_node"),
  sock_(-1),
  connected_(false),
  ping_config_sent_(false),
  next_connect_attempt_(std::chrono::steady_clock::now()),
  start_mm_(this->declare_parameter("start_mm", 0)),
  end_mm_(this->declare_parameter("end_mm", -10000)),
  sos_mps_(this->declare_parameter("sos_mps", 1500.0)),
  gain_index_(this->declare_parameter("gain_index", -1)),
  msec_per_ping_(this->declare_parameter("msec_per_ping", 100)),
  enable_yz_point_data_(this->declare_parameter("enable_yz_point_data", false)),
  enable_atof_data_(this->declare_parameter("enable_atof_data", true)),
  n_range_steps_(this->declare_parameter("n_range_steps", 400)),
  pulse_len_steps_(this->declare_parameter("pulse_len_steps", 1.5)),
  packets_seen_(0),
  decode_failures_(0),
  published_clouds_(0),
  last_reported_checksum_failures_(0),
  last_reported_sync_losses_(0),
  last_msg_id_(0),
  last_payload_len_(0)
{
    this->declare_parameter("ip", "192.168.2.86");
    this->declare_parameter("port", 62312);

    ip_ = this->get_parameter("ip").as_string();
    port_ = this->get_parameter("port").as_int();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "sonar/points", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&SurveyorNode::readSocket, this));
}

SurveyorNode::~SurveyorNode()
{
    closeSocket();
}

void SurveyorNode::connectSocket()
{
    if (sock_ >= 0)
    {
        return;
    }

    if (std::chrono::steady_clock::now() < next_connect_attempt_)
    {
        return;
    }

    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "socket() failed: %s", std::strerror(errno));
        scheduleReconnect();
        return;
    }

    const int flags = fcntl(sock_, F_GETFL, 0);
    if (flags < 0 || fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to make socket non-blocking: %s",
                    std::strerror(errno));
        closeSocket();
        scheduleReconnect();
        return;
    }

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(port_);
    if (inet_pton(AF_INET, ip_.c_str(), &server.sin_addr) != 1)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid Surveyor IP address: '%s'", ip_.c_str());
        closeSocket();
        scheduleReconnect();
        return;
    }

    if (connect(sock_, reinterpret_cast<sockaddr*>(&server), sizeof(server)) == 0)
    {
        connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Connected to Surveyor at %s:%d",
                    ip_.c_str(), port_);
        if (!sendPingParameters(true))
        {
            closeSocket();
            scheduleReconnect();
        }
        return;
    }

    if (errno == EINPROGRESS)
    {
        return;
    }

    RCLCPP_WARN(this->get_logger(), "connect() to %s:%d failed: %s",
                ip_.c_str(), port_, std::strerror(errno));
    closeSocket();
    scheduleReconnect();
}

void SurveyorNode::closeSocket()
{
    if (sock_ >= 0 && connected_ && ping_config_sent_)
    {
        RCLCPP_INFO(this->get_logger(), "Sending Surveyor stop command before disconnect");
        if (!sendPingParameters(false))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send Surveyor stop command before disconnect");
        }
    }

    if (sock_ >= 0)
    {
        close(sock_);
    }

    sock_ = -1;
    connected_ = false;
    ping_config_sent_ = false;
    parser_.reset();
}

void SurveyorNode::scheduleReconnect()
{
    next_connect_attempt_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
}

bool SurveyorNode::updateConnectionState()
{
    if (sock_ < 0)
    {
        return false;
    }

    if (connected_)
    {
        return true;
    }

    pollfd poll_fd{};
    poll_fd.fd = sock_;
    poll_fd.events = POLLOUT;

    const int poll_result = poll(&poll_fd, 1, 0);
    if (poll_result == 0)
    {
        return false;
    }

    if (poll_result < 0)
    {
        if (errno == EINTR)
        {
            return false;
        }

        RCLCPP_WARN(this->get_logger(), "poll() during connect failed: %s",
                    std::strerror(errno));
        closeSocket();
        scheduleReconnect();
        return false;
    }

    int socket_error = 0;
    socklen_t socket_error_len = sizeof(socket_error);
    if (getsockopt(sock_, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) < 0)
    {
        RCLCPP_WARN(this->get_logger(), "getsockopt(SO_ERROR) failed: %s",
                    std::strerror(errno));
        closeSocket();
        scheduleReconnect();
        return false;
    }

    if (socket_error != 0)
    {
        RCLCPP_WARN(this->get_logger(), "Surveyor connection failed: %s",
                    std::strerror(socket_error));
        closeSocket();
        scheduleReconnect();
        return false;
    }

    connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to Surveyor at %s:%d",
                ip_.c_str(), port_);
    if (!sendPingParameters(true))
    {
        closeSocket();
        scheduleReconnect();
        return false;
    }
    return true;
}

bool SurveyorNode::sendPacket(uint16_t packet_id, const std::vector<uint8_t>& payload)
{
    if (sock_ < 0)
    {
        return false;
    }

    std::vector<uint8_t> packet;
    packet.reserve(8 + payload.size() + 2);
    packet.push_back(0x42);
    packet.push_back(0x52);
    appendU16LE(packet, static_cast<uint16_t>(payload.size()));
    appendU16LE(packet, packet_id);
    packet.push_back(0);
    packet.push_back(0);
    packet.insert(packet.end(), payload.begin(), payload.end());
    appendU16LE(packet, computeChecksum(packet.data(), packet.size()));

    size_t sent = 0;
    while (sent < packet.size())
    {
        const ssize_t written = send(sock_,
                                     packet.data() + sent,
                                     packet.size() - sent,
                                     MSG_NOSIGNAL);
        if (written > 0)
        {
            sent += static_cast<size_t>(written);
            continue;
        }

        if (written < 0 && errno == EINTR)
        {
            continue;
        }

        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            pollfd poll_fd{};
            poll_fd.fd = sock_;
            poll_fd.events = POLLOUT;
            const int poll_result = poll(&poll_fd, 1, 1000);
            if (poll_result > 0)
            {
                continue;
            }

            if (poll_result == 0)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Timed out while sending packet %u to Surveyor", packet_id);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "poll() during send failed: %s", std::strerror(errno));
            }
            return false;
        }

        RCLCPP_WARN(this->get_logger(),
                    "send() failed for packet %u: %s", packet_id, std::strerror(errno));
        return false;
    }

    return true;
}

bool SurveyorNode::sendPingParameters(bool ping_enable)
{
    std::vector<uint8_t> payload;
    payload.reserve(36);

    appendI32LE(payload, static_cast<int32_t>(start_mm_));
    appendI32LE(payload, static_cast<int32_t>(end_mm_));
    appendFloatLE(payload, static_cast<float>(sos_mps_));
    appendI16LE(payload, static_cast<int16_t>(gain_index_));
    appendI16LE(payload, static_cast<int16_t>(msec_per_ping_));
    appendU16LE(payload, 0);  // deprecated pulse width
    appendU8(payload, 0);     // diagnostic injected signal
    appendU8(payload, ping_enable ? 1U : 0U);
    appendU8(payload, 0);     // enable channel data
    appendU8(payload, 0);     // reserved raw data
    appendU8(payload, enable_yz_point_data_ ? 1U : 0U);
    appendU8(payload, enable_atof_data_ ? 1U : 0U);
    appendI32LE(payload, 240000);
    appendU16LE(payload, static_cast<uint16_t>(n_range_steps_));
    appendU16LE(payload, 0);  // reserved
    appendFloatLE(payload, static_cast<float>(pulse_len_steps_));

    RCLCPP_INFO(this->get_logger(),
                "Sending SET_PING_PARAMETERS: ping_enable=%s start_mm=%d end_mm=%d sos_mps=%.1f gain_index=%d msec_per_ping=%d yz=%s atof=%s n_range_steps=%d pulse_len_steps=%.2f",
                ping_enable ? "true" : "false",
                start_mm_,
                end_mm_,
                sos_mps_,
                gain_index_,
                msec_per_ping_,
                enable_yz_point_data_ ? "true" : "false",
                enable_atof_data_ ? "true" : "false",
                n_range_steps_,
                pulse_len_steps_);

    const bool send_ok = sendPacket(kSetPingParametersPacketId, payload);
    ping_config_sent_ = send_ok && ping_enable;
    return send_ok;
}

void SurveyorNode::readSocket()
{
    if (sock_ < 0)
    {
        connectSocket();
        return;
    }

    if (!updateConnectionState())
    {
        return;
    }

    uint8_t buffer[4096];
    bool received_any_bytes = false;
    size_t total_bytes_received = 0;
    while (true)
    {
        const ssize_t len = recv(sock_, buffer, sizeof(buffer), 0);

        if (len > 0)
        {
            received_any_bytes = true;
            total_bytes_received += static_cast<size_t>(len);
            parser_.append(buffer, static_cast<size_t>(len));
            continue;
        }

        if (len == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Surveyor connection closed by peer");
            closeSocket();
            scheduleReconnect();
            return;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            break;
        }

        if (errno == EINTR)
        {
            continue;
        }

        RCLCPP_WARN(this->get_logger(), "recv() failed: %s", std::strerror(errno));
        closeSocket();
        scheduleReconnect();
        return;
    }

    if (parser_.checksumFailures() != last_reported_checksum_failures_ ||
        parser_.syncLosses() != last_reported_sync_losses_)
    {
        last_reported_checksum_failures_ = parser_.checksumFailures();
        last_reported_sync_losses_ = parser_.syncLosses();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Surveyor parser discarded data: buffered=%zu sync_losses=%zu checksum_failures=%zu",
            parser_.bufferedSize(),
            parser_.syncLosses(),
            parser_.checksumFailures());
    }

    std::vector<uint8_t> packet;
    bool extracted_packet = false;
    while (parser_.getNextPacket(packet))
    {
        extracted_packet = true;
        ++packets_seen_;
        last_payload_len_ = packet.size() >= 4 ? readU16LE(packet.data() + 2) : 0;
        last_msg_id_ = packet.size() >= 6 ? readU16LE(packet.data() + 4) : 0;

        if (last_msg_id_ == kNopPacketId)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Surveyor sent NOP packets while idle/alive (packets=%zu)",
                packets_seen_);
            continue;
        }

        if (last_msg_id_ == kAckPacketId && last_payload_len_ >= 2)
        {
            const uint16_t acked_id = readU16LE(packet.data() + 8);
            RCLCPP_INFO(this->get_logger(), "Surveyor ACK for packet %u", acked_id);
            continue;
        }

        if (last_msg_id_ == kNackPacketId && last_payload_len_ >= 2)
        {
            const uint16_t nacked_id = readU16LE(packet.data() + 8);
            RCLCPP_WARN(this->get_logger(), "Surveyor NACK for packet %u", nacked_id);
            continue;
        }

        std::vector<YZPoint> points;
        if (parseYZ(packet, points))
        {
            publishYZ(points);
        }
        else
        {
            if (last_msg_id_ != 3011 && last_msg_id_ != 3012)
            {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Ignoring non-point Surveyor packet: msg_id=%u payload_len=%u packets=%zu published=%zu",
                    last_msg_id_,
                    last_payload_len_,
                    packets_seen_,
                    published_clouds_);
                continue;
            }

            ++decode_failures_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Surveyor packet not decoded: msg_id=%u payload_len=%u packets=%zu decode_failures=%zu published=%zu",
                last_msg_id_,
                last_payload_len_,
                packets_seen_,
                decode_failures_,
                published_clouds_);
        }
    }

    if (received_any_bytes && !extracted_packet)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Surveyor received %zu bytes but no full packet yet: buffered=%zu sync_losses=%zu checksum_failures=%zu",
            total_bytes_received,
            parser_.bufferedSize(),
            parser_.syncLosses(),
            parser_.checksumFailures());
    }

    if (extracted_packet)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Surveyor traffic: last_msg_id=%u payload_len=%u packets=%zu decode_failures=%zu published=%zu buffered=%zu",
            last_msg_id_,
            last_payload_len_,
            packets_seen_,
            decode_failures_,
            published_clouds_,
            parser_.bufferedSize());
    }
}

void SurveyorNode::publishYZ(const std::vector<YZPoint>& points)
{
    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.stamp = now();
    cloud.header.frame_id = "sonar_frame";

    cloud.height = 1;
    cloud.width = points.size();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z)
    {
        *iter_x = 0.0f;
        *iter_y = points[i].y;
        *iter_z = points[i].z;
    }

    pub_->publish(cloud);
    ++published_clouds_;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Published Surveyor cloud with %zu points on sonar/points (published=%zu)",
        points.size(),
        published_clouds_);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurveyorNode>());
    rclcpp::shutdown();
    return 0;
}
