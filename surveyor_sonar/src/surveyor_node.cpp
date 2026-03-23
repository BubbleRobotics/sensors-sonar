#include "surveyor_sonar/surveyor_node.hpp"

#include <arpa/inet.h>
#include <unistd.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

SurveyorNode::SurveyorNode()
: Node("surveyor_node")
{
    this->declare_parameter("ip", "192.168.2.86");
    this->declare_parameter("port", 62312);

    ip_ = this->get_parameter("ip").as_string();
    port_ = this->get_parameter("port").as_int();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "sonar/points", 10);

    connectSocket();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&SurveyorNode::readSocket, this));
}

void SurveyorNode::connectSocket()
{
    sock_ = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &server.sin_addr);

    connect(sock_, (sockaddr*)&server, sizeof(server));
}

void SurveyorNode::readSocket()
{
    uint8_t buffer[4096];
    ssize_t len = recv(sock_, buffer, sizeof(buffer), 0);

    if (len <= 0)
        return;

    parser_.append(buffer, len);

    std::vector<uint8_t> packet;

    while (parser_.getNextPacket(packet))
    {
        std::vector<YZPoint> points;

        if (parseYZ(packet, points))
        {
            publishYZ(points);
        }
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
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurveyorNode>());
    rclcpp::shutdown();
    return 0;
}