#include "surveyor_sonar/yz_decoder.hpp"

bool parseYZ(const std::vector<uint8_t>& packet,
             std::vector<YZPoint>& points_out)
{
    if (packet.size() < 8)
    {
        std::cout << "FAIL, Smaller 8" << std::endl;
        return false;
    }

    uint16_t msg_id = *(uint16_t*)(&packet[4]);
    // if (msg_id != 3012)
    // {
    //     std::cout << "FAIL, message ID" << std::endl;
    //     return false;
    // }
    const uint8_t* payload = packet.data() + 8;
    size_t offset = 0;

    offset += 4; // timestamp
    offset += 4; // ping number

    offset += 4 * 3;
    offset += 4 * 3;
    offset += 4 * 3;
    offset += 4 * 10;

    offset += 4 * 8;

    offset += 2;

    uint16_t num_points = *(uint16_t*)(payload + offset);
    offset += 2;

    // if (offset + num_points * 2 * sizeof(float) > packet.size())
    // {
    //     std::cout << "FAIL, Offset" << std::endl;
    //     return false;
    // }

    points_out.clear();
    points_out.reserve(num_points);

    for (uint16_t i = 0; i < num_points; i++)
    {
        float y = *(float*)(payload + offset); offset += 4;
        float z = *(float*)(payload + offset); offset += 4;

        points_out.push_back({y, z});
    }

    return true;
}