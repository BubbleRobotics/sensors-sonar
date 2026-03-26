#include "surveyor_sonar/yz_decoder.hpp"

#include <cmath>
#include <cstring>

namespace
{
constexpr size_t kHeaderSize = 8;
constexpr uint16_t kATOFPointDataMessageId = 3012;
constexpr size_t kATOFMinimumPayloadSize = 40;

uint16_t readU16LE(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readU32LE(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

float readFloatLE(const uint8_t* data)
{
    const uint32_t bits = readU32LE(data);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
}  // namespace

bool parseYZ(const std::vector<uint8_t>& packet,
             std::vector<YZPoint>& points_out)
{
    if (packet.size() < kHeaderSize)
    {
        return false;
    }

    const uint16_t payload_len = readU16LE(packet.data() + 2);
    if (packet.size() != kHeaderSize + payload_len)
    {
        return false;
    }

    const uint16_t msg_id = readU16LE(packet.data() + 4);
    const uint8_t* payload = packet.data() + kHeaderSize;

    if (msg_id != kATOFPointDataMessageId)
    {
        return false;
    }

    if (payload_len < kATOFMinimumPayloadSize)
    {
        return false;
    }

    size_t offset = 0;
    offset += 4;  // pwr_up_msec
    offset += 8;  // utc_msec
    offset += 4;  // listening_sec
    const float sos_mps = readFloatLE(payload + offset);
    offset += 4;
    offset += 4;  // ping_number
    offset += 4;  // ping_hz
    offset += 4;  // pulse_sec
    offset += 4;  // flags

    const uint16_t num_points = readU16LE(payload + offset);
    offset += 2;
    offset += 2;  // reserved

    const size_t points_bytes = static_cast<size_t>(num_points) * 16U;
    if (offset + points_bytes > payload_len)
    {
        return false;
    }

    points_out.clear();
    points_out.reserve(num_points);

    for (uint16_t i = 0; i < num_points; ++i)
    {
        const float angle_rad = readFloatLE(payload + offset);
        offset += 4;
        const float tof_sec = readFloatLE(payload + offset);
        offset += 4;
        offset += 8;  // reserved[2]

        const float distance_m = 0.5F * sos_mps * tof_sec;
        const float y = distance_m * std::sin(angle_rad);
        const float z = -distance_m * std::cos(angle_rad);
        points_out.push_back({y, z});
    }

    return true;
}
