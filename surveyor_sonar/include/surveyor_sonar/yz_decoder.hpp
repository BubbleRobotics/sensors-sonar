#pragma once
#include <vector>
#include <cstdint>

/**
 * @brief Single sonar point in YZ plane
 */
struct YZPoint
{
    float y;
    float z;
};

/**
 * @brief Decode YZ_POINT_DATA (message ID 3012)
 *
 * Converts raw Ping protocol payload into vector of points.
 *
 * @param packet Full Ping packet (header + payload)
 * @param points_out Output vector
 * @return true if packet was valid YZ data
 */
bool parseYZ(const std::vector<uint8_t>& packet,
             std::vector<YZPoint>& points_out);