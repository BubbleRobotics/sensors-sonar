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
 * @brief Decode Surveyor point packets into Y/Z points
 *
 * Supports native YZ_POINT_DATA (3011) and ATOF_POINT_DATA (3012),
 * converting the latter into Y/Z coordinates.
 *
 * @param packet Full Ping packet (header + payload)
 * @param points_out Output vector
 * @return true if packet was a supported point-data message
 */
bool parseYZ(const std::vector<uint8_t>& packet,
             std::vector<YZPoint>& points_out);
