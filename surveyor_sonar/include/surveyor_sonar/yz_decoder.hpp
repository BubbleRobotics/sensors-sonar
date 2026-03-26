#pragma once
#include <vector>
#include <cstdint>

/**
 * @brief Single sonar point in YZ plane derived from ATOF data
 */
struct YZPoint
{
    float y;
    float z;
};

/**
 * @brief Decode Surveyor ATOF packets into Y/Z points
 *
 * Converts ATOF_POINT_DATA (3012) into Y/Z coordinates suitable for
 * publishing as a PointCloud2.
 *
 * @param packet Full Ping packet (header + payload)
 * @param points_out Output vector
 * @return true if packet was a valid ATOF point-data message
 */
bool parseYZ(const std::vector<uint8_t>& packet,
             std::vector<YZPoint>& points_out);
