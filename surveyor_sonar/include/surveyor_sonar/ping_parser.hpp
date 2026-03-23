#pragma once
#include <vector>
#include <cstdint>

/**
 * @brief Parses Ping protocol byte stream into full packets.
 *
 * Handles:
 * - TCP fragmentation
 * - packet synchronization ('B','R')
 * - payload length extraction
 *
 * Usage:
 *   append() incoming bytes
 *   repeatedly call getNextPacket()
 */
class PingParser
{
public:
    void append(const uint8_t* data, size_t len);

    bool getNextPacket(std::vector<uint8_t>& packet);

private:
    std::vector<uint8_t> buffer_;
};