#include "surveyor_sonar/ping_parser.hpp"

void PingParser::append(const uint8_t* data, size_t len)
{
    buffer_.insert(buffer_.end(), data, data + len);
}

bool PingParser::getNextPacket(std::vector<uint8_t>& packet)
{
    while (buffer_.size() >= 8)
    {
        if (!(buffer_[0] == 0x42 && buffer_[1] == 0x52))
        {
            buffer_.erase(buffer_.begin());
            continue;
        }

        uint16_t payload_len = *(uint16_t*)(&buffer_[2]);
        size_t total_len = 8 + payload_len;

        if (buffer_.size() < total_len)
            return false;

        packet.assign(buffer_.begin(), buffer_.begin() + total_len);
        buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);

        return true;
    }

    return false;
}