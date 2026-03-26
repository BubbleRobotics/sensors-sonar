#include "surveyor_sonar/ping_parser.hpp"

namespace
{
constexpr size_t kHeaderSize = 8;
constexpr size_t kChecksumSize = 2;

uint16_t readU16LE(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
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
}  // namespace

void PingParser::append(const uint8_t* data, size_t len)
{
    buffer_.insert(buffer_.end(), data, data + len);
}

bool PingParser::getNextPacket(std::vector<uint8_t>& packet)
{
    while (buffer_.size() >= kHeaderSize)
    {
        if (!(buffer_[0] == 0x42 && buffer_[1] == 0x52))
        {
            ++sync_losses_;
            buffer_.erase(buffer_.begin());
            continue;
        }

        const uint16_t payload_len = readU16LE(&buffer_[2]);
        const size_t total_len = kHeaderSize + payload_len + kChecksumSize;

        if (buffer_.size() < total_len)
            return false;

        const uint16_t expected_checksum =
            computeChecksum(buffer_.data(), kHeaderSize + payload_len);
        const uint16_t actual_checksum =
            readU16LE(buffer_.data() + kHeaderSize + payload_len);

        if (expected_checksum != actual_checksum)
        {
            ++checksum_failures_;
            buffer_.erase(buffer_.begin());
            continue;
        }

        packet.assign(buffer_.begin(), buffer_.begin() + kHeaderSize + payload_len);
        buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);

        return true;
    }

    return false;
}

void PingParser::reset()
{
    buffer_.clear();
}

size_t PingParser::bufferedSize() const
{
    return buffer_.size();
}

size_t PingParser::checksumFailures() const
{
    return checksum_failures_;
}

size_t PingParser::syncLosses() const
{
    return sync_losses_;
}
