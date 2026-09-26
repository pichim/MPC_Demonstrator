#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace protocol {
using Frame = std::array<uint8_t, 18>;
using Measurements = std::array<float, 4>;
constexpr uint8_t Read = 0x57, Command = 0x55, Reply = 0x45;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

constexpr auto make_crc_table()
{
    std::array<uint8_t, 256> table{};
    for (unsigned i = 0; i < table.size(); ++i) {
        unsigned value = i;
        for (int bit = 0; bit < 8; ++bit)
            value = (value & 0x80) ? (value << 1) ^ 0x07 : value << 1;
        table[i] = static_cast<uint8_t>(value);
    }
    return table;
}
inline constexpr auto crc_table = make_crc_table();
inline uint8_t crc8(const Frame &frame)
{
    uint8_t crc = 0;
    for (unsigned i = 0; i < frame.size() - 1; ++i)
        crc = crc_table[crc ^ frame[i]];
    return crc;
}
inline Frame frame(uint8_t header, Measurements values = {})
{
    Frame result{};
    result[0] = header;
    for (unsigned i = 0; i < values.size(); ++i) {
        uint32_t bits;
        std::memcpy(&bits, &values[i], sizeof(bits));
        for (unsigned byte = 0; byte < 4; ++byte)
            result[1 + 4 * i + byte] = static_cast<uint8_t>(bits >> (8 * byte));
    }
    result.back() = crc8(result);
    return result;
}
inline Measurements decode(const Frame &frame)
{
    if (frame[0] != Reply || frame.back() != crc8(frame))
        throw std::runtime_error("Invalid SPI reply (header/CRC)");
    Measurements values{};
    for (unsigned i = 0; i < values.size(); ++i) {
        uint32_t bits = 0;
        for (unsigned byte = 0; byte < 4; ++byte)
            bits |= uint32_t(frame[1 + 4 * i + byte]) << (8 * byte);
        std::memcpy(&values[i], &bits, sizeof(bits));
        if (!std::isfinite(values[i]))
            throw std::runtime_error("Non-finite SPI measurement");
    }
    return values;
}
} // namespace protocol
