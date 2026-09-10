#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

class SerialPort
{
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& port_name, std::uint32_t baud);
    void close();

    // Discard pending RX and TX bytes.
    // Use at startup only.
    bool clearBuffers();

    // Transfer all requested bytes; return false on a reported error or timeout.
    // Linux writes are blocking with no explicit timeout.
    bool writeExact(const void* data, std::size_t size);
    bool readExact(void* data, std::size_t size);

private:
#ifdef _WIN32
    void* handle_ = reinterpret_cast<void*>(-1);
#else
    int fd_ = -1;
#endif
};
