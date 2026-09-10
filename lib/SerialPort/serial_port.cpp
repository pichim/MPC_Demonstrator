#include "serial_port.h"

#include <cstdint>

#ifdef _WIN32

#include <windows.h>

namespace
{
constexpr DWORD READ_TIMEOUT_MS = 20;

HANDLE nativeHandle(void* handle)
{
    return static_cast<HANDLE>(handle);
}

std::string nativePortName(const std::string& port_name)
{
    if (port_name.rfind("\\\\.\\", 0) == 0)
        return port_name;

    return "\\\\.\\" + port_name;
}
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& port_name, std::uint32_t baud)
{
    close();

    HANDLE handle = CreateFileA(
        nativePortName(port_name).c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        nullptr,
        OPEN_EXISTING,
        0,
        nullptr);

    if (handle == INVALID_HANDLE_VALUE)
        return false;

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return false;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    // Do not inherit settings that discard or replace binary packet bytes.
    dcb.fDsrSensitivity = FALSE;
    dcb.fNull = FALSE;
    dcb.fErrorChar = FALSE;
    // Report driver errors; the caller stops communication on failure.
    dcb.fAbortOnError = TRUE;

    if (!SetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return false;
    }

    COMMTIMEOUTS timeouts{};
    // Wait for the requested bytes, with a 20 ms total timeout per ReadFile call.
    timeouts.ReadTotalTimeoutConstant = READ_TIMEOUT_MS;
    timeouts.WriteTotalTimeoutConstant = READ_TIMEOUT_MS;

    if (!SetCommTimeouts(handle, &timeouts)) {
        CloseHandle(handle);
        return false;
    }

    handle_ = handle;
    return true;
}

void SerialPort::close()
{
    HANDLE handle = nativeHandle(handle_);

    if (handle != INVALID_HANDLE_VALUE) {
        CloseHandle(handle);
        handle_ = reinterpret_cast<void*>(-1);
    }
}

bool SerialPort::clearBuffers()
{
    HANDLE handle = nativeHandle(handle_);

    return handle != INVALID_HANDLE_VALUE &&
           PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR) != 0;
}

bool SerialPort::writeExact(const void* data, std::size_t size)
{
    HANDLE handle = nativeHandle(handle_);
    if (handle == INVALID_HANDLE_VALUE)
        return false;

    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::size_t total = 0;

    while (total < size) {
        DWORD written = 0;

        if (!WriteFile(
                handle,
                bytes + total,
                static_cast<DWORD>(size - total),
                &written,
                nullptr))
            return false;

        if (written == 0)
            return false;

        total += written;
    }

    return true;
}

bool SerialPort::readExact(void* data, std::size_t size)
{
    HANDLE handle = nativeHandle(handle_);
    if (handle == INVALID_HANDLE_VALUE)
        return false;

    auto* bytes = static_cast<std::uint8_t*>(data);
    std::size_t total = 0;

    while (total < size) {
        DWORD nread = 0;

        if (!ReadFile(
                handle,
                bytes + total,
                static_cast<DWORD>(size - total),
                &nread,
                nullptr))
            return false;

        if (nread == 0)
            return false; // timeout

        total += nread;
    }

    return true;
}

#else

#include <cerrno>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace
{
constexpr int READ_TIMEOUT_MS = 20;

speed_t nativeBaud(std::uint32_t baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return 0;
    }
}
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& port_name, std::uint32_t baud)
{
    close();

    const speed_t speed = nativeBaud(baud);
    if (speed == 0)
        return false;

    const int fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
        return false;

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);
    // cfmakeraw clears IXON, but leaves these other flow-control flags intact.
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    if (cfsetispeed(&tty, speed) != 0 ||
        cfsetospeed(&tty, speed) != 0) {
        ::close(fd);
        return false;
    }

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return false;
    }

    fd_ = fd;
    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::clearBuffers()
{
    return fd_ >= 0 && tcflush(fd_, TCIOFLUSH) == 0;
}

bool SerialPort::writeExact(const void* data, std::size_t size)
{
    if (fd_ < 0)
        return false;

    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::size_t total = 0;

    while (total < size) {
        const ssize_t written = ::write(fd_, bytes + total, size - total);

        if (written < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }

        if (written == 0)
            return false;

        total += static_cast<std::size_t>(written);
    }

    return true;
}

bool SerialPort::readExact(void* data, std::size_t size)
{
    if (fd_ < 0)
        return false;

    auto* bytes = static_cast<std::uint8_t*>(data);
    std::size_t total = 0;

    while (total < size) {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        const int result = poll(&pfd, 1, READ_TIMEOUT_MS);

        if (result < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }

        if (result == 0)
            return false; // timeout

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            return false;

        const ssize_t nread = ::read(fd_, bytes + total, size - total);

        if (nread < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }

        if (nread == 0)
            continue;

        total += static_cast<std::size_t>(nread);
    }

    return true;
}

#endif
