#include "serial_port.h"

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
    case 230400: return B230400;
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
