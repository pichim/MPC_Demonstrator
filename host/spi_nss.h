#pragma once

#include "config.h"
#include "protocol.h"
#include <cerrno>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <system_error>
#include <time.h>
#include <unistd.h>

inline long long monotonic_ns() {
    timespec time{};
    if (clock_gettime(CLOCK_MONOTONIC, &time) < 0)
        throw std::system_error(errno, std::generic_category(), "clock_gettime");
    return time.tv_sec * 1'000'000'000LL + time.tv_nsec;
}

// One ioctl asserts NSS, waits in the kernel without clocking bytes, exchanges
// the payload, then releases NSS. No REQUEST/READY GPIO access is needed.
class NssSPI {
public:
    explicit NssSPI(unsigned speed) : speed_(speed) {
        fd_ = open(config::spi_device, O_RDWR | O_CLOEXEC);
        if (fd_ < 0) fail("open SPI");
        uint8_t mode = SPI_MODE_0, bits = 8;
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
            const int error = errno;
            ::close(fd_);
            fd_ = -1;
            errno = error;
            fail("configure SPI");
        }
    }
    ~NssSPI() { if (fd_ >= 0) ::close(fd_); }
    NssSPI(const NssSPI&) = delete;
    NssSPI& operator=(const NssSPI&) = delete;

    protocol::Frame transfer(const protocol::Frame& tx) {
        protocol::Frame rx{};
        transfer_raw(tx.data(), rx.data(), tx.size());
        return rx;
    }

    // Variable lengths are used only by the disabled-command fault test.
    void transfer_raw(const uint8_t* tx, uint8_t* rx, unsigned length) {
        exchange(tx, rx, length, config::nss_setup_us);
    }

    // Assert NSS without clocks; optionally hold past the MCU's 20 ms timeout.
    void cancel_selection(bool wait_for_timeout) {
        exchange(nullptr, nullptr, 0, wait_for_timeout ? 35000 : config::nss_setup_us);
    }

private:
    [[noreturn]] static void fail(const char* operation) {
        throw std::system_error(errno, std::generic_category(), operation);
    }

    void exchange(const uint8_t* tx, uint8_t* rx, unsigned length, unsigned setup_us) {
        // ioctl returns after NSS release. This enforces a minimum high interval,
        // including between an immediate READ and COMMAND with no computation.
        while (monotonic_ns() - last_end_ < config::nss_inactive_us * 1000LL) {}
        spi_ioc_transfer transfers[2]{};
        transfers[0].delay_usecs = setup_us; // zero bytes; CS stays asserted
        transfers[1].tx_buf = reinterpret_cast<uintptr_t>(tx);
        transfers[1].rx_buf = reinterpret_cast<uintptr_t>(rx);
        transfers[1].len = length;
        transfers[1].speed_hz = speed_;
        transfers[1].bits_per_word = 8;
        // Do not retry EINTR: SPI clocks might already have been generated.
        const int result = ioctl(fd_, SPI_IOC_MESSAGE(2), transfers);
        const int error = errno;
        last_end_ = monotonic_ns();
        if (result < 0) {
            errno = error;
            fail("SPI exchange");
        }
        if (static_cast<unsigned>(result) != length)
            throw std::runtime_error("Short SPI exchange");
    }

    int fd_ = -1;
    unsigned speed_;
    long long last_end_ = 0;
};
