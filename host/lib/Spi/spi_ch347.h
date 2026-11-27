#pragma once

#include <libusb.h>

#include "protocol.h"

class Ch347SPI
{
public:
    explicit Ch347SPI(unsigned speed);
    ~Ch347SPI();
    Ch347SPI(const Ch347SPI &) = delete;
    Ch347SPI &operator=(const Ch347SPI &) = delete;

    protocol::Frame transfer(const protocol::Frame &frame);

private:
    void close() noexcept;
    void select(bool active);
    void write(uint8_t *data, int size);
    int read(uint8_t *data, int capacity);

    libusb_context *context_ = nullptr;
    libusb_device_handle *device_ = nullptr;
    bool claimed_ = false;
    bool reply_synchronized_ = true;
    long long released_at_ = 0;
};
