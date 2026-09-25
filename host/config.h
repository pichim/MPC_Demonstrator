#pragma once

namespace config {
constexpr unsigned nss_setup_us = 30;
constexpr unsigned nss_inactive_us = 30;
inline constexpr char spi_device[] = "/dev/spidev0.0";
inline constexpr unsigned spi_speed = 30'000'000;
} // namespace config
