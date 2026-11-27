#include "spi_ch347.h"

#include <algorithm>
#include <array>
#include <string>

#include "monotonic_clock.h"

namespace {
// CH347T mode 1, as exposed by WCH's Linux SPI driver.
constexpr unsigned Vendor = 0x1a86, Product = 0x55db, UartProduct = 0x55da, HidProduct = 0x55dc;
constexpr int Interface = 2;
constexpr uint8_t Out = 0x06, In = 0x86;
constexpr uint8_t Configure = 0xc0, ChipSelect = 0xc1, Exchange = 0xc2;
constexpr unsigned UsbTimeoutMs = 10;
constexpr uint8_t SetupUs = 30, InactiveUs = 30;
constexpr size_t UsbHeaderSize = 3;
constexpr std::array<unsigned, 7> Speeds{30'000'000, 15'000'000, 7'500'000, 3'750'000, 1'875'000, 937'500, 468'750};

void check_usb(int status, const char *operation)
{
    if (status < 0)
        throw std::runtime_error(std::string(operation) + ": " + libusb_error_name(status));
}

void wait_until(long long deadline)
{
    while (monotonic_ns() < deadline) {
    }
}
} // namespace

Ch347SPI::Ch347SPI(unsigned speed)
{
    const auto setting = std::find(Speeds.begin(), Speeds.end(), speed);
    if (setting == Speeds.end())
        throw std::invalid_argument("Unsupported CH347T SPI speed");

    check_usb(libusb_init(&context_), "Initialize USB");
    try {
        libusb_device **devices = nullptr;
        check_usb(libusb_get_device_list(context_, &devices), "List USB devices");
        libusb_device *adapter = nullptr;
        bool other_mode = false, multiple = false;
        for (auto entry = devices; *entry; ++entry) {
            libusb_device_descriptor id{};
            if (libusb_get_device_descriptor(*entry, &id) != 0 || id.idVendor != Vendor)
                continue;
            if (id.idProduct == Product) {
                multiple = adapter != nullptr;
                adapter = *entry;
            }
            other_mode |= id.idProduct == UartProduct || id.idProduct == HidProduct;
        }
        const bool found = adapter != nullptr;
        const int open_status = found && !multiple ? libusb_open(adapter, &device_) : 0;
        libusb_free_device_list(devices, 1);
        if (multiple)
            throw std::runtime_error("Connect only one CH347T in mode M1");
        if (!found)
            throw std::runtime_error(other_mode ? "CH347T detected in M0 or M2. Set M1 with USB unplugged."
                                                : "CH347T in mode M1 (1a86:55db) was not found");
        check_usb(open_status, "Open CH347T");
        check_usb(libusb_claim_interface(device_, Interface), "Claim CH347T SPI interface");
        claimed_ = true;

        // Hardware configuration payload: two-line master, 8-bit, mode 0,
        // software-controlled active-low CS, MSB first.
        std::array<uint8_t, 29> config{Configure, 26};
        config[5] = 0x04;
        config[6] = 0x01;
        config[14] = 0x02;
        config[15] = static_cast<uint8_t>((setting - Speeds.begin() + 1) * 8);
        config[19] = 0x07;
        write(config.data(), config.size());
        const int size = read(config.data(), config.size());
        if (size < 4 || config[0] != Configure || config[3] != 0)
            throw std::runtime_error("CH347T rejected SPI configuration");
        select(false);
    } catch (...) {
        close();
        throw;
    }
}

Ch347SPI::~Ch347SPI() { close(); }

void Ch347SPI::close() noexcept
{
    if (claimed_)
        libusb_release_interface(device_, Interface);
    if (device_)
        libusb_close(device_);
    libusb_exit(context_);
}

void Ch347SPI::write(uint8_t *data, int size)
{
    int transferred = 0;
    check_usb(libusb_bulk_transfer(device_, Out, data, size, &transferred, UsbTimeoutMs), "CH347T USB write");
    if (transferred != size)
        throw std::runtime_error("Short CH347T USB write");
}

int Ch347SPI::read(uint8_t *data, int capacity)
{
    int transferred = 0;
    check_usb(libusb_bulk_transfer(device_, In, data, capacity, &transferred, UsbTimeoutMs), "CH347T USB read");
    return transferred;
}

void Ch347SPI::select(bool active)
{
    // Assert CS0 with automatic release after the burst, or force it inactive.
    // The control byte is followed by the little-endian setup and inactive delays.
    std::array<uint8_t, 13> command{ChipSelect, 10, 0, uint8_t(active ? 0xa0 : 0xc0), SetupUs, 0, InactiveUs};
    write(command.data(), command.size());
    if (!active)
        released_at_ = monotonic_ns();
}

protocol::Frame Ch347SPI::transfer(const protocol::Frame &frame)
{
    wait_until(released_at_ + InactiveUs * 1000LL);
    constexpr uint8_t frame_size = std::tuple_size_v<protocol::Frame>;
    std::array<uint8_t, UsbHeaderSize + frame_size> packet{Exchange, frame_size, 0};
    std::copy(frame.begin(), frame.end(), packet.begin() + UsbHeaderSize);

    try {
        select(true);
        wait_until(monotonic_ns() + SetupUs * 1000LL);
        write(packet.data(), packet.size());
        const int received = read(packet.data(), packet.size());
        if (received != static_cast<int>(packet.size()) || packet[0] != Exchange ||
            packet[1] != frame_size || packet[2] != 0)
            throw std::runtime_error("Invalid CH347T SPI response");
        released_at_ = monotonic_ns();
    } catch (...) {
        // The payload might already have reached the MCU. Release CS without
        // replaying the frame and let the application attempt its final disable.
        reply_synchronized_ = false;
        try {
            select(false);
        } catch (...) {
        }
        throw;
    }
    // A timed-out USB reply can arrive late. The next transfer still sends
    // the shutdown command, but its received packet may belong to the earlier one.
    if (!reply_synchronized_)
        throw std::runtime_error("CH347T reply synchronization lost after USB exchange failure");
    protocol::Frame reply{};
    std::copy(packet.begin() + UsbHeaderSize, packet.end(), reply.begin());
    return reply;
}
