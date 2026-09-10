#include "serial_port.h"
#include "host_config.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

namespace
{
constexpr std::uint32_t BAUD = 115200;
constexpr double COMMUNICATION_FREQUENCY_HZ = 500.0;
constexpr double EXPECTED_PERIOD_MS = 1000.0 / COMMUNICATION_FREQUENCY_HZ;

struct Command
{
    float current_cmd = 0.0f;
    bool enable = false;
};

struct Measurements
{
    float motor_angle = 0.0f;
    float pendulum_angle = 0.0f;
    float current = 0.0f;
};

bool setHighPriority()
{
#ifdef _WIN32
    return SetThreadPriority(
        GetCurrentThread(),
        THREAD_PRIORITY_HIGHEST) != 0;
#else
    sched_param param{};
    param.sched_priority = 50;

    return pthread_setschedparam(
        pthread_self(),
        SCHED_FIFO,
        &param) == 0;
#endif
}

void encodeCommand(const Command& command, std::array<std::uint8_t, 5>& data)
{
    static_assert(sizeof(float) == 4, "Protocol requires 32-bit float.");

    std::memcpy(data.data(), &command.current_cmd, sizeof(float));
    data[4] = command.enable ? 1 : 0;
}

void decodeMeasurements(
    const std::array<std::uint8_t, 12>& data,
    Measurements& measurements)
{
    static_assert(sizeof(float) == 4, "Protocol requires 32-bit float.");

    std::memcpy(&measurements.motor_angle, data.data() + 0, sizeof(float));
    std::memcpy(&measurements.pendulum_angle, data.data() + 4, sizeof(float));
    std::memcpy(&measurements.current, data.data() + 8, sizeof(float));
}

Command controller(const Measurements& measurements)
{
    (void)measurements;

    // TODO: implement the host-side controller.
    // Safe default: motor disabled, zero current command.
    return {};
}

void communicationLoop(const std::string& port_name)
{
    SerialPort serial;

    if (!serial.open(port_name, BAUD)) {
        std::cerr << "Could not open serial port: " << port_name << '\n';
        return;
    }

    if (!serial.clearBuffers()) {
        std::cerr << "Could not clear serial buffers.\n";
        return;
    }

    if (!setHighPriority())
        std::cerr << "Warning: could not raise communication thread priority.\n";

    std::array<std::uint8_t, 5> tx{};
    std::array<std::uint8_t, 12> rx{};

    std::cout << "Expected communication rate: "
              << COMMUNICATION_FREQUENCY_HZ << " Hz ("
              << EXPECTED_PERIOD_MS << " ms)\n";

    // Prime communication with a safe command.
    Command command{};
    encodeCommand(command, tx);

    if (!serial.writeExact(tx.data(), tx.size())) {
        std::cerr << "Initial UART write failed.\n";
        return;
    }

#ifdef UART_HOST_PRINT_SAMPLES
    static_assert(UART_HOST_PRINT_EVERY_N > 0, "Print interval must be positive.");
    using clock = std::chrono::steady_clock;
    clock::time_point previous_time{};
    bool first_sample = true;
    unsigned sample_count = 0;
    unsigned dt_count = 0;
    double dt_sum_ms = 0.0;
    double dt_min_ms = 0.0;
    double dt_max_ms = 0.0;
#endif

    while (true) {
        // The MCU checks for new data on its own faster tick and replies to commands.
        if (!serial.readExact(rx.data(), rx.size())) {
            std::cerr << "UART read failed or timed out.\n";
            return;
        }

#ifdef UART_HOST_PRINT_SAMPLES
        const auto now = clock::now();
        if (!first_sample) {
            const double dt_ms =
                std::chrono::duration<double, std::milli>(now - previous_time).count();
            if (dt_count == 0 || dt_ms < dt_min_ms)
                dt_min_ms = dt_ms;
            if (dt_count == 0 || dt_ms > dt_max_ms)
                dt_max_ms = dt_ms;
            dt_sum_ms += dt_ms;
            ++dt_count;
        }
        previous_time = now;
        first_sample = false;
#endif

        Measurements measurements;
        decodeMeasurements(rx, measurements);

#ifdef UART_HOST_PRINT_SAMPLES
        if (++sample_count == UART_HOST_PRINT_EVERY_N) {
            std::cout
                << "motor_angle=" << measurements.motor_angle
                << "  pendulum_angle=" << measurements.pendulum_angle
                << "  current=" << measurements.current;
            // The first received sample has no packet-to-packet interval.
            if (dt_count > 0) {
                std::cout
                    << "  dt_avg=" << dt_sum_ms / dt_count
                    << " ms  dt_min=" << dt_min_ms
                    << " ms  dt_max=" << dt_max_ms << " ms";
            }
            std::cout << '\n';
            sample_count = 0;
            dt_count = 0;
            dt_sum_ms = 0.0;
        }
#endif

        command = controller(measurements);
        encodeCommand(command, tx);

        // Send the next command; the MCU processes it when available.
        if (!serial.writeExact(tx.data(), tx.size())) {
            std::cerr << "UART write failed.\n";
            return;
        }
    }
}
}

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: uart_host <serial-port>\n";
        return 1;
    }

    std::thread communication_thread(communicationLoop, std::string(argv[1]));
    communication_thread.join();

    return 0;
}
