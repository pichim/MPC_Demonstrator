#include "serial_port.h"
#include "host_config.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include <pthread.h>
#include <sched.h>

namespace
{
constexpr std::uint32_t BAUD = 230400;
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

#ifdef UART_HOST_PRINT_SAMPLES
struct SampleReport
{
    Measurements measurements;
    Command command;
    unsigned dt_count = 0;
    double dt_sum_ms = 0.0;
    double dt_min_ms = 0.0;
    double dt_max_ms = 0.0;
};

class SampleReporter
{
public:
    // Exactly one communication thread may publish. Stop only after it has joined.
    SampleReporter()
        : reporting_thread_(&SampleReporter::reportingLoop, this)
    {
    }

    ~SampleReporter()
    {
        stop();
    }

    SampleReporter(const SampleReporter&) = delete;
    SampleReporter& operator=(const SampleReporter&) = delete;

    void publish(const SampleReport& report)
    {
        const std::size_t write_index = write_index_.load(std::memory_order_relaxed);
        const std::size_t next_write_index = (write_index + 1) % REPORT_QUEUE_CAPACITY;

        if (next_write_index == read_index_.load(std::memory_order_acquire)) {
            dropped_reports_.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        reports_[write_index] = report;
        write_index_.store(next_write_index, std::memory_order_release);
    }

    void stop()
    {
        if (!reporting_thread_.joinable())
            return;

        stop_requested_.store(true, std::memory_order_release);
        reporting_thread_.join();
    }

private:
    static constexpr std::size_t REPORT_QUEUE_CAPACITY = 64;
    static_assert(std::atomic<std::size_t>::is_always_lock_free,
                  "Sample queue indices must be lock-free.");
    static_assert(std::atomic<unsigned>::is_always_lock_free,
                  "Sample drop counter must be lock-free.");

    bool tryPop(SampleReport& report)
    {
        const std::size_t read_index = read_index_.load(std::memory_order_relaxed);
        if (read_index == write_index_.load(std::memory_order_acquire))
            return false;

        report = reports_[read_index];
        read_index_.store(
            (read_index + 1) % REPORT_QUEUE_CAPACITY,
            std::memory_order_release);
        return true;
    }

    void reportingLoop()
    {
        while (true) {
            // Observe producer completion before checking for an empty queue.
            // Otherwise a final publish between tryPop() and the stop load
            // could be left behind during shutdown.
            const bool stopping = stop_requested_.load(std::memory_order_acquire);
            SampleReport report;
            if (tryPop(report)) {
                std::cout
                    << "motor_angle=" << report.measurements.motor_angle
                    << "  pendulum_angle=" << report.measurements.pendulum_angle
                    << "  current=" << report.measurements.current
                    << "  current_cmd=" << report.command.current_cmd
                    << "  enable=" << report.command.enable;
                if (report.dt_count > 0) {
                    std::cout
                        << "  dt_avg=" << report.dt_sum_ms / report.dt_count
                        << " ms  dt_min=" << report.dt_min_ms
                        << " ms  dt_max=" << report.dt_max_ms << " ms";
                }
                std::cout << std::endl;
                continue;
            }

            if (stopping)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        const unsigned dropped_reports =
            dropped_reports_.load(std::memory_order_relaxed);
        if (dropped_reports > 0)
            std::cerr << "Warning: dropped " << dropped_reports
                      << " sample reports because output was too slow.\n";
    }

    std::array<SampleReport, REPORT_QUEUE_CAPACITY> reports_{};
    std::atomic<std::size_t> write_index_{0};
    std::atomic<std::size_t> read_index_{0};
    std::atomic<unsigned> dropped_reports_{0};
    std::atomic<bool> stop_requested_{false};
    std::thread reporting_thread_;
};
#else
class SampleReporter
{
public:
    void stop() {}
};
#endif

bool setHighPriority()
{
    sched_param param{};
    param.sched_priority = 50;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
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

    // Constant-current test; startup still uses the disabled Command defaults.
    return {0.05f, true};
}

void communicationLoop(
    const std::string& port_name,
    SampleReporter& sample_reporter)
{
#ifndef UART_HOST_PRINT_SAMPLES
    (void)sample_reporter;
#endif

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
              << EXPECTED_PERIOD_MS << " ms)" << std::endl;

    // Prime communication with a safe command.
    Command command{};
    encodeCommand(command, tx);

    using clock = std::chrono::steady_clock;
    const auto communication_period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / COMMUNICATION_FREQUENCY_HZ));
    auto last_send_time = clock::now();

    if (!serial.writeExact(tx.data(), tx.size())) {
        std::cerr << "Initial UART write failed.\n";
        return;
    }

#ifdef UART_HOST_PRINT_SAMPLES
    static_assert(UART_HOST_PRINT_EVERY_N > 0, "Print interval must be positive.");
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
            sample_reporter.publish({
                measurements,
                command,
                dt_count,
                dt_sum_ms,
                dt_min_ms,
                dt_max_ms});
            sample_count = 0;
            dt_count = 0;
            dt_sum_ms = 0.0;
        }
#endif

        command = controller(measurements);
        encodeCommand(command, tx);

        // Send the next command; the MCU processes it when available.
        // Space command starts by at least 2 ms. Late cycles do not trigger catch-up bursts.
        std::this_thread::sleep_until(last_send_time + communication_period);
        last_send_time = clock::now();
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

    SampleReporter sample_reporter;
    std::thread communication_thread(
        communicationLoop,
        std::string(argv[1]),
        std::ref(sample_reporter));
    communication_thread.join();
    sample_reporter.stop();

    return 0;
}
