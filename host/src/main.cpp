// Single-threaded SPI baseline, matching python/main.py.
// Keep aligned with the variables in python/main.py.
#define SPI_SPEED_HZ 30000000
#define PERIOD_US 1000
#define CURRENT_A 0.08f
#define Trun 20.0     // Nominal seconds; converted to a rounded integer cycle count.
#define PRINT_EVERY 1 // Samples per report after stopping; 0 reports all at once.

#include <algorithm>
#include <cmath>
#include <csignal>
#include <ctime>
#include <exception>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <type_traits>
#include <vector>

#include "spi_nss.h"

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }

protocol::Measurements read_measurements(NssSPI &spi)
{
    return protocol::decode(spi.transfer(protocol::frame(protocol::Read)));
}

protocol::Measurements send_command(NssSPI &spi, float current, bool enable)
{
    // Reply is prepared before receipt of this command; not an application ACK.
    return protocol::decode(spi.transfer(protocol::frame(protocol::Command, {current, enable ? 1.0f : 0.0f, 0.0f})));
}

void sleep_until(long long deadline)
{
    const timespec target{deadline / 1'000'000'000, deadline % 1'000'000'000};
    int error;
    do {
        if (stopped)
            return;
        error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr);
    } while (error == EINTR && !stopped);
    if (error != 0 && error != EINTR)
        throw std::system_error(error, std::generic_category(), "clock_nanosleep");
}

struct Log {
    // Fixed storage; add sensor/command fields here when data logging is needed.
    std::vector<double> dt, spi;
    size_t count = 0;
    Log()
    {
        const double count = std::floor(Trun * 1e6 / PERIOD_US + 0.5);
        if (!std::isfinite(count) || count < 1 || count >= dt.max_size())
            throw std::invalid_argument("Invalid run sample count");
        dt.resize(static_cast<size_t>(count));
        spi.resize(dt.size());
    }
};
static_assert(PERIOD_US > 0 && PRINT_EVERY >= 0 && std::is_integral_v<decltype(PERIOD_US)> &&
                  std::is_integral_v<decltype(PRINT_EVERY)>,
              "Invalid period/reporting settings");
static_assert(CURRENT_A >= -std::numeric_limits<float>::max() && CURRENT_A <= std::numeric_limits<float>::max(),
              "Invalid current");

void run(NssSPI &spi, Log &log)
{
    long long previous = 0;
    bool enabled = false;
    while (!stopped && log.count < log.spi.size()) {
        const auto start = monotonic_ns();
        const auto measurements = read_measurements(spi);
        (void)measurements; // Available to the future controller.
        const auto read_end = monotonic_ns();
        if (stopped)
            break;
        // Future controller computation belongs here, after receiving sensors.
        const float current = enabled ? CURRENT_A : 0.0f;
        const auto command_start = monotonic_ns();
        send_command(spi, current, enabled);
        const auto now = monotonic_ns();
        log.dt[log.count] = previous ? (now - previous) / 1e6 : 0.0;
        log.spi[log.count] = (read_end - start + now - command_start) / 1e6;
        ++log.count;
        previous = now;
        enabled = true; // First exchange always sends zero current, disabled.
        const auto deadline = start + PERIOD_US * 1000LL;
        if (!stopped && monotonic_ns() < deadline)
            sleep_until(deadline);
    }
}

void report(const Log &log, double cpu_s, double wall_s)
{
    // Formatting and output happen only after the final disable and SPI close.
    std::cout << "cycle,dt_n,dt_min_ms,dt_mean_ms,dt_max_ms,spi_n,spi_min_ms,spi_mean_ms,spi_max_ms,cpu_s,wall_s\n"
              << std::fixed << std::setprecision(4);
    const size_t window = PRINT_EVERY ? PRINT_EVERY : std::max(log.count, size_t{1});
    for (size_t first = 0; first < log.count; first += window) {
        const auto last = std::min(first + window, log.count);
        std::cout << last;
        const auto stats = [&](const std::vector<double> &data, size_t begin) {
            std::cout << ',' << last - begin;
            if (begin < last) {
                const auto a = data.begin() + begin, b = data.begin() + last;
                std::cout << ',' << *std::min_element(a, b) << ',' << std::accumulate(a, b, 0.0) / (last - begin) << ','
                          << *std::max_element(a, b);
            } else {
                std::cout << ",0.0000,0.0000,0.0000";
            }
        };
        stats(log.dt, std::max(first, size_t{1}));
        stats(log.spi, first);
        // Run totals repeated as metadata, not per-window CPU measurements.
        std::cout << ',' << cpu_s << ',' << wall_s << '\n';
    }
}
} // namespace

int main(int argc, char **)
{
    if (argc != 1) {
        std::cerr << "No run options: edit host/src/main.cpp and rebuild.\n";
        return 1;
    }
    try {
        struct sigaction action{};
        action.sa_handler = stop;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGINT, &action, nullptr) || sigaction(SIGTERM, &action, nullptr))
            throw std::system_error(errno, std::generic_category(), "sigaction");
        Log log;
        int status = 0;
        double cpu_s, wall_s;
        std::exception_ptr failure;
        {
            NssSPI spi(SPI_SPEED_HZ);
            const auto wall_start = monotonic_ns();
            const auto cpu_start = std::clock();
            try {
                run(spi, log);
            } catch (...) {
                failure = std::current_exception();
                status = 1;
            }
            cpu_s = double(std::clock() - cpu_start) / CLOCKS_PER_SEC;
            wall_s = (monotonic_ns() - wall_start) / 1e9;
            try {
                send_command(spi, 0.0f, false);
            } catch (const std::exception &error) {
                std::cerr << "Final disable unconfirmed: " << error.what() << '\n';
                status = 1;
            }
        } // Close SPI before reporting.
        if (failure) {
            try {
                std::rethrow_exception(failure);
            } catch (const std::exception &error) {
                std::cerr << error.what() << '\n';
            } catch (...) {
                std::cerr << "Unknown control-loop error\n";
            }
        }
        report(log, cpu_s, wall_s);
        std::cout.flush();
        return std::cout ? status : 1;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
