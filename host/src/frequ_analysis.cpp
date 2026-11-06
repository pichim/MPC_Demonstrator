#define SPI_SPEED_HZ 30000000
#define PERIOD_US 1000
#define MODE 0                 
#define CURRENT_LIMIT_A 1.7
#define VOLTAGE_LIMIT_V 24.0

// Chirp settings
#define CHIRP_F0_HZ 1.0  
#define CHIRP_F1_HZ 500.0
#define CHIRP_DURATION_S 30.0
#define CHIRP_AMPLITUDE_V 0.05 //0.05 max if mode = current ref
#define CHIRP_OFFSET_V 0.0

#define Trun (CHIRP_DURATION_S + PERIOD_US * 1e-6)
#define PRINT_EVERY 1

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <ctime>
#include <exception>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <system_error>
#include <type_traits>
#include <vector>

#include "spi_nss.h"
#include "chirp.hpp"

#define CHIRP_ABS_OFFSET_V (CHIRP_OFFSET_V < 0.0 ? -(CHIRP_OFFSET_V) : (CHIRP_OFFSET_V))

#if MODE == 0
static_assert(CHIRP_AMPLITUDE_V + CHIRP_ABS_OFFSET_V <= CURRENT_LIMIT_A,
              "Chirp amplitude plus offset exceeds current command limit in current mode");
#else
static_assert(CHIRP_AMPLITUDE_V + CHIRP_ABS_OFFSET_V <= VOLTAGE_LIMIT_V,
              "Chirp amplitude plus offset exceeds voltage command limit");
#endif

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }

protocol::Measurements read_measurements(NssSPI &spi)
{
    return protocol::decode(spi.transfer(protocol::frame(protocol::Read)));
}

protocol::Measurements send_command(NssSPI &spi, double setpoint, bool enable, int mode = MODE)
{
    
    return protocol::decode(spi.transfer(
        protocol::frame(protocol::Command,
                        {static_cast<float>(setpoint),
                         enable ? 1.0f : 0.0f,
                         static_cast<float>(mode),
                         0.0f})));
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

    std::vector<double> dt, spi, time_s, voltage_V, current_A,
        motor_position_rad, motor_velocity_rad_s, sent_setpoint,
        sent_enable, sent_mode, chirp_freq_Hz;
    size_t count = 0;
    uint64_t skipped_releases = 0;

    Log()
    {
        const double samples = std::floor(Trun * 1e6 / PERIOD_US + 0.5);
        if (!std::isfinite(samples) || samples < 1.0 ||
            samples >= static_cast<double>(dt.max_size())) {
            throw std::invalid_argument("Invalid run sample count");
        }

        dt.resize(static_cast<size_t>(samples));
        for (auto *values : {&spi, &time_s, &voltage_V, &current_A,
                             &motor_position_rad, &motor_velocity_rad_s,
                             &sent_setpoint, &sent_enable, &sent_mode,
                             &chirp_freq_Hz}) {
            values->resize(dt.size());
        }
    }
};

static_assert(PERIOD_US > 0 && PRINT_EVERY >= 0 &&
                  std::is_integral_v<decltype(PERIOD_US)> &&
                  std::is_integral_v<decltype(PRINT_EVERY)>,
              "Invalid period/reporting settings");
static_assert(MODE == 0 || MODE == 1, "Invalid command mode");

void run(NssSPI &spi, Log &log)
{
    const auto run_start = monotonic_ns();
    constexpr long long period_ns = PERIOD_US * 1000LL;
    auto next_release = run_start;
    long long previous = 0;
    bool enabled = false;

    Chirp chirp(CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_DURATION_S,
                static_cast<double>(PERIOD_US) * 1e-6);

    while (!stopped && log.count < log.spi.size()) {
        const auto start = monotonic_ns();
        const auto measurements = read_measurements(spi);
        const auto read_end = monotonic_ns();
        if (stopped)
            break;

        double chirp_voltage = 0.0;
        double chirp_frequency = 0.0;
        if (enabled) {
            if (chirp.update()) {
                chirp_voltage = CHIRP_AMPLITUDE_V * chirp.getExc() + CHIRP_OFFSET_V;
                chirp_frequency = chirp.getFreq();
            }
        }

        double setpoint = enabled ? chirp_voltage : 0.0;
        const int mode = MODE;
        const double limit = mode == 0 ? CURRENT_LIMIT_A : VOLTAGE_LIMIT_V;
        setpoint = std::clamp(setpoint, -limit, limit);

        const auto command_start = monotonic_ns();
        send_command(spi, setpoint, enabled, mode);
        const auto now = monotonic_ns();

        log.dt[log.count] = previous ? (now - previous) / 1e6 : 0.0;
        log.spi[log.count] = (read_end - start + now - command_start) / 1e6;
        log.time_s[log.count] = (read_end - run_start) / 1e9;
        log.voltage_V[log.count] = measurements[0];
        log.current_A[log.count] = measurements[1];
        log.motor_position_rad[log.count] = measurements[2];
        log.motor_velocity_rad_s[log.count] = measurements[3];
        log.sent_setpoint[log.count] = setpoint;
        log.sent_enable[log.count] = enabled;
        log.sent_mode[log.count] = mode;
        log.chirp_freq_Hz[log.count] = chirp_frequency;

        ++log.count;
        previous = now;
        enabled = true; 

        if (stopped || log.count == log.spi.size())
            break;

        next_release += period_ns;
        
        const auto finished = monotonic_ns();
        if (finished >= next_release) {
            const auto missed = (finished - next_release) / period_ns + 1;
            log.skipped_releases += missed;
            next_release += missed * period_ns;
        }
        sleep_until(next_release);
        
        const auto missed = std::max(0LL, (monotonic_ns() - next_release) / period_ns);
        log.skipped_releases += missed;
        next_release += missed * period_ns;
    }
}

void report(const Log &log, double cpu_s, double wall_s)
{
    std::cerr << "skipped_releases=" << log.skipped_releases << '\n';
    std::cout << "cycle,dt_n,dt_min_ms,dt_mean_ms,dt_max_ms,"
                 "spi_n,spi_min_ms,spi_mean_ms,spi_max_ms,cpu_s,wall_s,"
                 "time_s,voltage_V,current_A,motor_position_rad,"
                 "motor_velocity_rad_s,sent_setpoint,sent_enable,sent_mode,"
                 "chirp_freq_Hz\n"
              << std::fixed << std::setprecision(4);

    const size_t window = PRINT_EVERY ? PRINT_EVERY : std::max(log.count, size_t{1});
    for (size_t first = 0; first < log.count; first += window) {
        const auto last = std::min(first + window, log.count);
        std::cout << std::setprecision(4) << last;

        const auto stats = [&](const std::vector<double> &data, size_t begin) {
            std::cout << ',' << last - begin;
            if (begin < last) {
                const auto a = data.begin() + begin;
                const auto b = data.begin() + last;
                std::cout << ',' << *std::min_element(a, b)
                          << ',' << std::accumulate(a, b, 0.0) / (last - begin)
                          << ',' << *std::max_element(a, b);
            } else {
                std::cout << ",0.0000,0.0000,0.0000";
            }
        };

        stats(log.dt, std::max(first, size_t{1}));
        stats(log.spi, first);
        const auto i = last - 1;
        std::cout << ',' << cpu_s << ',' << wall_s
                  << std::setprecision(6) << ',' << log.time_s[i]
                  << ',' << log.voltage_V[i]
                  << ',' << log.current_A[i]
                  << ',' << log.motor_position_rad[i]
                  << ',' << log.motor_velocity_rad_s[i]
                  << ',' << log.sent_setpoint[i]
                  << ',' << static_cast<int>(log.sent_enable[i])
                  << ',' << static_cast<int>(log.sent_mode[i])
                  << ',' << log.chirp_freq_Hz[i] << '\n';
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
        if (sigaction(SIGINT, &action, nullptr) ||
            sigaction(SIGTERM, &action, nullptr)) {
            throw std::system_error(errno, std::generic_category(), "sigaction");
        }

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
                send_command(spi, 0.0, false);
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
                std::cerr << "Unknown chirp-loop error\n";
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

