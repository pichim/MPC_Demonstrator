#include "spi_nss.h"
#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
struct Options {
    unsigned count = 10000, period_us = 1000, speed = config::spi_speed;
    float current = 0.08f;
    bool enable = false, faults = false;
    std::string csv;
};
unsigned positive(const std::string& value, unsigned maximum) {
    if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("Expected a positive integer: " + value);
    const auto number = std::stoull(value);
    if (number == 0 || number > maximum) throw std::runtime_error("Argument out of range");
    return static_cast<unsigned>(number);
}
Options options(int argc, char** argv) {
    Options result;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cout << "mpc_spi [--count N] [--period-us N] [--speed Hz]\n"
                         "        [--enable] [--current A] [--csv path] [--faults]\n"
                         "Defaults: 10000 cycles, 1000 us, 30 MHz, disabled.\n"
                         "--enable sends 0.08 A unless --current is supplied.\n"
                         "--faults adds 10 rounds of disabled-command recovery tests.\n";
            std::exit(0);
        } else if (arg == "--enable") result.enable = true;
        else if (arg == "--faults") result.faults = true;
        else {
            if (++i == argc) throw std::runtime_error("Missing value for " + arg);
            const std::string value = argv[i];
            if (arg == "--count") result.count = positive(value, 1'000'000);
            else if (arg == "--period-us") result.period_us = positive(value, 1'000'000);
            else if (arg == "--speed") result.speed = positive(value, 30'000'000);
            else if (arg == "--csv") result.csv = value;
            else if (arg == "--current") {
                std::size_t used = 0;
                result.current = std::stof(value, &used);
                if (used != value.size() || !std::isfinite(result.current))
                    throw std::runtime_error("Current must be finite");
            } else throw std::runtime_error("Unknown option: " + arg);
        }
    }
    if (result.enable && result.faults)
        throw std::runtime_error("--faults requires disabled commands");
    return result;
}
void sleep_until(long long deadline) {
    const timespec time{deadline / 1'000'000'000, deadline % 1'000'000'000};
    int result;
    do {
        result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, nullptr);
    } while (result == EINTR && !stopped);
    if (result != 0 && result != EINTR)
        throw std::system_error(result, std::generic_category(), "clock_nanosleep");
}
long long cpu_ns() {
    timespec time{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time) < 0)
        throw std::system_error(errno, std::generic_category(), "CPU clock");
    return time.tv_sec * 1'000'000'000LL + time.tv_nsec;
}
struct Sample { long long start, read_done, done; };
void fault_tests(NssSPI& spi) {
    const auto disabled = protocol::frame(protocol::Command);
    for (unsigned round = 0; round < 10 && !stopped; ++round) {
        for (unsigned fault = 0; fault < 9 && !stopped; ++fault) {
            if (fault >= 7) spi.cancel_selection(fault == 8);
            else {
                std::array<uint8_t, 28> tx{}, rx{};
                std::copy(disabled.begin(), disabled.end(), tx.begin());
                constexpr unsigned lengths[] = {1, 7, 13, 15, 28, 14, 14};
                if (fault == 5) tx[13] ^= 1;
                if (fault == 6) {
                    const auto unknown = protocol::frame(0x7f);
                    std::copy(unknown.begin(), unknown.end(), tx.begin());
                }
                spi.transfer_raw(tx.data(), rx.data(), lengths[fault]);
            }
            for (unsigned recovery = 0; recovery < 100 && !stopped; ++recovery) {
                const auto start = monotonic_ns();
                protocol::decode(spi.transfer(protocol::frame(protocol::Read)));
                protocol::decode(spi.transfer(disabled));
                sleep_until(start + 1'000'000);
            }
        }
        if (!stopped) std::cout << "PASS recovery round " << round + 1 << "/10\n";
    }
}
void statistics(const char* name, std::vector<double> values) {
    if (values.empty()) return;
    std::sort(values.begin(), values.end());
    const auto p99 = static_cast<std::size_t>(std::ceil(0.99 * values.size())) - 1;
    std::cout << name << "_mean_us="
              << std::accumulate(values.begin(), values.end(), 0.0) / values.size()
              << " " << name << "_p99_us=" << values[p99]
              << " " << name << "_max_us=" << values.back() << '\n';
}
} // namespace

int main(int argc, char** argv) {
    try {
        const auto opt = options(argc, argv);
        // Allocate and open output before touching hardware or entering the loop.
        std::vector<Sample> samples(opt.count);
        std::ofstream csv;
        if (!opt.csv.empty()) {
            csv.open(opt.csv);
            if (!csv) throw std::runtime_error("Cannot open CSV output");
        }
        struct sigaction action{};
        action.sa_handler = stop;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGINT, &action, nullptr) || sigaction(SIGTERM, &action, nullptr))
            throw std::system_error(errno, std::generic_category(), "sigaction");
        unsigned completed = 0;
        long long elapsed = 0, cpu = 0;
        int status = 0;
        {
            NssSPI spi(opt.speed);
            const auto read = protocol::frame(protocol::Read);
            const auto disabled = protocol::frame(protocol::Command);
            std::cout << "NSS: " << opt.speed << " Hz SPI, " << opt.period_us
                      << " us period, " << opt.count << " cycles, setup=" << config::nss_setup_us
                      << " us, inactive=" << config::nss_inactive_us
                      << " us, enable=" << opt.enable
                      << std::endl;
            const auto begin = monotonic_ns(), cpu_begin = cpu_ns();
            try {
                for (; completed < opt.count && !stopped; ++completed) {
                    auto& sample = samples[completed];
                    sample.start = monotonic_ns();
                    const auto measurements = protocol::decode(spi.transfer(read));
                    sample.read_done = monotonic_ns();
                    // Future controller uses measurements here, before COMMAND.
                    (void)measurements;
                    const bool enabled = opt.enable && completed != 0 && !stopped;
                    const auto command = protocol::frame(protocol::Command,
                        {enabled ? opt.current : 0.0f, enabled ? 1.0f : 0.0f, 0.0f});
                    // This reply predates the command; it is not an application ACK.
                    protocol::decode(spi.transfer(command));
                    sample.done = monotonic_ns();
                    const auto deadline = sample.start + opt.period_us * 1000LL;
                    if (!stopped && monotonic_ns() < deadline) sleep_until(deadline);
                }
                elapsed = monotonic_ns() - begin;
                cpu = cpu_ns() - cpu_begin;
                if (opt.faults && !stopped) fault_tests(spi);
            } catch (const std::exception& error) {
                elapsed = monotonic_ns() - begin;
                cpu = cpu_ns() - cpu_begin;
                std::cerr << error.what() << '\n';
                status = 1;
            }
            try {
                protocol::decode(spi.transfer(disabled));
            } catch (const std::exception& error) {
                std::cerr << "Final disable unconfirmed: " << error.what() << '\n';
                status = 1;
            }
        } // Release SPI before reporting or writing samples.
        std::vector<double> intervals, work, reads;
        unsigned over_budget = 0;
        if (csv.is_open()) csv << "start_ns,read_done_ns,done_ns\n";
        for (unsigned i = 0; i < completed; ++i) {
            const auto& sample = samples[i];
            if (i) intervals.push_back((sample.done - samples[i - 1].done) / 1000.0);
            work.push_back((sample.done - sample.start) / 1000.0);
            reads.push_back((sample.read_done - sample.start) / 1000.0);
            over_budget += sample.done - sample.start > opt.period_us * 1000LL;
            if (csv.is_open()) csv << sample.start << ',' << sample.read_done << ',' << sample.done << '\n';
        }
        if (csv.is_open()) {
            csv.flush();
            if (!csv) throw std::runtime_error("Failed to write CSV output");
        }
        std::cout << std::fixed << std::setprecision(3)
                  << "completed=" << completed << " stopped=" << stopped
                  << " work_over_budget=" << over_budget
                  << " cpu_percent=" << (elapsed > 0 ? 100.0 * cpu / elapsed : 0.0) << '\n';
        statistics("dt", std::move(intervals));
        statistics("work", std::move(work));
        statistics("read", std::move(reads));
        return status;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
