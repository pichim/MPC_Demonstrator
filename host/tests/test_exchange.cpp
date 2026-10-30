// Run via test_exchange.py: it fixes only application settings in a temporary
// source copy and supplies TEST_EXPECTED_SETPOINT for each command-limit case.
// Application logic and the real transport run with fake SPI/clock syscalls.
#define main application_main
#include "../src/main.cpp"
#undef main

#include <cassert>
#include <cstdarg>
#include <sstream>

namespace {
std::vector<protocol::Measurements> commands;
long long fake_now = 1'000'000'000, first_work = 100'000, wake_delay = 0;
unsigned fault_at = 0, stop_at = 0, sleeps = 0;
enum class Fault { None, CRC, Header, Nonfinite, Short, IO } fault = Fault::None;
bool closed = false;

void reset()
{
    commands.clear();
    fake_now = 1'000'000'000;
    first_work = 100'000;
    wake_delay = 0;
    fault_at = stop_at = sleeps = 0;
    fault = Fault::None;
    closed = false;
    stopped = 0;
}

void near(double actual, double expected) { assert(std::abs(actual - expected) < 1e-6); }
} // namespace

extern "C" int __wrap_open(const char *path, int, ...)
{
    assert(std::strcmp(path, "/dev/spidev0.0") == 0);
    return 99;
}

extern "C" int __wrap_close(int fd)
{
    assert(fd == 99);
    closed = true;
    return 0;
}

extern "C" int __wrap_clock_gettime(clockid_t, timespec *time)
{
    time->tv_sec = fake_now / 1'000'000'000;
    time->tv_nsec = fake_now % 1'000'000'000;
    fake_now += 100; // Let the transport's minimum-inactive busy wait progress.
    return 0;
}

extern "C" int __wrap_clock_nanosleep(clockid_t, int flags, const timespec *target, timespec *)
{
    assert(flags == TIMER_ABSTIME);
    fake_now = target->tv_sec * 1'000'000'000LL + target->tv_nsec + (sleeps++ == 0 ? wake_delay : 0);
    return 0;
}

extern "C" int __wrap_ioctl(int fd, unsigned long request, ...)
{
    assert(fd == 99);
    if (request != SPI_IOC_MESSAGE(2)) return 0;
    va_list args;
    va_start(args, request);
    auto *segments = va_arg(args, spi_ioc_transfer *);
    va_end(args);
    assert(segments[0].len == 0 && segments[0].delay_usecs == 30 && !segments[0].cs_change);
    assert(segments[1].len == 18 && segments[1].bits_per_word == 8 && !segments[1].cs_change);
    auto tx = *reinterpret_cast<const protocol::Frame *>(segments[1].tx_buf);
    assert(tx[0] == protocol::Command && tx.back() == protocol::crc8(tx));
    tx[0] = protocol::Reply;
    tx.back() = protocol::crc8(tx);
    commands.push_back(protocol::decode(tx));
    const float n = commands.size();
    auto reply = protocol::frame(protocol::Reply, {n, -n, n + 0.5f, -n - 0.5f});
    fake_now += commands.size() == 1 ? first_work : 100'000;
    if (commands.size() == stop_at) stop(SIGINT);
    if (commands.size() == fault_at) {
        switch (fault) {
        case Fault::CRC: reply.back() ^= 1; break;
        case Fault::Header: reply = protocol::frame(0x7F); break;
        case Fault::Nonfinite:
            reply = protocol::frame(protocol::Reply, {NAN, 0, 0, 0}); break;
        case Fault::Short: return 17;
        case Fault::IO: errno = EIO; return -1;
        case Fault::None: break;
        }
    }
    std::memcpy(reinterpret_cast<void *>(segments[1].rx_buf), reply.data(), reply.size());
    return 18;
}

int main()
{
    for (int scenario = 0; scenario < 4; ++scenario) {
        reset();
        if (scenario == 1) first_work = 2'000'000;
        if (scenario == 2) wake_delay = 2'000'000;
        Log log;
        log.spi.resize(scenario == 3 ? 1 : 3);
        NssSPI spi(SPI_SPEED_HZ);
        run(spi, log);
        assert(commands.size() == log.spi.size() && log.count == commands.size());
        assert(log.skipped_releases == (scenario == 1 || scenario == 2 ? 2 : 0));
        assert(sleeps == commands.size() - 1);
        for (size_t i = 0; i < commands.size(); ++i) {
            near(commands[i][0], i ? TEST_EXPECTED_SETPOINT : 0);
            near(commands[i][1], i ? 1 : 0);
            near(commands[i][2], MODE);
            near(commands[i][3], 0);
            near(log.sent_setpoint[i], commands[i][0]);
            near(log.sent_enable[i], commands[i][1]);
            near(log.sent_mode[i], commands[i][2]);
            near(log.voltage_V[i], i + 1);
            near(log.current_A[i], -double(i + 1));
            near(log.motor_position_rad[i], i + 1.5);
            near(log.motor_velocity_rad_s[i], -double(i) - 1.5);
            assert(log.spi[i] >= 0.1 && log.spi[i] < (scenario == 1 && i == 0 ? 2.001 : 0.101));
            if (i) {
                near(log.dt[i], (log.time_s[i] - log.time_s[i - 1]) * 1000);
                assert(log.dt[i] > 0.9);
            } else {
                near(log.dt[i], 0);
                assert(log.time_s[i] > 0);
            }
        }
    }
    // Full normal completion, stop during an exchange, reply/transport errors,
    // and failure of the final disable. No real device or real sleeping is used.
    for (int scenario = 0; scenario < 9; ++scenario) {
        reset();
        if (scenario == 1 || scenario == 7) stop_at = 3;
        if (scenario >= 2 && scenario <= 6) {
            fault_at = 2;
            fault = static_cast<Fault>(scenario - 1);
        }
        if (scenario == 7) { fault_at = 4; fault = Fault::IO; }
        if (scenario == 8) { fault_at = 1; fault = Fault::CRC; }
        std::ostringstream output, errors;
        auto *old_out = std::cout.rdbuf(output.rdbuf());
        auto *old_err = std::cerr.rdbuf(errors.rdbuf());
        const int status = application_main(1, nullptr);
        std::cout.rdbuf(old_out);
        std::cerr.rdbuf(old_err);
        assert(status == (scenario >= 2 ? 1 : 0));
        assert(closed);
        const size_t cycles = static_cast<size_t>(std::floor(Trun * 1e6 / PERIOD_US + 0.5));
        assert(commands.size() == (scenario == 0 ? cycles + 1 : scenario == 1 || scenario == 7 ? 4 : scenario == 8 ? 2 : 3));
        near(commands.front()[0], 0);
        near(commands.front()[1], 0);
        near(commands.back()[0], 0);
        near(commands.back()[1], 0);
        const auto csv = output.str();
        const size_t rows = std::count(csv.begin(), csv.end(), '\n');
        assert(rows == (scenario == 0 ? cycles + 1 : scenario == 1 || scenario == 7 ? 4 : scenario == 8 ? 1 : 2));
    }
}
