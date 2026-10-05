// Single-threaded SPI baseline, matching python/main.py.
// Keep aligned with the variables in python/main.py.
#define SPI_SPEED_HZ 30000000
#define PERIOD_US 1000
#define SETPOINT 0.08f        // A in current mode, V in voltage mode.
#define MODE 1                // 0: current setpoint, 1: direct voltage.
#define CURRENT_LIMIT_A 1.0f  // Host current-command limit; keep aligned with Python host.
#define VOLTAGE_LIMIT_V 24.0f // MCU supply minus compensation; keep aligned with config.h.
#define Trun 5.0              // Nominal seconds; converted to a rounded integer cycle count.
#define PRINT_EVERY 1         // Samples per report after stopping; 0 reports all at once.

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
#include "mpc_solver.hpp"
#include "ref.hpp"
#include "csv_utils.hpp"
#include <Eigen/Dense>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }

protocol::Measurements read_measurements(NssSPI &spi)
{
    return protocol::decode(spi.transfer(protocol::frame(protocol::Read)));
}

protocol::Measurements send_command(NssSPI &spi, float setpoint, bool enable, int mode = MODE)
{
    // Reply is prepared before receipt of this command; not an application ACK.
    return protocol::decode(spi.transfer(
        protocol::frame(protocol::Command, {setpoint, enable ? 1.0f : 0.0f, static_cast<float>(mode), 0.0f})));
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
    // Fixed storage for timing, READ telemetry and the subsequently sent command.
    std::vector<double> dt, spi, time_s, voltage_V, current_A, motor_position_rad, motor_velocity_rad_s, sent_setpoint,
    sent_enable, sent_mode, u_mpc, xhat_current, xhat_speed, xhat_pose, xhat_dist, mpc_status, mpc_iters, mpc_fault, mpc_us;
    size_t count = 0;
    uint64_t skipped_releases = 0;
    Log()
    {
        const double count = std::floor(Trun * 1e6 / PERIOD_US + 0.5);
        if (!std::isfinite(count) || count < 1 || count >= dt.max_size())
            throw std::invalid_argument("Invalid run sample count");
        dt.resize(static_cast<size_t>(count));
        for (auto *values : {&spi,
                     &time_s,
                     &voltage_V,
                     &current_A,
                     &motor_position_rad,
                     &motor_velocity_rad_s,
                     &sent_setpoint,
                     &sent_enable,
                     &sent_mode,
                     &u_mpc,
                     &xhat_current,
                     &xhat_speed,
                     &xhat_pose,
                     &xhat_dist,
                     &mpc_status,
                     &mpc_iters,
                     &mpc_fault,
                     &mpc_us})
              values->resize(dt.size());
    }
};
static_assert(PERIOD_US > 0 && PRINT_EVERY >= 0 && std::is_integral_v<decltype(PERIOD_US)> &&
                  std::is_integral_v<decltype(PRINT_EVERY)>,
              "Invalid period/reporting settings");
static_assert(SETPOINT >= -std::numeric_limits<float>::max() && SETPOINT <= std::numeric_limits<float>::max(),
              "Invalid setpoint");
static_assert((MODE == 0 || MODE == 1) && CURRENT_LIMIT_A > 0.0f &&
                  CURRENT_LIMIT_A <= std::numeric_limits<float>::max() && VOLTAGE_LIMIT_V > 0.0f &&
                  VOLTAGE_LIMIT_V <= std::numeric_limits<float>::max(),
              "Invalid mode/command limits");

void run(NssSPI &spi, Log &log)
{
    const auto run_start = monotonic_ns();
    constexpr long long period_ns = PERIOD_US * 1000LL;
    auto next_release = run_start;
    long long previous = 0;
    bool enabled = false;

    //import the matrices
    std::string matDir = "/home/pi/MPC_Demonstrator/host/mpc_matrices";

    Eigen::MatrixXd H      = loadMatrixCSV(matDir + "/H.csv").cast<double>();
    Eigen::MatrixXd Aineq  = loadMatrixCSV(matDir + "/Aineq.csv").cast<double>();
    Eigen::MatrixXd Fx     = loadMatrixCSV(matDir + "/Fx.csv").cast<double>();
    Eigen::VectorXd Fu0    = loadVectorCSV(matDir + "/Fu0.csv").cast<double>();
    Eigen::VectorXd Fd     = loadVectorCSV(matDir + "/Fd.csv").cast<double>();
    Eigen::MatrixXd Fr     = loadMatrixCSV(matDir + "/Fr.csv").cast<double>();
    Eigen::MatrixXd Ex     = loadMatrixCSV(matDir + "/Ex.csv").cast<double>();
    Eigen::VectorXd Eu0    = loadVectorCSV(matDir + "/Eu0.csv").cast<double>();
    Eigen::VectorXd Ed     = loadVectorCSV(matDir + "/Ed.csv").cast<double>();
    Eigen::VectorXd bconst = loadVectorCSV(matDir + "/bconst.csv").cast<double>();
    Eigen::MatrixXd Aaug   = loadMatrixCSV(matDir + "/Aaug.csv").cast<double>();
    Eigen::VectorXd Baug   = loadVectorCSV(matDir + "/Baug.csv").cast<double>();
    Eigen::MatrixXd Caug   = loadMatrixCSV(matDir + "/Caug.csv").cast<double>();
    Eigen::MatrixXd Lgain  = loadMatrixCSV(matDir + "/Lgain.csv").cast<double>();
    CsvTable meta = loadCsvTable(matDir + "/meta.csv");
    double umx = meta.rows[0][colIndex(meta, "umx")];

    MpcController ctrl(H, Aineq, Fx, Fu0, Fd, Fr, Ex, Eu0, Ed, bconst, umx, Aaug, Baug, Caug, Lgain);
    double theta0 = 0.0;
    bool have_theta0 = false;

    while (!stopped && log.count < log.spi.size()) {
        const auto start = monotonic_ns();
        const auto measurements = read_measurements(spi);
        const auto read_end = monotonic_ns();
        if (stopped)
            break;

        //MPC control
        if (!have_theta0) { theta0 = measurements[2]; have_theta0 = true; }
        const double position = measurements[2] - theta0;
        Eigen::Vector2d xmeas(measurements[1], position);

        const double t = (read_end - run_start) / 1e9;
        double theta_ref = ref::reference(t);
        //double theta_ref = 2*3.141592;

        const auto mpc_t0 = monotonic_ns();
        MpcController::Result r = ctrl.step(xmeas, theta_ref);
        const double mpc_us = (monotonic_ns() - mpc_t0) / 1e3;

        const int mode = MODE;
        //float setpoint = enabled ? SETPOINT : 0.0f;
        float setpoint = enabled ? static_cast<float>(r.u_apply) : 0.0f;
        if (!std::isfinite(setpoint))
            throw std::runtime_error("Non-finite controller setpoint; stopping.");
        const float limit = mode == 0 ? CURRENT_LIMIT_A : VOLTAGE_LIMIT_V;
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
        //log mpc output values
        log.u_mpc[log.count] = r.u_apply;
        log.xhat_current[log.count] = r.xhat(0);
        log.xhat_speed[log.count] = r.xhat(1);
        log.xhat_pose[log.count] = r.xhat(2);
        log.xhat_dist[log.count] = r.xhat(3);
        log.mpc_status[log.count] = r.status;
        log.mpc_iters[log.count] = r.iters;
        log.mpc_fault[log.count] = r.fault;
        log.mpc_us[log.count] = mpc_us;
        ++log.count;
        previous = now;
        enabled = true; // First exchange always sends zero setpoint, disabled.
        if (stopped || log.count == log.spi.size())
            break;
        next_release += period_ns;
        // Keep the original time grid; discard releases crossed during work.
        const auto finished = monotonic_ns();
        if (finished >= next_release) {
            const auto missed = (finished - next_release) / period_ns + 1;
            log.skipped_releases += missed;
            next_release += missed * period_ns;
        }
        sleep_until(next_release);
        // A wake delayed by whole periods executes only the latest release.
        const auto missed = std::max(0LL, (monotonic_ns() - next_release) / period_ns);
        log.skipped_releases += missed;
        next_release += missed * period_ns;
    }
}

void report(const Log &log, double cpu_s, double wall_s)
{
    // Formatting and output happen only after the final disable and SPI close.
    std::cerr << "skipped_releases=" << log.skipped_releases << '\n';
    std::cout << "cycle,dt_n,dt_min_ms,dt_mean_ms,dt_max_ms,spi_n,spi_min_ms,spi_mean_ms,spi_max_ms,cpu_s,wall_s,time_"
                 "s,voltage_V,current_A,motor_position_rad,motor_velocity_rad_s,sent_setpoint,sent_enable,sent_mode,"
                 "u_mpc,xhat_current_A,xhat_speed_rad_s,xhat_pose_rad,xhat_dist,mpc_status,mpc_iters,mpc_fault,mpc_us\n"
              << std::fixed << std::setprecision(4);
    
    const size_t window = PRINT_EVERY ? PRINT_EVERY : std::max(log.count, size_t{1});
    for (size_t first = 0; first < log.count; first += window) {
        const auto last = std::min(first + window, log.count);
        std::cout << std::setprecision(4) << last;
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
        const auto i = last - 1; // Last-cycle data when timing windows are grouped.
        
        std::cout << ',' << cpu_s << ',' << wall_s << std::setprecision(6) << ',' << log.time_s[i] << ','
                  << log.voltage_V[i] << ',' << log.current_A[i] << ',' << log.motor_position_rad[i] << ','
                  << log.motor_velocity_rad_s[i] << ',' << log.sent_setpoint[i] << ','
                  << static_cast<int>(log.sent_enable[i]) << ',' << static_cast<int>(log.sent_mode[i]) << ',' << log.u_mpc[i] << ',' << log.xhat_current[i] << ','
                  << log.xhat_speed[i] << ',' << log.xhat_pose[i] << ',' << log.xhat_dist[i] << ','
                  << static_cast<int>(log.mpc_status[i]) << ',' << static_cast<int>(log.mpc_iters[i]) << ','
                  << static_cast<int>(log.mpc_fault[i]) << ',' << log.mpc_us[i] << '\n';
                  
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
