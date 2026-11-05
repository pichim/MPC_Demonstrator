#define SPI_SPEED_HZ 30000000
#define PERIOD_US 1000
#define CURRENT_LIMIT_A 1.7f   
#define VOLTAGE_LIMIT_V 24.0f  
#define TELEM_DECIM 10         //100 Hz telemetry
#define REF_DELAY_S 1.0        
#define SPIN_COUNT 10.0        
#define SPIN_HOLD_S 2.0        
#define CYCLE_SETTLE_S 3.0     
#define LINK_TIMEOUT_S 1.0     
#define MAX_STEP_REV 30.0      
#define SPEED_MAX_LEAD_RAD 10.0

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>

#include "spi_nss.h"
#include "mpc_solver.hpp"
#include "mpc_solver_iRef.hpp"
#include "csv_utils.hpp"
#include <Eigen/Dense>

namespace {

enum State : int { IDLE = 0, RUNNING = 1, FAULT = 2 };
const char *state_name(int s) { return s == RUNNING ? "RUNNING" : s == FAULT ? "FAULT" : "IDLE"; }

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

std::atomic<int> g_state{IDLE};
std::atomic<long long> g_last_rx_ns{0};  
std::string g_matrix_root = "/home/pi/MPC_Demonstrator/host";

struct Config {
    int mode = 1;                 
    bool speed_ctrl = false;     
    int direction = 1;                  
    bool cycle = false;           
    double value = 3.0;           
};

enum class Cmd { None, Cfg, Start, Stop, Estop };

struct Mailbox {                  
    std::mutex m;
    Cmd cmd = Cmd::None;
    Config cfg;
    std::atomic<bool> pending{false};
} g_box;

std::mutex g_msg_m;
std::string g_msg;              
void post_error(const std::string &s)
{
    std::lock_guard<std::mutex> l(g_msg_m);
    g_msg = s;
}

void low_priority_thread()        
{
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int c = 0; c < 3; ++c) CPU_SET(c, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
        std::cerr << "warning: could not move I/O thread off the control core\n";
    sched_param sp{};
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
}

void reader_thread()
{
    low_priority_thread();
    std::string line;
    Config cfg;
    while (std::getline(std::cin, line)) {
        g_last_rx_ns = monotonic_ns();   
        std::istringstream in(line);
        std::string word;
        in >> word;
        Cmd cmd = Cmd::None;
        if (word == "CFG") {
            std::string kv;
            while (in >> kv) {
                const auto eq = kv.find('=');
                if (eq == std::string::npos) continue;
                const std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
                try {
                    if (k == "mode") cfg.mode = (v == "voltage") ? 1 : 0;
                    else if (k == "ctrl") cfg.speed_ctrl = (v == "speed");
                    else if (k == "dir") cfg.direction = std::stoi(v) >= 0 ? 1 : -1;
                    else if (k == "ref") cfg.cycle = (v == "cycle");
                    else if (k == "value") cfg.value = std::stod(v);
                } catch (...) { post_error("bad CFG field: " + kv); }
            }
            cmd = Cmd::Cfg;
        } else if (word == "START") cmd = Cmd::Start;
        else if (word == "STOP") cmd = Cmd::Stop;
        else if (word == "ESTOP") cmd = Cmd::Estop;
        else continue;

        std::lock_guard<std::mutex> l(g_box.m);
        if (cmd != Cmd::Cfg && (g_box.cmd != Cmd::Estop || cmd == Cmd::Estop)) g_box.cmd = cmd;
        g_box.cfg = cfg;
        g_box.pending = true;
    }
    g_stop = 1;                   
}


struct Sample {
    double t, pos_ref, vel_ref, pos, vel, i_q, u, mpc_iter, fault, mpc_us, spi_us;
};
constexpr size_t RING = 1024;
Sample g_ring[RING];
std::atomic<size_t> g_head{0}, g_tail{0};

void push_sample(const Sample &s)  
{
    const size_t h = g_head.load(std::memory_order_relaxed);
    if (h - g_tail.load(std::memory_order_acquire) >= RING) return;  // full: drop
    g_ring[h % RING] = s;
    g_head.store(h + 1, std::memory_order_release);
}

void writer_thread()
{
    low_priority_thread();
    int last_state = -1;
    char buf[256];
    std::fputs("H mpc_server 1\n", stdout);
    std::fflush(stdout);
    for (;;) {
        const bool done = g_stop;
        const int st = g_state.load();
        if (st != last_state) {
            std::printf("S %s\n", state_name(st));
            last_state = st;
        }
        {
            std::lock_guard<std::mutex> l(g_msg_m);
            if (!g_msg.empty()) { std::printf("E %s\n", g_msg.c_str()); g_msg.clear(); }
        }
        size_t t = g_tail.load(std::memory_order_relaxed);
        const size_t h = g_head.load(std::memory_order_acquire);
        for (; t != h; ++t) {
            const Sample &s = g_ring[t % RING];
            std::snprintf(buf, sizeof buf, "T %.4f %.6g %.6g %.6g %.6g %.6g %.6g %.0f %.0f %.1f %.1f\n", s.t,
                          s.pos_ref, s.vel_ref, s.pos, s.vel, s.i_q, s.u, s.mpc_iter, s.fault, s.mpc_us, s.spi_us);
            std::fputs(buf, stdout);
        }
        g_tail.store(t, std::memory_order_release);
        std::fflush(stdout);
        if (done && g_tail.load() == g_head.load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

protocol::Measurements read_measurements(NssSPI &spi)
{
    return protocol::decode(spi.transfer(protocol::frame(protocol::Read)));
}

void send_command(NssSPI &spi, float setpoint, bool enable, int mode)
{
    protocol::decode(spi.transfer(
        protocol::frame(protocol::Command, {setpoint, enable ? 1.0f : 0.0f, static_cast<float>(mode), 0.0f})));
}

void sleep_until(long long deadline)
{
    const timespec target{deadline / 1'000'000'000, deadline % 1'000'000'000};
    int error;
    do {
        if (g_stop) return;
        error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr);
    } while (error == EINTR && !g_stop);
    if (error != 0 && error != EINTR)
        throw std::system_error(error, std::generic_category(), "clock_nanosleep");
}

template <class C>
std::unique_ptr<C> make_controller(const std::string &dir)
{
    Eigen::MatrixXd H      = loadMatrixCSV(dir + "/H.csv").cast<double>();
    Eigen::MatrixXd Aineq  = loadMatrixCSV(dir + "/Aineq.csv").cast<double>();
    Eigen::MatrixXd Fx     = loadMatrixCSV(dir + "/Fx.csv").cast<double>();
    Eigen::VectorXd Fu0    = loadVectorCSV(dir + "/Fu0.csv").cast<double>();
    Eigen::VectorXd Fd     = loadVectorCSV(dir + "/Fd.csv").cast<double>();
    Eigen::MatrixXd Fr     = loadMatrixCSV(dir + "/Fr.csv").cast<double>();
    Eigen::MatrixXd Ex     = loadMatrixCSV(dir + "/Ex.csv").cast<double>();
    Eigen::VectorXd Eu0    = loadVectorCSV(dir + "/Eu0.csv").cast<double>();
    Eigen::VectorXd Ed     = loadVectorCSV(dir + "/Ed.csv").cast<double>();
    Eigen::VectorXd bconst = loadVectorCSV(dir + "/bconst.csv").cast<double>();
    Eigen::MatrixXd Aaug   = loadMatrixCSV(dir + "/Aaug.csv").cast<double>();
    Eigen::VectorXd Baug   = loadVectorCSV(dir + "/Baug.csv").cast<double>();
    Eigen::MatrixXd Caug   = loadMatrixCSV(dir + "/Caug.csv").cast<double>();
    Eigen::MatrixXd Lgain  = loadMatrixCSV(dir + "/Lgain.csv").cast<double>();
    CsvTable meta = loadCsvTable(dir + "/meta.csv");
    const double umx = meta.rows[0][colIndex(meta, "umx")];
    return std::make_unique<C>(H, Aineq, Fx, Fu0, Fd, Fr, Ex, Eu0, Ed, bconst, umx, Aaug, Baug, Caug, Lgain);
}

struct SpeedToPosRef {
    double p_int = 0.0;
    bool init = false;
    double step(double v_ref, double p_meas, double Ts, double max_lead)
    {
        if (!init) { p_int = p_meas; init = true; }
        p_int += v_ref * Ts;
        p_int = std::clamp(p_int, p_meas - max_lead, p_meas + max_lead);
        return p_int;
    }
};

struct RefOut { double pos, vel; bool done; };

RefOut reference(const Config &c, double tr, SpeedToPosRef &s2p, double position)
{
    const double tc = tr - REF_DELAY_S;
    const double nan = std::nan("");
    if (c.speed_ctrl) {
        const double v = tc >= 0 ? c.value : 0.0;
        return {nan, v, false};   
    }
    (void)s2p; (void)position;
    if (!c.cycle) {
        const double lim = MAX_STEP_REV * 2.0 * M_PI;
        return {tc >= 0 ? std::clamp(c.value, -lim, lim) : 0.0, nan, false};
    }
    
    const double target = SPIN_COUNT * 2.0 * M_PI;
    if (tc < 0) return {0.0, nan, false};
    if (tc < SPIN_HOLD_S) return {target, nan, false};
    return {0.0, nan, tc >= SPIN_HOLD_S + CYCLE_SETTLE_S};
}


void run(NssSPI &spi)
{
    const auto process_start = monotonic_ns();
    g_last_rx_ns = process_start;
    constexpr long long period_ns = PERIOD_US * 1000LL;
    auto next_release = process_start;
    uint64_t skipped = 0;

    Config cfg;                                   
    int state = IDLE;
    std::unique_ptr<MpcController> ctrl_v;
    std::unique_ptr<MpcControllerCurr> ctrl_i;
    Config run_cfg;                               
    double theta0 = 0.0;
    SpeedToPosRef s2p;  
    bool have_theta0 = false;
    long long run_start = 0;
    uint64_t run_cycles = 0;

    Sample acc{};
    int acc_n = 0;
    double last_pos_ref = std::nan(""), last_vel_ref = std::nan(""), last_u = 0.0;

    auto fail = [&](const std::string &why) {
        post_error(why);
        state = FAULT;
        g_state = FAULT;
        ctrl_v.reset();
        ctrl_i.reset();
    };

    while (!g_stop) {
        const auto start = monotonic_ns();
        const auto meas = read_measurements(spi);
        const auto read_end = monotonic_ns();
        if (g_stop) break;

        if (!have_theta0) { theta0 = meas[2]; have_theta0 = true; }

        if (g_box.pending.exchange(false)) {
            Cmd cmd;
            {
                std::lock_guard<std::mutex> l(g_box.m);
                cmd = g_box.cmd;
                g_box.cmd = Cmd::None;
                cfg = g_box.cfg;
            }
            if (cmd == Cmd::Estop) {
                state = FAULT; g_state = FAULT;
                ctrl_v.reset(); ctrl_i.reset();
            } else if (cmd == Cmd::Stop) {
                state = IDLE; g_state = IDLE;
                ctrl_v.reset(); ctrl_i.reset();
            } else if (cmd == Cmd::Start && state != RUNNING) {
                try {
                    run_cfg = cfg;
                    const std::string root = g_matrix_root;
                    if (run_cfg.mode == 1) { ctrl_v = make_controller<MpcController>(root + "/mpc_matrices"); ctrl_i.reset(); }
                    else                   { ctrl_i = make_controller<MpcControllerCurr>(root + "/mpc_matrices_curr"); ctrl_v.reset(); }
                    s2p = SpeedToPosRef{};
                    theta0 = meas[2];                 
                    run_start = monotonic_ns();
                    run_cycles = 0;
                    state = RUNNING; g_state = RUNNING;
                } catch (const std::exception &e) {
                    fail(std::string("START failed: ") + e.what());
                }
            }
        }

        if (state == RUNNING && (read_end - g_last_rx_ns.load()) > static_cast<long long>(LINK_TIMEOUT_S * 1e9))
            fail("Link to the GUI lost - run stopped.");

        const double position = meas[2] - theta0;
        double mpc_us = 0.0;
        float setpoint = 0.0f;
        bool enabled = false;
        int mode = state == RUNNING ? run_cfg.mode : 1;
        int solver_iters = 0, solver_fault = 0;

        if (state == RUNNING) {
            const double tr = (read_end - run_start) / 1e9;
            const double sgn = run_cfg.direction;
            RefOut r = reference(run_cfg, tr, s2p, position);
            double theta_ref;
            if (run_cfg.speed_ctrl) {
                const double v_ref = sgn * r.vel;
                theta_ref = s2p.step(v_ref, position, PERIOD_US * 1e-6, SPEED_MAX_LEAD_RAD);
                last_vel_ref = v_ref; last_pos_ref = std::nan("");
            } else {
                theta_ref = sgn * r.pos;
                last_pos_ref = theta_ref; last_vel_ref = std::nan("");
            }
            if (r.done) {
                state = IDLE; g_state = IDLE;
                ctrl_v.reset(); ctrl_i.reset();
            } else {
                const auto t0 = monotonic_ns();
                double u_apply; int iters, flt;
                if (run_cfg.mode == 1) {
                    auto res = ctrl_v->step(Eigen::Vector2d(meas[1], position), theta_ref);
                    u_apply = res.u_apply; iters = res.iters; flt = res.fault;
                } else {
                    auto res = ctrl_i->step(position, theta_ref);
                    u_apply = res.u_apply; iters = res.iters; flt = res.fault;
                }
                mpc_us = (monotonic_ns() - t0) / 1e3;
                solver_iters = iters; solver_fault = flt;
                if (!std::isfinite(u_apply)) {
                    fail("Non-finite controller output; stopped.");
                } else {
                    const float limit = run_cfg.mode == 0 ? CURRENT_LIMIT_A : VOLTAGE_LIMIT_V;
                    setpoint = std::clamp(static_cast<float>(u_apply), -limit, limit);
                    enabled = run_cycles > 0;         
                    ++run_cycles;
                    if (!enabled) setpoint = 0.0f;
                }
            }
        }
        if (state != RUNNING) { last_pos_ref = last_vel_ref = std::nan(""); }

        const auto command_start = monotonic_ns();
        send_command(spi, setpoint, enabled, mode);
        const auto now = monotonic_ns();
        last_u = setpoint;

        const double spi_us = ((read_end - start) + (now - command_start)) / 1e3;
        acc.mpc_iter = std::max<double>(acc.mpc_iter, solver_iters);
        acc.fault = std::max<double>(acc.fault, solver_fault);
        acc.mpc_us = std::max(acc.mpc_us, mpc_us);
        acc.spi_us = std::max(acc.spi_us, spi_us);
        if (++acc_n >= TELEM_DECIM) {
            acc.t = (read_end - process_start) / 1e9;
            acc.pos_ref = last_pos_ref;
            acc.vel_ref = last_vel_ref;
            acc.pos = position;
            acc.vel = meas[3];
            acc.i_q = meas[1];
            acc.u = last_u;
            push_sample(acc);
            acc = Sample{};
            acc_n = 0;
        }

        next_release += period_ns;
        const auto finished = monotonic_ns();
        if (finished >= next_release) {
            const auto missed = (finished - next_release) / period_ns + 1;
            skipped += missed;
            next_release += missed * period_ns;
        }
        sleep_until(next_release);
        const auto missed = std::max(0LL, (monotonic_ns() - next_release) / period_ns);
        skipped += missed;
        next_release += missed * period_ns;
    }
    std::cerr << "skipped_releases=" << skipped << '\n';
}

} // namespace

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--matrices") && i + 1 < argc) g_matrix_root = argv[++i];
        else { std::cerr << "usage: mpc_server [--matrices <host dir containing mpc_matrices*>]\n"; return 1; }
    }
    int status = 0;
    try {
        struct sigaction action{};
        action.sa_handler = on_signal;
        sigemptyset(&action.sa_mask);
        if (sigaction(SIGINT, &action, nullptr) || sigaction(SIGTERM, &action, nullptr) ||
            sigaction(SIGHUP, &action, nullptr))
            throw std::system_error(errno, std::generic_category(), "sigaction");

        std::thread writer(writer_thread);
        std::thread reader(reader_thread);
        reader.detach();                                  
        {
            NssSPI spi(SPI_SPEED_HZ);
            try {
                run(spi);
            } catch (const std::exception &e) {
                post_error(std::string("control loop: ") + e.what());
                g_state = FAULT;
                status = 1;
            }
            try {
                send_command(spi, 0.0f, false, 1);        // final disable, always
            } catch (const std::exception &e) {
                std::cerr << "Final disable unconfirmed: " << e.what() << '\n';
                status = 1;
            }
        }
        g_stop = 1;
        writer.join();
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return status;
}


