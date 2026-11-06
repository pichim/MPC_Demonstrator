#pragma once
#include <algorithm>
#include <cmath>

namespace planner {

constexpr double V_MAX = 70.0;
constexpr double A_MAX = 5200.0;
constexpr double TWO_PI = 6.283185307179586;
constexpr double SPIN_COUNT = 10.0;
constexpr double SPIN_HOLD_S = 2.0;
constexpr double CYCLE_SETTLE_S = 3.0;
constexpr double MAX_STEP_REV = 30.0;
constexpr double SPEED_MAX_LEAD_RAD = 10.0;

inline double trajectory(double t, double s_des, double s_start, double t0,
                         double v_max = V_MAX, double a_max = A_MAX)
{
    double D = s_des - s_start;
    if (std::abs(D) < 1e-9) return s_start;

    const double sgn = D > 0.0 ? 1.0 : -1.0;
    D = std::abs(D);

    double tAcc = v_max / a_max;
    const double Dacc = 0.5 * a_max * tAcc * tAcc;
    double vPeak, tConst;

    if (2.0 * Dacc >= D) {
        tAcc = std::sqrt(D / a_max);
        vPeak = a_max * tAcc;
        tConst = 0.0;
    } else {
        vPeak = v_max;
        tConst = (D - 2.0 * Dacc) / v_max;
    }

    const double tDec = vPeak / a_max;
    const double t1 = t0 + tAcc;
    const double t2 = t1 + tConst;
    const double t3 = t2 + tDec;
    const double qAcc = 0.5 * a_max * tAcc * tAcc;

    if (t <= t0) return s_start;

    if (t <= t1) {
        const double tau = t - t0;
        return s_start + sgn * 0.5 * a_max * tau * tau;
    }

    if (t <= t2) {
        const double tau = t - t1;
        return s_start + sgn * (qAcc + vPeak * tau);
    }

    if (t <= t3) {
        const double tau = t - t2;
        return s_start + sgn *
            (qAcc + vPeak * tConst + vPeak * tau
             - 0.5 * a_max * tau * tau);
    }

    return s_des;
}

inline double duration(double D, double v_max = V_MAX, double a_max = A_MAX)
{
    D = std::abs(D);
    if (D < 1e-9) return 0.0;

    const double tAcc = v_max / a_max;
    const double Dacc = 0.5 * a_max * tAcc * tAcc;

    if (2.0 * Dacc >= D)
        return 2.0 * std::sqrt(D / a_max);

    return 2.0 * tAcc + (D - 2.0 * Dacc) / v_max;
}

struct Params {
    bool speed = false;
    bool cycle = false;
    int direction = 1;
    double value = 0.0;
    double t0 = 1.0;
    double Ts = 1e-3;
};

struct Output {
    double theta_ref;
    double pos_ref;
    double vel_ref;
    bool done;
};

class Reference {
public:
    void start(const Params &p)
    {
        p_ = p;
        p_int_ = 0.0;
        init_ = false;
        theta_ref_ = 0.0;
    }

    Output step(double tr, double position)
    {
        const double nan = std::nan("");
        const double sgn = p_.direction;
        Output o{0.0, nan, nan, false};

        if (p_.speed) {
            const double v = sgn * (tr >= p_.t0 ? p_.value : 0.0);

            if (!init_) {
                p_int_ = position;
                init_ = true;
            }

            p_int_ += v * p_.Ts;
            p_int_ = std::clamp(
                p_int_,
                position - SPEED_MAX_LEAD_RAD,
                position + SPEED_MAX_LEAD_RAD
            );

            o.theta_ref = p_int_;
            o.vel_ref = v;
        } else {
            o.theta_ref = sgn * pos(tr);
            o.pos_ref = o.theta_ref;

            if (p_.cycle)
                o.done = tr >= cycleEnd() + CYCLE_SETTLE_S;
        }

        theta_ref_ = o.theta_ref;
        return o;
    }

    template <class Vec>
    void preview(Vec &out, int n, double tr) const
    {
        const double sgn = p_.direction;

        for (int j = 0; j < n; ++j) {
            out(j) = sgn * pos(tr + (j + 1) * p_.Ts);
        }
    }

private:
    static double cycleTarget()
    {
        return SPIN_COUNT * TWO_PI;
    }

    double cycleReturnStart() const
    {
        return std::max(
            p_.t0 + SPIN_HOLD_S,
            p_.t0 + duration(cycleTarget())
        );
    }

    double cycleEnd() const
    {
        return cycleReturnStart() + duration(cycleTarget());
    }

    double pos(double t) const
    {
        if (!p_.cycle) {
            return trajectory(t, p_.value, 0.0, p_.t0);
        }

        const double target = cycleTarget();
        const double t_back = cycleReturnStart();

        return t < t_back
            ? trajectory(t, target, 0.0, p_.t0)
            : trajectory(t, 0.0, target, t_back);
    }

    Params p_;
    double p_int_ = 0.0;
    bool init_ = false;
    double theta_ref_ = 0.0;
};

} // namespace planner


