#pragma once
// Choose the reference with REF_TYPE below (only one is active per run).

#define REF_STEP 0   // single 2*pi step
#define REF_SPINS 1  // N spins out, hold, N spins back
#define REF_TYPE REF_SPINS

namespace ref {

constexpr double TWO_PI = 6.28318530717958647692;

//reference 1: single step
constexpr double STEP_START_S = 1.0;       // time of the step [s]
constexpr double STEP_VALUE_RAD = TWO_PI;  // step height [rad]

inline double step(double t) { return t >= STEP_START_S ? STEP_VALUE_RAD : 0.0; }

//reference 2: N spins in one direction, hold, then N spins back
constexpr double SPIN_START_S = 1.0;
constexpr double SPIN_COUNT = 10.0;
constexpr double SPIN_HOLD_S = 2.0;

inline double spins(double t)
{
    if (t < SPIN_START_S)
        return 0.0;
    if (t < SPIN_START_S + SPIN_HOLD_S)
        return SPIN_COUNT * TWO_PI;
    return 0.0;
}

inline double reference(double t)
{
#if REF_TYPE == REF_STEP
    return step(t);
#elif REF_TYPE == REF_SPINS
    return spins(t);
#else
#error "REF_TYPE must be REF_STEP or REF_SPINS"
#endif
}

}  // namespace ref
