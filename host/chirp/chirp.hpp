#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

// Exponential chirp generator.

class Chirp
{
public:
    Chirp(const double f0, const double f1, const double t1, const double Ts)
    {
        init(f0, f1, t1, Ts);
    }

    // f0: start frequency [Hz]
    // f1: end frequency [Hz]
    // t1: signal length [s]
    // Ts: sampling time [s]
    void init(const double f0, const double f1, const double t1, const double Ts)
    {
        if (!(f0 > 0.0) || !(f1 > 0.0) || !(t1 > 0.0) || !(Ts > 0.0) ||
            !std::isfinite(f0) || !std::isfinite(f1) ||
            !std::isfinite(t1) || !std::isfinite(Ts) || f0 == f1) {
            throw std::invalid_argument("Invalid chirp parameters");
        }

        m_chirp.f0 = f0;
        m_chirp.Ts = Ts;

        const double samples = t1 / Ts;
        if (!std::isfinite(samples) || samples < 1.0 ||
            samples > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
            throw std::invalid_argument("Invalid chirp sample count");
        }

        m_chirp.N = static_cast<std::uint32_t>(samples);
        m_chirp.beta = std::pow(f1 / f0, 1.0 / t1);
        m_chirp.k0 = 2.0 * pi() / std::log(m_chirp.beta);
        m_chirp.k1 = m_chirp.k0 * f0;

        if (!std::isfinite(m_chirp.beta) || !std::isfinite(m_chirp.k0) ||
            !std::isfinite(m_chirp.k1)) {
            throw std::invalid_argument("Chirp parameters are numerically ill-conditioned");
        }

        reset();
    }


    void reset()
    {
        m_chirp.count = 0;
        m_chirp.isFinished = false;
        resetSignals();
    }

    bool update()
    {
        if (m_chirp.isFinished) {
            return false;
        } else if (m_chirp.count == m_chirp.N) {
            m_chirp.isFinished = true;
            resetSignals();
            return false;
        } else {
            m_chirp.fchirp =
                m_chirp.f0 * std::pow(m_chirp.beta,
                                      static_cast<double>(m_chirp.count) * m_chirp.Ts);
            m_chirp.sinarg = m_chirp.k0 * m_chirp.fchirp - m_chirp.k1;
            m_chirp.sinarg = std::fmod(m_chirp.sinarg, 2.0 * pi());
            m_chirp.exc = std::sin(m_chirp.sinarg);
            ++m_chirp.count;
            return true;
        }
    }

    double getFreq() const { return m_chirp.fchirp; }
    double getSinarg() const { return m_chirp.sinarg; }
    double getExc() const { return m_chirp.exc; }

private:
    struct ChirpData
    {
        double f0 = 0.0;
        double Ts = 0.0;
        std::uint32_t N = 0;
        std::uint32_t count = 0;
        double beta = 0.0;
        double k0 = 0.0;
        double k1 = 0.0;
        double fchirp = 0.0;
        double sinarg = 0.0;
        double exc = 0.0;
        bool isFinished = false;
    };

    static constexpr double pi() { return 3.141592653589793238462643383279502884; }

    void resetSignals()
    {
        m_chirp.exc = 0.0;
        m_chirp.fchirp = 0.0;
        m_chirp.sinarg = 0.0;
    }

    ChirpData m_chirp;
};
