#include "Encoder.h"

Encoder::Encoder(PinName enc_a_pin, PinName enc_b_pin, float counts_per_turn, float velocity_fcut, float Ts)
    : m_EncoderCounter(enc_a_pin, enc_b_pin)
    , m_rotation_gain(2.0f * M_PIf / counts_per_turn)
    , m_Ts(Ts)
{
    m_velocity_filter.lowPass1Init(velocity_fcut, Ts);
    reset();
}

void Encoder::reset()
{
    m_EncoderCounter.reset();
    m_counts = m_count_previous = m_EncoderCounter.read();
    m_velocity_filter.reset(0.0f);
}

Encoder::Signals Encoder::update(float sign)
{
    const int16_t count = m_EncoderCounter.read();
    int32_t delta = int32_t(count) - int32_t(m_count_previous);
    // Unwrap the 16-bit hardware counter (less than half a turn of the counter
    // between samples). Use explicit arithmetic instead of signed narrowing.
    if (delta > 32767)
        delta -= 65536;
    if (delta < -32768)
        delta += 65536;
    m_count_previous = count;
    m_counts += delta;
    const float velocity = sign * m_rotation_gain * static_cast<float>(delta) / m_Ts;
    return {sign * m_rotation_gain * static_cast<float>(m_counts), m_velocity_filter.apply(velocity)};
}
