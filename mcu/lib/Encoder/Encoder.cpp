#include "Encoder.h"

Encoder::Encoder(PinName enc_a_pin,
                 PinName enc_b_pin,
                 float counts_per_turn) : m_EncoderCounter(enc_a_pin, enc_b_pin)
                                        , m_rotation_gain(2.0f * M_PIf / counts_per_turn)
{
    reset();
}

void Encoder::reset()
{
    m_EncoderCounter.reset();
    m_counts = m_count_previous = m_EncoderCounter.read();
}

float Encoder::getAngleRad(float sign)
{
    // avoid overflow by using short for counts
    const short count = m_EncoderCounter.read();
    const short count_delta = count - m_count_previous;
    m_count_previous = count;

    // total counts
    m_counts += count_delta;

    // rotations
    return sign * m_rotation_gain * static_cast<float>(m_counts);
}
