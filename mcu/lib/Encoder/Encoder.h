#ifndef ENCODER_H_
#define ENCODER_H_

#include "EncoderCounter.h"
#include "IIRFilter.h"

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f /* pi */
#endif

class Encoder
{
public:
    explicit Encoder(PinName enc_a_pin, PinName enc_b_pin, float counts_per_turn, float velocity_fcut, float Ts);
    virtual ~Encoder() = default;

    void reset();
    struct Signals {
        float position; // rad
        float velocity; // rad/s, first-order low-pass filtered
    };
    Signals update(float sign = 1.0f);

private:
    EncoderCounter m_EncoderCounter;

    int64_t m_counts;
    short m_count_previous;
    float m_rotation_gain;
    float m_Ts;
    IIRFilter m_velocity_filter;
};
#endif /* ENCODER_H_ */
