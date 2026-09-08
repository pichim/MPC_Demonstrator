#ifndef ENCODER_H_
#define ENCODER_H_

#include "EncoderCounter.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

class Encoder
{
public:
    explicit Encoder(PinName enc_a_pin,
                     PinName enc_b_pin,
                     float counts_per_turn);
    virtual ~Encoder() = default;

    void reset();
    float getAngleRad(float sign = 1.0f);

private:
    EncoderCounter m_EncoderCounter;

    long m_counts;
    short m_count_previous;
    float m_rotation_gain;
};
#endif /* ENCODER_H_ */
