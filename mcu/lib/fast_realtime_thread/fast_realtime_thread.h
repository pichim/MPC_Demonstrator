#pragma once

#include <chrono>

#include "CurrentCommand.h"
#include "IIRFilter.h"
#include "IO_handler.h"
#include "PIDCntrl.h"
#include "SPISlaveDMA.h"
#include "ThreadFlag.h"
#include "config.h"
#include "mbed.h"

#if MPC_PERFORM_GPA_MEAS
#include "GPA.h"
#endif

using namespace std::chrono;

class fast_realtime_thread
{
public:
    fast_realtime_thread(IO_handler &io, SpiSlaveDMA &spi, float Ts);
    virtual ~fast_realtime_thread();
    bool start_loop(void);

private:
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    float Ts;
    IO_handler &io_handler;
    SpiSlaveDMA &spi;
    IIRFilter notchEncoders[2];
    IIRFilter lowPass2CurrentSetpoint;
    PIDCntrl pidCntrl;

#if MPC_PERFORM_GPA_MEAS
    GPA m_GPA;
#endif

    void loop(void);
    void sendSignal();
    float clamp(float val, float min, float max);
};
