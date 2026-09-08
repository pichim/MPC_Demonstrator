#pragma once

#include <chrono>

// #include "Chirp.h"
#include "IO_handler.h"
// #include "SerialStream.h"
#include "ThreadFlag.h"
#include "fast_realtime_thread.h"
#include "mbed.h"
#include "serial_pipe.h"

#define BAUD 115200 // Tested 230400 and 460800 not working, without implementing a protocol we just use safe 115200
#define WATCHDOG_TIMEOUT_SEC 0.3f // Watchdog timeout in seconds
// #define F0_HZ 0.05f
// #define T1_SEC 1 / F0_HZ
// #define OFFSET_V 6.0f

using namespace std::chrono;

class realtime_thread
{
public:
    realtime_thread(IO_handler &io, float Ts, float Ts_fast);
    virtual ~realtime_thread();
    void start_loop(void);

private:
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    float Ts;
    IO_handler &io_handler;
    SerialPipe serialPipe;
    fast_realtime_thread fast_rt_thread;

    // SerialStream m_SerialStream;
    // Timer m_Timer;
    // microseconds m_time_previous_us{0};
    // Chirp m_Chirp;
    // float m_sinarg{0.0f};

    void loop(void);
    void sendSignal() { thread.flags_set(threadFlag); }
};
