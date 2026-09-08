#include "IO_handler.h"
#include "mbed.h"
#include "realtime_thread.h"

// Note:
// - You find the compiled firmware in the firmware folder. Make sure to update the firmware if you make changes to the code.

int main()
{
    // Input-Output handler
    IO_handler io_handler;

    // Real-time thread with sampling time Ts
    float Ts = 200.0e-6f;
    float Ts_fast = 50.0e-6f;
    realtime_thread rt_thread(io_handler, Ts, Ts_fast);
    rt_thread.start_loop();

    while (true)
        ThisThread::sleep_for(500ms);
}
