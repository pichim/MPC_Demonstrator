#include "IO_handler.h"
#include "mbed.h"
#include "fast_realtime_thread.h"
#include "SPISlaveDMA.h"

int main()
{
    IO_handler io_handler;
    SpiSlaveDMA spi(MPC_SPI_MOSI_PIN, MPC_SPI_MISO_PIN, MPC_SPI_SCK_PIN, MPC_SPI_NSS_PIN,
                   MPC_SPI_TRANSACTION_TIMEOUT_US,
                   MPC_SPI_PRIORITY, MPC_SPI_STACK_SIZE);
    fast_realtime_thread current_task(io_handler, spi, MPC_FAST_RT_PERIOD_US * 1.0e-6f);
    if (!spi.start() || !current_task.start_loop()) {
        printf("Startup failed; motor remains disabled.\n");
        // Keep objects alive; no current task was started on this failure path.
        while (true) ThisThread::sleep_for(1s);
    }

    while (true)
        ThisThread::sleep_for(500ms);
}
