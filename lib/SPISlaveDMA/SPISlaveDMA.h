/**
 * NSS-framed full-duplex SPI slave. Master provides setup and inactive intervals.
 * NSS falling freezes the latest telemetry into an immutable DMA frame.
 * NSS rising validates READ (0x57) or COMMAND (0x55); only COMMAND publishes data.
 * Normal transfers and bounded resets run in the NSS IRQ; worker handles timeout
 * and fallback recovery. A valid telemetry reply is not a command application ACK.
 */

#ifndef SPI_SLAVE_DMA_H_
#define SPI_SLAVE_DMA_H_

#include "mbed.h"
#include "ThreadFlag.h"

extern "C" {
#include "stm32f4xx_hal.h"
}

using namespace std::chrono;

// ----------------------------- Protocol constants -----------------------------
#define SPI_HEADER_MASTER   0x55 // publish (Pi->STM)
#define SPI_HEADER_READ     0x57 // read-only (Pi->STM), zero payload
#define SPI_HEADER_SLAVE    0x45 // data-from-STM (STM->Pi)
#define SPI_NUM_FLOATS      3
#define SPI_MSG_SIZE        (1 + SPI_NUM_FLOATS * 4 + 1) // header + floats + crc

// Single latest command; reception time is on the MCU microsecond ticker.
struct SpiCommand {
    float data[SPI_NUM_FLOATS]{};
    uint32_t received_at_us{0};
};

struct SpiDiagnostics {
    uint32_t message_count{0};
    uint32_t failed_count{0};
    uint32_t last_delta_time_us{0};
    uint32_t readout_time_us{0}; // Completion processing, inside NSS completion handler
    uint32_t incomplete_count{0};
    uint32_t invalid_frame_count{0};
    uint32_t peripheral_error_count{0};
    uint32_t recovery_count{0};
    uint32_t arm_failure_count{0};
    uint32_t max_prepare_us{0}; // ISR preparation only; excludes edge-to-ISR latency
    uint32_t transaction_timeout_count{0};
    uint32_t ignored_nss_count{0};
};

// ----------------------------- SpiSlaveDMA ------------------------------------
class SpiSlaveDMA {
public:
    // Public API: auto-detect SPI instance from the provided pins
    explicit SpiSlaveDMA(PinName mosi,
                         PinName miso,
                         PinName sck,
                         PinName nss,
                         uint32_t transaction_timeout_us,
                         osPriority priority,
                         uint32_t stack_size);
    ~SpiSlaveDMA();

    // Start in recovery; require NSS high before accepting its next falling edge.
    bool start();

    // Update 3 floats (thread-safe; copied atomically into next TX frame)
    void setReplyData(float f0, float f1, float f2);

    // Atomically consume the latest command, if any. Intermediate commands
    // may be overwritten; there is no FIFO. Diagnostics are a separate snapshot.
    bool takeCommand(SpiCommand &command);
    SpiDiagnostics getDiagnostics();

private:
    // Internal instance tag (inferred from pins)
    enum class Instance { SPI_1, SPI_2, SPI_3 };
    static Instance inferInstance(PinName mosi, PinName miso, PinName sck, PinName nss);

    // Recover: worker owns hardware; WaitHigh: discard the old selection;
    // Idle: next falling edge may arm DMA; Active: DMA buffers belong to SPI.
    enum class State { Recover, WaitHigh, Idle, Active };
    volatile State m_state{State::Recover};
    uint32_t m_armed_at{0};
    // ====================== RTOS/Mbed =======================
    Thread       m_Thread;            // worker thread
    InterruptIn  m_InterruptIn_NSS;   // select and completion edges
    ThreadFlag   m_ThreadFlag;
    const uint32_t m_transaction_timeout_us;

    // ====================== Selected pins ===================
    PinName      m_MOSI;
    PinName      m_MISO;
    PinName      m_SCK;
    PinName      m_NSS;

    // ====================== SPI selection ===================
    Instance     m_instance;

    // ====================== Timing ==========================
    uint32_t m_time_previous{0}; // us_ticker_read(); unsigned deltas tolerate wrap

    // ====================== Reply & State ===================
    float   m_reply_data[SPI_NUM_FLOATS] = {};
    SpiCommand m_command;
    SpiDiagnostics m_diagnostics;
    bool m_has_new_data{false};

    // Dedicated DMA storage: the current task updates m_reply_data, never these.
    alignas(4) uint8_t m_buffer_rx[SPI_MSG_SIZE];
    alignas(4) uint8_t m_buffer_tx[SPI_MSG_SIZE];

    // ====================== HAL Handles =====================
    SPI_HandleTypeDef  m_hspi;   // SPI1/2/3 selected by m_instance
    DMA_HandleTypeDef  m_dma_rx;
    DMA_HandleTypeDef  m_dma_tx;

    // ====================== DMA Flag Masks ==================
    uint32_t m_rx_tc_flag{0};
    uint32_t m_tx_tc_flag{0};
    uint32_t m_rx_error_flags{0};
    uint32_t m_tx_error_flags{0};
    uint32_t m_rx_all_flags{0};
    uint32_t m_tx_all_flags{0};

private:
    // ---------- Worker flow ----------
    void threadTask();
    void buildTX();
    bool tryArmDmaFrame();

    // ---------- CRC helpers ----------
    static uint8_t calculateCRC8(const uint8_t* buf, size_t len);
    static bool    verifyChecksum(const uint8_t* buf, size_t len, uint8_t expected_crc);

    // ---------- ISR hook ------------
    void handleNss();

    // ---------- HW config -----------
    void configureGPIOandDMA();
    bool validatePins() const;
    void resetSPIPeripheral();
    bool processFrame();
    bool resetFast();

    // ---------- Flag helpers (per DMA stream group) ----------
    static uint32_t tc_flag_for(DMA_Stream_TypeDef* s);
    static uint32_t ht_flag_for(DMA_Stream_TypeDef* s);
    static uint32_t te_flag_for(DMA_Stream_TypeDef* s);
    static uint32_t fe_flag_for(DMA_Stream_TypeDef* s);
};

#endif // SPI_SLAVE_DMA_H_
