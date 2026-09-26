#include "SPISlaveDMA.h"

// Keep HAL SPI MSP init empty; class configures pins/clocks/DMA itself.
extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) { (void)hspi; }


// ============================ CRC-8 (poly 0x07) ==============================
static const uint8_t CRC8_TAB[256] = {
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

// ============================ Helpers: DMA flag masks =========================
static inline bool is_stream_0_4(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream0 || s == DMA2_Stream0 || s == DMA1_Stream4 || s == DMA2_Stream4);
}
static inline bool is_stream_1_5(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream1 || s == DMA2_Stream1 || s == DMA1_Stream5 || s == DMA2_Stream5);
}
static inline bool is_stream_2_6(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream2 || s == DMA2_Stream2 || s == DMA1_Stream6 || s == DMA2_Stream6);
}
static inline bool is_stream_3_7(DMA_Stream_TypeDef* s) {
    return (s == DMA1_Stream3 || s == DMA2_Stream3 || s == DMA1_Stream7 || s == DMA2_Stream7);
}

uint32_t SpiSlaveDMA::tc_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_TCIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_TCIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_TCIF2_6;
    return DMA_FLAG_TCIF3_7;
}
uint32_t SpiSlaveDMA::ht_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_HTIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_HTIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_HTIF2_6;
    return DMA_FLAG_HTIF3_7;
}
uint32_t SpiSlaveDMA::te_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_TEIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_TEIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_TEIF2_6;
    return DMA_FLAG_TEIF3_7;
}
uint32_t SpiSlaveDMA::fe_flag_for(DMA_Stream_TypeDef* s) {
    if (is_stream_0_4(s)) return DMA_FLAG_FEIF0_4;
    if (is_stream_1_5(s)) return DMA_FLAG_FEIF1_5;
    if (is_stream_2_6(s)) return DMA_FLAG_FEIF2_6;
    return DMA_FLAG_FEIF3_7;
}

// ============================ Public API =====================================

SpiSlaveDMA::SpiSlaveDMA(PinName mosi,
                         PinName miso,
                         PinName sck,
                         PinName nss,
                         uint32_t transaction_timeout_us,
                         osPriority priority,
                         uint32_t stack_size)
    : m_Thread(priority, stack_size)
    , m_InterruptIn_NSS(nss)
    , m_transaction_timeout_us(transaction_timeout_us)
    , m_MOSI(mosi)
    , m_MISO(miso)
    , m_SCK(sck)
    , m_NSS(nss)
    , m_instance(inferInstance(mosi, miso, sck, nss))
{
    m_time_previous = us_ticker_read();

    std::memset(&m_hspi, 0, sizeof(m_hspi));
    // Instance set in configureGPIOandDMA() just before HAL_SPI_Init
    m_hspi.Init.Mode              = SPI_MODE_SLAVE;
    m_hspi.Init.Direction         = SPI_DIRECTION_2LINES;       // full-duplex
    m_hspi.Init.DataSize          = SPI_DATASIZE_8BIT;
    m_hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;           // CPOL=0
    m_hspi.Init.CLKPhase          = SPI_PHASE_1EDGE;            // CPHA=0 => mode 0
    m_hspi.Init.NSS               = SPI_NSS_HARD_INPUT;         // hardware NSS
    m_hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    m_hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    m_hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; // use our CRC-8
    m_hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;    // ignored in slave

    std::memset(&m_dma_rx, 0, sizeof(m_dma_rx));
    std::memset(&m_dma_tx, 0, sizeof(m_dma_tx));
}

// ----------- Instance inference (NUCLEO-F446RE pin map) -----------
SpiSlaveDMA::Instance SpiSlaveDMA::inferInstance(PinName mosi, PinName miso, PinName sck, PinName nss) {
    auto in = [](PinName p, std::initializer_list<PinName> set)
    {
        for (auto q : set)
            if (p == q)
                return true;
        return false;
    };

    // 1) Fast path: SCK is unique?
    if (sck == PA_5)                     return Instance::SPI_1;   // SPI1 only
    if (sck == PB_10 || sck == PB_13)    return Instance::SPI_2;   // SPI2 only
    if (sck == PC_10)                    return Instance::SPI_3;   // SPI3 only

    // 2) Ambiguous SCK = PB_3 (SPI1 or SPI3). Disambiguate by unique companions.
    if (sck == PB_3) {
        // Any uniquely-SPI3 companion? -> SPI3
        if (in(miso, {PC_11}) || in(mosi, {PC_12})) return Instance::SPI_3;
        // Any uniquely-SPI1 companion? -> SPI1
        if (in(miso, {PA_6}) || in(mosi, {PA_7}))   return Instance::SPI_1;
        // Inconsistent case guard: PB_3 SCK cannot go with SPI2-only NSS pins
        if (nss == PB_12 || nss == PB_9) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
                       "Inconsistent pins: SCK=PB_3 cannot be SPI2");
        }
        // Tie-breaker: default to SPI1 for PB_3/PB_4/PB_5 with PA_4/PA_15.
        return Instance::SPI_1;
    }

    // 3) No SCK match: pick by other unique signals (rare edge-cases).
    if (in(mosi, {PC_3}) || in(miso, {PC_2}) || nss == PB_12 || nss == PB_9) return Instance::SPI_2;
    if (in(mosi, {PC_12}) || in(miso, {PC_11}))                              return Instance::SPI_3;
    if (in(mosi, {PA_7})  || in(miso, {PA_6}))                               return Instance::SPI_1;

    MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
               "Cannot infer SPI instance from the provided pins");
    // Unreachable, but keeps some compilers happy.
    return Instance::SPI_2;
}

SpiSlaveDMA::~SpiSlaveDMA() {
    m_InterruptIn_NSS.rise(nullptr);
    m_InterruptIn_NSS.fall(nullptr);
    m_Thread.terminate();
    HAL_SPI_DMAStop(&m_hspi);
}

bool SpiSlaveDMA::start() {
    MBED_ASSERT(validatePins());
    MBED_ASSERT(m_transaction_timeout_us > 0);
    m_InterruptIn_NSS.rise(callback(this, &SpiSlaveDMA::handleNss));
    m_InterruptIn_NSS.fall(callback(this, &SpiSlaveDMA::handleNss));
    const uint32_t line = static_cast<uint32_t>(m_NSS) & 15U;
    const IRQn_Type irq = line < 5 ? static_cast<IRQn_Type>(static_cast<int>(EXTI0_IRQn)+line)
        : (line < 10 ? EXTI9_5_IRQn : EXTI15_10_IRQn);
    NVIC_SetPriority(irq, 5);
    configureGPIOandDMA();
    if (HAL_SPI_Init(&m_hspi) != HAL_OK) return false;
    return m_Thread.start(callback(this, &SpiSlaveDMA::threadTask)) == osOK;
}

void SpiSlaveDMA::setReplyData(float voltage, float current, float position, float velocity) {
    const float values[SPI_NUM_FLOATS] = {voltage, current, position, velocity};
    core_util_critical_section_enter();
    std::memcpy(m_reply_data, values, sizeof(values));
    core_util_critical_section_exit();
}

bool SpiSlaveDMA::takeCommand(SpiCommand &command) {
    core_util_critical_section_enter();
    const bool available = m_has_new_data;
    if (available) {
        command = m_command;
        m_has_new_data = false;
    }
    core_util_critical_section_exit();
    return available;
}

SpiDiagnostics SpiSlaveDMA::getDiagnostics() {
    core_util_critical_section_enter();
    const SpiDiagnostics result = m_diagnostics;
    core_util_critical_section_exit();
    return result;
}

// The ISR owns normal transfers. Recover state excludes it while the worker
// performs potentially blocking HAL recovery. Critical sections claim ownership.
void SpiSlaveDMA::threadTask() {
    while (true) {
        core_util_critical_section_enter();
        if (m_state == State::Active &&
            static_cast<uint32_t>(us_ticker_read()-m_armed_at) >= m_transaction_timeout_us) {
            ++m_diagnostics.failed_count;
            ++m_diagnostics.transaction_timeout_count;
            m_state = State::Recover;
        }
        const bool recover = m_state == State::Recover;
        core_util_critical_section_exit();
        if (recover) {
            resetSPIPeripheral();
            core_util_critical_section_enter();
            m_state = m_InterruptIn_NSS.read() ? State::Idle : State::WaitHigh;
            core_util_critical_section_exit();
        }
        ThisThread::flags_wait_any_for(m_ThreadFlag, 1ms);
    }
}

bool SpiSlaveDMA::processFrame() {
    // Called on NSS rising, after wire completion. TX DMA alone can finish
    // before the last byte leaves SPI, so both DMA completion and errors matter.
    const bool complete = __HAL_DMA_GET_FLAG(&m_dma_rx, m_rx_tc_flag) &&
                          __HAL_DMA_GET_FLAG(&m_dma_tx, m_tx_tc_flag);
    const bool peripheral_error = __HAL_SPI_GET_FLAG(&m_hspi, SPI_FLAG_OVR) ||
        __HAL_SPI_GET_FLAG(&m_hspi, SPI_FLAG_RXNE) ||
        __HAL_DMA_GET_FLAG(&m_dma_rx, m_rx_error_flags) ||
        __HAL_DMA_GET_FLAG(&m_dma_tx, m_tx_error_flags);
    const bool stopped = !(m_dma_rx.Instance->CR & DMA_SxCR_EN) &&
                         !(m_dma_tx.Instance->CR & DMA_SxCR_EN);
    if (!complete || !stopped || peripheral_error) {
        core_util_critical_section_enter();
        m_diagnostics.failed_count++;
        if (!complete || !stopped) m_diagnostics.incomplete_count++;
        if (peripheral_error) m_diagnostics.peripheral_error_count++;
        core_util_critical_section_exit();
        return false;
    }
    CLEAR_BIT(m_hspi.Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    m_dma_rx.State = HAL_DMA_STATE_READY;
    m_dma_tx.State = HAL_DMA_STATE_READY;
    m_hspi.State = HAL_SPI_STATE_READY;
    const uint8_t header = m_buffer_rx[0];
    bool valid = (header == SPI_HEADER_MASTER || header == SPI_HEADER_READ) &&
                 verifyChecksum(m_buffer_rx, SPI_MSG_SIZE - 1, m_buffer_rx[SPI_MSG_SIZE - 1]);
    if (header == SPI_HEADER_READ) {
        for (size_t i = 1; i < SPI_MSG_SIZE - 1; ++i)
            valid = valid && m_buffer_rx[i] == 0;
    }
    if (!valid) {
        core_util_critical_section_enter();
        m_diagnostics.failed_count++;
        m_diagnostics.invalid_frame_count++;
        core_util_critical_section_exit();
        return false;
    }
    if (header == SPI_HEADER_MASTER) {
        SpiCommand command;
        std::memcpy(command.data, &m_buffer_rx[1], sizeof(command.data));
        command.received_at_us = us_ticker_read();
        const uint32_t now = command.received_at_us;
        const uint32_t delta_us = now - m_time_previous;
        m_time_previous = now;
        core_util_critical_section_enter();
        m_command = command;
        m_diagnostics.message_count++;
        m_diagnostics.last_delta_time_us = delta_us;
        m_has_new_data = true;
        core_util_critical_section_exit();
    }
    return true;
}

void SpiSlaveDMA::buildTX() {
    core_util_critical_section_enter();
    m_buffer_tx[0] = SPI_HEADER_SLAVE;
    std::memcpy(&m_buffer_tx[1], m_reply_data, SPI_NUM_FLOATS * sizeof(float));
    core_util_critical_section_exit();

    m_buffer_tx[SPI_MSG_SIZE - 1] = calculateCRC8(m_buffer_tx, SPI_MSG_SIZE - 1);
}

bool SpiSlaveDMA::tryArmDmaFrame() {
    // Normal-mode streams must have stopped before their addresses/counts change.
    // Prepare the immutable frame before enabling DMA; do not wait in this ISR.
    if ((m_dma_rx.Instance->CR | m_dma_tx.Instance->CR) & DMA_SxCR_EN) return false;
    __HAL_DMA_CLEAR_FLAG(&m_dma_rx, m_rx_all_flags);
    __HAL_DMA_CLEAR_FLAG(&m_dma_tx, m_tx_all_flags);
    m_dma_rx.Instance->M0AR = reinterpret_cast<uint32_t>(m_buffer_rx);
    m_dma_tx.Instance->M0AR = reinterpret_cast<uint32_t>(m_buffer_tx);
    m_dma_rx.Instance->PAR = reinterpret_cast<uint32_t>(&m_hspi.Instance->DR);
    m_dma_tx.Instance->PAR = reinterpret_cast<uint32_t>(&m_hspi.Instance->DR);
    m_dma_rx.Instance->NDTR = SPI_MSG_SIZE;
    m_dma_tx.Instance->NDTR = SPI_MSG_SIZE;
    m_dma_rx.State = HAL_DMA_STATE_BUSY;
    m_dma_tx.State = HAL_DMA_STATE_BUSY;
    m_hspi.State = HAL_SPI_STATE_BUSY_TX_RX;
    __DMB();
    __HAL_DMA_ENABLE(&m_dma_rx);
    __HAL_DMA_ENABLE(&m_dma_tx);
    SET_BIT(m_hspi.Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    __HAL_SPI_ENABLE(&m_hspi);
    return true;
}

// ============================ CRC helpers =====================================

uint8_t SpiSlaveDMA::calculateCRC8(const uint8_t* buffer, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc = CRC8_TAB[crc ^ buffer[i]];
    }
    return crc;
}

bool SpiSlaveDMA::verifyChecksum(const uint8_t* buffer, size_t length, uint8_t expected_crc) {
    return (calculateCRC8(buffer, length) == expected_crc);
}

// ============================ ISR hook ========================================

void SpiSlaveDMA::handleNss() {
    const uint32_t started = us_ticker_read();
    if (!m_InterruptIn_NSS.read()) {
        if (m_state != State::Idle) { ++m_diagnostics.ignored_nss_count; return; }
        buildTX();
        if (!tryArmDmaFrame()) {
            ++m_diagnostics.arm_failure_count;
            m_state = State::Recover;
            m_Thread.flags_set(m_ThreadFlag);
            return;
        }
        m_armed_at = started;
        m_state = State::Active;
        const uint32_t elapsed = us_ticker_read()-started;
        if (elapsed > m_diagnostics.max_prepare_us)
            m_diagnostics.max_prepare_us = elapsed;
    } else {
        if (m_state == State::WaitHigh) { m_state = State::Idle; return; }
        if (m_state != State::Active) return;
        const bool expired = static_cast<uint32_t>(started - m_armed_at) >= m_transaction_timeout_us;
        if (expired) {
            ++m_diagnostics.failed_count;
            ++m_diagnostics.transaction_timeout_count;
        }
        if (!expired && processFrame()) {
            m_state = State::Idle;
        } else {
            m_state = State::Recover;
            if (resetFast()) m_state = State::Idle;
            else m_Thread.flags_set(m_ThreadFlag);
        }
        const uint32_t elapsed = us_ticker_read()-started;
        m_diagnostics.readout_time_us = elapsed;
    }
}

// ============================ HW config / pins / DMA ==========================

static void enable_gpio_clk_for(PinName pin) {
    if (pin == PA_4 || pin == PA_5 || pin == PA_6 || pin == PA_7 || pin == PA_15) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (pin == PB_3 || pin == PB_4 || pin == PB_5 || pin == PB_9 || pin == PB_10 || pin == PB_12 || pin == PB_13 || pin == PB_14 || pin == PB_15) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (pin == PC_2 || pin == PC_3 || pin == PC_10 || pin == PC_11 || pin == PC_12) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
}

void SpiSlaveDMA::configureGPIOandDMA() {
    // Clocks: GPIOs + SPIx + DMAx
    enable_gpio_clk_for(m_MOSI);
    enable_gpio_clk_for(m_MISO);
    enable_gpio_clk_for(m_SCK);
    enable_gpio_clk_for(m_NSS);

    if (m_instance == Instance::SPI_1) { __HAL_RCC_SPI1_CLK_ENABLE(); __HAL_RCC_DMA2_CLK_ENABLE(); }
    else                                { __HAL_RCC_DMA1_CLK_ENABLE(); } // SPI2/SPI3 use DMA1
    if (m_instance == Instance::SPI_2) { __HAL_RCC_SPI2_CLK_ENABLE(); }
    if (m_instance == Instance::SPI_3) { __HAL_RCC_SPI3_CLK_ENABLE(); }

    // AF: SPI1/SPI2 = AF5, SPI3 = AF6 (use instance-specific macro names)
    const uint32_t af =
        (m_instance == Instance::SPI_3) ? GPIO_AF6_SPI3 :
        (m_instance == Instance::SPI_2) ? GPIO_AF5_SPI2 : GPIO_AF5_SPI1;

    auto do_pin = [&](PinName pin) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_TypeDef* port = nullptr;
        uint16_t p = 0;

        // Map PinName to port + pinmask (only the pins we support)
        if (pin == PA_4 || pin == PA_5 || pin == PA_6 || pin == PA_7 || pin == PA_15) {
            port = GPIOA;
            p = (pin == PA_4 ) ? GPIO_PIN_4  :
                (pin == PA_5 ) ? GPIO_PIN_5  :
                (pin == PA_6 ) ? GPIO_PIN_6  :
                (pin == PA_7 ) ? GPIO_PIN_7  : GPIO_PIN_15;
        } else if (pin == PB_3 || pin == PB_4 || pin == PB_5 || pin == PB_9 || pin == PB_10 || pin == PB_12 || pin == PB_13 || pin == PB_14 || pin == PB_15) {
            port = GPIOB;
            p = (pin == PB_3 ) ? GPIO_PIN_3  :
                (pin == PB_4 ) ? GPIO_PIN_4  :
                (pin == PB_5 ) ? GPIO_PIN_5  :
                (pin == PB_9 ) ? GPIO_PIN_9  :
                (pin == PB_10) ? GPIO_PIN_10 :
                (pin == PB_12) ? GPIO_PIN_12 :
                (pin == PB_13) ? GPIO_PIN_13 :
                (pin == PB_14) ? GPIO_PIN_14 : GPIO_PIN_15;
        } else if (pin == PC_2 || pin == PC_3 || pin == PC_10 || pin == PC_11 || pin == PC_12) {
            port = GPIOC;
            p = (pin == PC_2 ) ? GPIO_PIN_2  :
                (pin == PC_3 ) ? GPIO_PIN_3  :
                (pin == PC_10) ? GPIO_PIN_10 :
                (pin == PC_11) ? GPIO_PIN_11 : GPIO_PIN_12;
        } else {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),
                       "Unsupported pin requested for SPI AF");
        }

        GPIO_InitStruct.Pin       = p;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = (pin == m_NSS) ? GPIO_PULLUP : GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = af;
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    };

    do_pin(m_SCK);
    do_pin(m_MISO);
    do_pin(m_MOSI);
    do_pin(m_NSS);

    // Select SPI instance for HAL
    m_hspi.Instance = (m_instance == Instance::SPI_1) ? SPI1 :
                      (m_instance == Instance::SPI_2) ? SPI2 : SPI3;

    // DMA mapping (STM32F446):
    //
    // SPI1 → DMA2: RX Stream0/Ch3, TX Stream3/Ch3
    // SPI2 → DMA1: RX Stream3/Ch0, TX Stream4/Ch0
    // SPI3 → DMA1: RX Stream0/Ch0, TX Stream5/Ch0
    if (m_instance == Instance::SPI_1) {
        // RX
        m_dma_rx.Instance                 = DMA2_Stream0;
        m_dma_rx.Init.Channel             = DMA_CHANNEL_3;
        m_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        m_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_rx.Init.Mode                = DMA_NORMAL;
        m_dma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_rx);
        __HAL_LINKDMA(&m_hspi, hdmarx, m_dma_rx);

        // TX
        m_dma_tx.Instance                 = DMA2_Stream3;
        m_dma_tx.Init.Channel             = DMA_CHANNEL_3;
        m_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        m_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_tx.Init.Mode                = DMA_NORMAL;
        m_dma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_tx);
        __HAL_LINKDMA(&m_hspi, hdmatx, m_dma_tx);
    }
    else if (m_instance == Instance::SPI_2) {
        // RX
        m_dma_rx.Instance                 = DMA1_Stream3;
        m_dma_rx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        m_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_rx.Init.Mode                = DMA_NORMAL;
        m_dma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_rx);
        __HAL_LINKDMA(&m_hspi, hdmarx, m_dma_rx);

        // TX
        m_dma_tx.Instance                 = DMA1_Stream4;
        m_dma_tx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        m_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_tx.Init.Mode                = DMA_NORMAL;
        m_dma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_tx);
        __HAL_LINKDMA(&m_hspi, hdmatx, m_dma_tx);
    }
    else { // SPI_3
        // RX
        m_dma_rx.Instance                 = DMA1_Stream0;
        m_dma_rx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        m_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_rx.Init.Mode                = DMA_NORMAL;
        m_dma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_rx);
        __HAL_LINKDMA(&m_hspi, hdmarx, m_dma_rx);

        // TX
        m_dma_tx.Instance                 = DMA1_Stream5;
        m_dma_tx.Init.Channel             = DMA_CHANNEL_0;
        m_dma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        m_dma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        m_dma_tx.Init.MemInc              = DMA_MINC_ENABLE;
        m_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        m_dma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        m_dma_tx.Init.Mode                = DMA_NORMAL;
        m_dma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        m_dma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&m_dma_tx);
        __HAL_LINKDMA(&m_hspi, hdmatx, m_dma_tx);
    }

    // Precompute flag masks for fast TC checks/clears
    const uint32_t te = te_flag_for(m_dma_rx.Instance);
    const uint32_t ht = ht_flag_for(m_dma_rx.Instance);
    const uint32_t fe = fe_flag_for(m_dma_rx.Instance);
    m_rx_tc_flag   = tc_flag_for(m_dma_rx.Instance);
    m_rx_error_flags = te | fe | __HAL_DMA_GET_DME_FLAG_INDEX(&m_dma_rx);
    m_rx_all_flags = m_rx_tc_flag | ht | m_rx_error_flags;

    const uint32_t te2 = te_flag_for(m_dma_tx.Instance);
    const uint32_t ht2 = ht_flag_for(m_dma_tx.Instance);
    const uint32_t fe2 = fe_flag_for(m_dma_tx.Instance);
    m_tx_tc_flag   = tc_flag_for(m_dma_tx.Instance);
    m_tx_error_flags = te2 | fe2 | __HAL_DMA_GET_DME_FLAG_INDEX(&m_dma_tx);
    m_tx_all_flags = m_tx_tc_flag | ht2 | m_tx_error_flags;
}

bool SpiSlaveDMA::validatePins() const {
    auto in = [](PinName p, std::initializer_list<PinName> set) {
        for (auto q : set)
            if (p == q)
                return true;
        return false;
    };

    bool sck_ok=false, miso_ok=false, mosi_ok=false, nss_ok=false;

    if (m_instance == Instance::SPI_1) {
        sck_ok  = in(m_SCK , {PA_5, PB_3});
        miso_ok = in(m_MISO, {PA_6, PB_4});
        mosi_ok = in(m_MOSI, {PA_7, PB_5});
        nss_ok  = in(m_NSS , {PA_4, PA_15});
    } else if (m_instance == Instance::SPI_2) {
        sck_ok  = in(m_SCK , {PB_10, PB_13});
        miso_ok = in(m_MISO, {PB_14, PC_2});
        mosi_ok = in(m_MOSI, {PB_15, PC_3});
        nss_ok  = in(m_NSS , {PB_9, PB_12});
    } else { // SPI_3
        sck_ok  = in(m_SCK , {PC_10, PB_3});
        miso_ok = in(m_MISO, {PC_11, PB_4});
        mosi_ok = in(m_MOSI, {PC_12, PB_5});
        nss_ok  = in(m_NSS , {PA_4, PA_15});
    }

    if (!(sck_ok && miso_ok && mosi_ok && nss_ok)) return false;

    if ((m_SCK == m_MISO) || (m_SCK == m_MOSI) || (m_SCK == m_NSS) ||
        (m_MISO == m_MOSI) || (m_MISO == m_NSS) ||
        (m_MOSI == m_NSS)) return false;

    return true;
}

void SpiSlaveDMA::resetSPIPeripheral() {
    // Reset only this SPI instance, not DMA1/2 (other peripherals may use it).
    HAL_SPI_DMAStop(&m_hspi);
    HAL_SPI_DeInit(&m_hspi);
    HAL_DMA_DeInit(&m_dma_rx);
    HAL_DMA_DeInit(&m_dma_tx);
    if (m_instance == Instance::SPI_1) {
        __HAL_RCC_SPI1_FORCE_RESET();
        __HAL_RCC_SPI1_RELEASE_RESET();
    } else if (m_instance == Instance::SPI_2) {
        __HAL_RCC_SPI2_FORCE_RESET();
        __HAL_RCC_SPI2_RELEASE_RESET();
    } else {
        __HAL_RCC_SPI3_FORCE_RESET();
        __HAL_RCC_SPI3_RELEASE_RESET();
    }
    configureGPIOandDMA();
    (void)HAL_SPI_Init(&m_hspi);
    core_util_critical_section_enter();
    m_diagnostics.recovery_count++;
    core_util_critical_section_exit();
}

// No polling or HAL calls: if a stream has not stopped, defer to the worker.
// Resetting the selected SPI clears stale DR/shift-register data after a short
// or oversized frame, without disturbing other DMA users or GPIO configuration.
bool SpiSlaveDMA::resetFast() {
    const uint32_t cr1 = m_hspi.Instance->CR1 & ~SPI_CR1_SPE;
    const uint32_t cr2 = m_hspi.Instance->CR2 & ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    CLEAR_BIT(m_hspi.Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    __HAL_DMA_DISABLE(&m_dma_rx);
    __HAL_DMA_DISABLE(&m_dma_tx);
    __DSB();
    if ((m_dma_rx.Instance->CR | m_dma_tx.Instance->CR) & DMA_SxCR_EN) return false;
    if (m_instance == Instance::SPI_1) {
        __HAL_RCC_SPI1_FORCE_RESET(); __HAL_RCC_SPI1_RELEASE_RESET();
    } else if (m_instance == Instance::SPI_2) {
        __HAL_RCC_SPI2_FORCE_RESET(); __HAL_RCC_SPI2_RELEASE_RESET();
    } else {
        __HAL_RCC_SPI3_FORCE_RESET(); __HAL_RCC_SPI3_RELEASE_RESET();
    }
    m_hspi.Instance->CR1 = cr1;
    m_hspi.Instance->CR2 = cr2;
    m_hspi.State = HAL_SPI_STATE_READY;
    m_dma_rx.State = HAL_DMA_STATE_READY;
    m_dma_tx.State = HAL_DMA_STATE_READY;
    ++m_diagnostics.recovery_count;
    return true;
}
