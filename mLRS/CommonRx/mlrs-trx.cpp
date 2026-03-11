//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Unified transparent serial transceiver for identical modules
//*******************************************************

#if defined(FIRMWARE_MATEK_MR900_30_G431KB)

#define DBG_MAIN(x)
#define DEBUG_ENABLED
#define FAIL_ENABLED

// Keep IRQ priorities aligned with existing RX target
#define UARTB_IRQ_PRIORITY          11
#define UART_IRQ_PRIORITY           12
#define UARTF_IRQ_PRIORITY          11
#define SX_DIO_EXTI_IRQ_PRIORITY    13
#define SX2_DIO_EXTI_IRQ_PRIORITY   13
#define SWUART_TIM_IRQ_PRIORITY      9
#define FDCAN_IRQ_PRIORITY          14

#include "../Common/common_conf.h"
#include "../Common/common_types.h"

#include "../Common/hal/glue.h"
#include "../modules/stm32ll-lib/src/stdstm32.h"
#include "../modules/stm32ll-lib/src/stdstm32-peripherals.h"
#include "../modules/stm32ll-lib/src/stdstm32-mcu.h"
#include "../modules/stm32ll-lib/src/stdstm32-dac.h"
#include "../modules/stm32ll-lib/src/stdstm32-stack.h"
#ifdef STM32WL
#include "../modules/stm32ll-lib/src/stdstm32-subghz.h"
#endif
#include "../Common/hal/hal.h"
#include "../modules/stm32ll-lib/src/stdstm32-delay.h"
#include "../modules/stm32ll-lib/src/stdstm32-eeprom.h"
#include "../modules/stm32ll-lib/src/stdstm32-spi.h"
#ifdef USE_SX2
#include "../modules/stm32ll-lib/src/stdstm32-spib.h"
#endif
#ifdef USE_SERIAL
#include "../modules/stm32ll-lib/src/stdstm32-uartb.h"
#endif
#ifdef USE_COM
#include "../modules/stm32ll-lib/src/stdstm32-uartc.h"
#endif
#ifdef USE_DEBUG
#ifdef DEVICE_HAS_DEBUG_SWUART
#include "../modules/stm32ll-lib/src/stdstm32-uart-sw.h"
#else
#include "../modules/stm32ll-lib/src/stdstm32-uartf.h"
#endif
#endif
#include "../Common/hal/timer.h"

#include "../Common/sx-drivers/sx12xx.h"
#include "../Common/setup.h"
#include "../Common/common.h"
#include "../Common/arq.h"
#ifdef USE_COM
#include "../CommonTx/info.h"
#include "../CommonTx/cli.h"
#endif


// We run a symmetric transparent serial link on identical hardware.
// During discovery, both sides exchange UID beacons and elect roles:
//   higher UID => master, lower UID => slave.

typedef enum {
    TRX_ROLE_MASTER = 0,
    TRX_ROLE_SLAVE,
} TRX_ROLE_ENUM;

static TRX_ROLE_ENUM trx_role = TRX_ROLE_MASTER;

// Pairing (random bind phrase exchange)
#define TRX_PAIRING_PHRASE        "pair.0"
#define TRX_PAIRING_TIMEOUT_MS    180000U
#define TRX_PAIRING_MAGIC         0x52494150UL  // 'PAIR'

typedef enum {
    TRX_PAIRING_REQ = 1,
    TRX_PAIRING_ACK = 2,
} TRX_PAIRING_TYPE_ENUM;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t type;
    uint8_t token;
    char phrase[6];
    uint16_t crc;
} tTrxPairingMsg;

static volatile uint16_t irq_status;
#ifdef USE_SX2
static volatile uint16_t irq2_status;
#endif

static bool link_connected;
static uint16_t link_tmo_cnt;

static uint8_t tx_seq_no;
static tReceiveArq recv_arq;

static bool pairing_mode;
static bool pairing_restart;
static uint32_t pairing_deadline_ms;
static bool pairing_phrase_valid;
static char pairing_phrase[6 + 1];
static uint8_t pairing_token;

#ifdef USE_COM
static tComPort config_port;
static tTxCli cli;
tTxInfo info;
tTasks tasks;
#endif

//------------------------------------------------------------------------------
// Radio IRQ handlers
//------------------------------------------------------------------------------

IRQHANDLER(
void SX_DIO_EXTI_IRQHandler(void)
{
    sx_dio_exti_isr_clearflag();
    irq_status |= sx.GetAndClearIrqStatus(SX_IRQ_ALL);
})

#ifdef USE_SX2
IRQHANDLER(
void SX2_DIO_EXTI_IRQHandler(void)
{
    sx2_dio_exti_isr_clearflag();
    irq2_status |= sx2.GetAndClearIrqStatus(SX2_IRQ_ALL);
})
#endif


//------------------------------------------------------------------------------
// Utilities
//------------------------------------------------------------------------------

static uint16_t crc16_ccitt(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

static uint32_t uid_to_u32(void)
{
    uint8_t uid[STM32_MCU_UID_LEN];
    mcu_uid(uid);

    // Fold 96-bit UID into 32 bits, stable across boots.
    uint32_t a = ((uint32_t)uid[0] << 24) | ((uint32_t)uid[1] << 16) | ((uint32_t)uid[2] << 8) | uid[3];
    uint32_t b = ((uint32_t)uid[4] << 24) | ((uint32_t)uid[5] << 16) | ((uint32_t)uid[6] << 8) | uid[7];
    uint32_t c = ((uint32_t)uid[8] << 24) | ((uint32_t)uid[9] << 16) | ((uint32_t)uid[10] << 8) | uid[11];
    return a ^ b ^ c;
}

static void generate_pairing_phrase(char out[6 + 1])
{
    uint32_t seed = uid_to_u32() ^ millis32();
    for (uint8_t i = 0; i < 6; i++) {
        seed = seed * 1664525U + 1013904223U;
        out[i] = bindphrase_chars[seed % BINDPHRASE_CHARS_LEN];
    }
    out[6] = '\0';
}

static void apply_pairing_phrase_config(void)
{
    char phrase[6 + 1] = TRX_PAIRING_PHRASE;
    strcpy(Setup.Common[Config.ConfigId].BindPhrase, phrase);
    setup_sanitize_config(Config.ConfigId);
    setup_configure_config(Config.ConfigId);
}

static bool parse_pairing_msg(const uint8_t* payload, uint8_t len, tTrxPairingMsg* out)
{
    if (len < sizeof(tTrxPairingMsg)) return false;
    memcpy(out, payload, sizeof(tTrxPairingMsg));
    if (out->magic != TRX_PAIRING_MAGIC) return false;
    if (out->crc != crc16_ccitt((uint8_t*)out, sizeof(tTrxPairingMsg) - 2)) return false;
    return true;
}

static void pairing_commit(const char* phrase)
{
    strncpy(Setup.Common[Config.ConfigId].BindPhrase, phrase, 6);
    Setup.Common[Config.ConfigId].BindPhrase[6] = '\0';
    setup_sanitize_config(Config.ConfigId);
    setup_configure_config(Config.ConfigId);
    setup_store_to_EEPROM();
    pairing_mode = false;
    pairing_phrase_valid = false;
}

static bool wait_for_rx_done(uint16_t timeout_ms)
{
    uint32_t t0 = millis32();
    while ((uint32_t)(millis32() - t0) <= timeout_ms + 1U) {
        uint16_t irqs = irq_status;
        irq_status = 0;

        if (irqs & SX_IRQ_RX_DONE) return true;
        if (irqs & SX_IRQ_TIMEOUT) return false;
    }
    return false;
}

static bool wait_for_tx_done(uint16_t timeout_ms)
{
    uint32_t t0 = millis32();
    while ((uint32_t)(millis32() - t0) <= timeout_ms + 1U) {
        uint16_t irqs = irq_status;
        irq_status = 0;

        if (irqs & SX_IRQ_TX_DONE) return true;
        if (irqs & SX_IRQ_TIMEOUT) return false;
    }
    return false;
}


//------------------------------------------------------------------------------
// Role discovery (pairing)
//------------------------------------------------------------------------------

#define TRX_DISCOVERY_MAGIC  0x4D4C5253UL  // 'MLRS'

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t uid;
    uint16_t crc;
} tTrxDiscoveryFrame;

static void send_discovery(uint32_t uid)
{
    tTrxDiscoveryFrame d;
    d.magic = TRX_DISCOVERY_MAGIC;
    d.uid = uid;
    d.crc = crc16_ccitt((uint8_t*)&d, sizeof(d) - 2);

    irq_status = 0;
    sx.SendFrame((uint8_t*)&d, sizeof(d), SEND_FRAME_TMO_MS);
    (void)wait_for_tx_done(SEND_FRAME_TMO_MS + 2);
}

static bool recv_discovery(uint32_t* peer_uid, uint16_t rx_timeout_ms)
{
    tTrxDiscoveryFrame d;

    irq_status = 0;
    sx.SetToRx(rx_timeout_ms);
    if (!wait_for_rx_done(rx_timeout_ms)) {
        return false;
    }

    sx.ReadFrame((uint8_t*)&d, sizeof(d));

    if (d.magic != TRX_DISCOVERY_MAGIC) return false;
    if (d.crc != crc16_ccitt((uint8_t*)&d, sizeof(d) - 2)) return false;

    *peer_uid = d.uid;
    return true;
}

static TRX_ROLE_ENUM discover_role(void)
{
    const uint32_t my_uid = uid_to_u32();
    const uint32_t t_end = millis32() + 3000U;

    uint32_t next_tx = millis32() + 20U + (my_uid & 0x1FU);

    while ((int32_t)(millis32() - t_end) < 0) {
        uint32_t now = millis32();
        if ((int32_t)(now - next_tx) >= 0) {
            send_discovery(my_uid);
            next_tx = now + 40U + (my_uid & 0x0FU);
        }

        uint32_t peer_uid = 0;
        if (recv_discovery(&peer_uid, 12)) {
            if (peer_uid == my_uid) continue;
            return (my_uid > peer_uid) ? TRX_ROLE_MASTER : TRX_ROLE_SLAVE;
        }
    }

    // If nothing was discovered (single module powered), default to master.
    return TRX_ROLE_MASTER;
}


//------------------------------------------------------------------------------
// Link helpers
//------------------------------------------------------------------------------

bool connected(void)
{
    return link_connected;
}

static void set_connected(void)
{
    link_connected = true;
    link_tmo_cnt = CONNECT_TMO_SYSTICKS;
}

static void set_disconnected(void)
{
    link_connected = false;
    link_tmo_cnt = 0;
    recv_arq.Disconnected();
}

static void prepare_rc_defaults(void)
{
    for (uint8_t i = 0; i < 18; i++) rcData.ch[i] = 1024;
}

static uint8_t read_serial_payload(uint8_t* payload, uint8_t max_len)
{
    uint8_t n = 0;
    for (; n < max_len; n++) {
        if (!serial.available()) break;
        payload[n] = (uint8_t)serial.getc();
    }
    return n;
}

static void process_rx_payload(uint8_t* payload, uint8_t len, uint8_t seq_no)
{
    recv_arq.Received(seq_no);
    if (!recv_arq.AcceptPayload()) return;

    if (len) serial.putbuf(payload, len);
}

static void fill_frame_stats(tFrameStats* s, uint8_t seq_no)
{
    s->seq_no = seq_no;
    s->ack = recv_arq.AckSeqNo();
    s->antenna = ANTENNA_1;
    s->transmit_antenna = ANTENNA_1;
    s->rssi = stats.GetLastRssi();
    s->tx_fhss_index_band = 0;
    s->tx_fhss_index = 0;
    s->LQ_rc = 0;
    s->LQ_serial = 0;
}


//------------------------------------------------------------------------------
// Master / slave transactions
//------------------------------------------------------------------------------

static void pairing_master_step(void)
{
    static uint32_t next_cycle_ms = 0;

    uint32_t now = millis32();
    if ((int32_t)(now - pairing_deadline_ms) > 0) {
        pairing_mode = false;
        pairing_restart = true;
        return;
    }
    if ((int32_t)(now - next_cycle_ms) < 0) return;
    next_cycle_ms = now + Config.frame_rate_ms;

    if (!pairing_phrase_valid) {
        generate_pairing_phrase(pairing_phrase);
        pairing_phrase_valid = true;
    }

    tTrxPairingMsg msg = {};
    msg.magic = TRX_PAIRING_MAGIC;
    msg.type = TRX_PAIRING_REQ;
    msg.token = pairing_token;
    memcpy(msg.phrase, pairing_phrase, 6);
    msg.crc = crc16_ccitt((uint8_t*)&msg, sizeof(msg) - 2);

    tFrameStats fs = {};
    fill_frame_stats(&fs, tx_seq_no++);

    pack_txframe(&txFrame, &fs, &rcData, (uint8_t*)&msg, sizeof(msg));

    irq_status = 0;
    sx.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO_MS);
    if (!wait_for_tx_done(SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        return;
    }

    irq_status = 0;
    sx.SetToRx(SEND_FRAME_TMO_MS);
    if (!wait_for_rx_done(SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        return;
    }

    sx.ReadFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN);
    if (check_rxframe(&rxFrame) != CHECK_OK) {
        recv_arq.FrameMissed();
        return;
    }

    tTrxPairingMsg ack = {};
    if (parse_pairing_msg(rxFrame.payload, rxFrame.status.payload_len, &ack) &&
        ack.type == TRX_PAIRING_ACK &&
        ack.token == pairing_token &&
        memcmp(ack.phrase, pairing_phrase, 6) == 0) {
        pairing_commit(pairing_phrase);
        pairing_restart = true;
    }
}

static void pairing_slave_step(void)
{
    if ((int32_t)(millis32() - pairing_deadline_ms) > 0) {
        pairing_mode = false;
        pairing_restart = true;
        return;
    }

    irq_status = 0;
    sx.SetToRx(25);
    if (!wait_for_rx_done(28)) {
        return;
    }

    sx.ReadFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN);
    if (check_txframe(&txFrame) != CHECK_OK) {
        return;
    }

    tTrxPairingMsg req = {};
    if (!parse_pairing_msg(txFrame.payload, txFrame.status.payload_len, &req)) {
        return;
    }
    if (req.type != TRX_PAIRING_REQ) {
        return;
    }

    tTrxPairingMsg ack = {};
    ack.magic = TRX_PAIRING_MAGIC;
    ack.type = TRX_PAIRING_ACK;
    ack.token = req.token;
    memcpy(ack.phrase, req.phrase, 6);
    ack.crc = crc16_ccitt((uint8_t*)&ack, sizeof(ack) - 2);

    tFrameStats fs = {};
    fill_frame_stats(&fs, tx_seq_no++);
    pack_rxframe(&rxFrame, &fs, (uint8_t*)&ack, sizeof(ack));

    irq_status = 0;
    sx.SendFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO_MS);
    (void)wait_for_tx_done(SEND_FRAME_TMO_MS + 2);

    pairing_commit(req.phrase);
    pairing_restart = true;
}

static void master_step(void)
{
    static uint32_t next_cycle_ms = 0;

    uint32_t now = millis32();
    if ((int32_t)(now - next_cycle_ms) < 0) return;
    next_cycle_ms = now + Config.frame_rate_ms;

    uint8_t payload[FRAME_TX_PAYLOAD_LEN];
    uint8_t payload_len = read_serial_payload(payload, FRAME_TX_PAYLOAD_LEN);

    tFrameStats fs = {};
    fill_frame_stats(&fs, tx_seq_no++);

    pack_txframe(&txFrame, &fs, &rcData, payload, payload_len);

    irq_status = 0;
    sx.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO_MS);
    if (!wait_for_tx_done(SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        return;
    }

    irq_status = 0;
    sx.SetToRx(SEND_FRAME_TMO_MS);
    if (!wait_for_rx_done(SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        if (!connected()) serial.flush();
        return;
    }

    sx.ReadFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN);
    if (check_rxframe(&rxFrame) != CHECK_OK) {
        recv_arq.FrameMissed();
        return;
    }

    sx.GetPacketStatus(&stats.last_rssi1, &stats.last_snr1);
    process_rx_payload(rxFrame.payload, rxFrame.status.payload_len, rxFrame.status.seq_no);
    set_connected();
}

static void slave_step(void)
{
    uint8_t tx_payload[FRAME_TX_PAYLOAD_LEN];
    uint8_t tx_payload_len;

    irq_status = 0;
    sx.SetToRx(25);
    if (!wait_for_rx_done(28)) {
        return;
    }

    sx.ReadFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN);
    if (check_txframe(&txFrame) != CHECK_OK) {
        return;
    }

    sx.GetPacketStatus(&stats.last_rssi1, &stats.last_snr1);

    tx_payload_len = txFrame.status.payload_len;
    for (uint8_t i = 0; i < tx_payload_len; i++) tx_payload[i] = txFrame.payload[i];
    process_rx_payload(tx_payload, tx_payload_len, txFrame.status.seq_no);

    uint8_t rx_payload[FRAME_RX_PAYLOAD_LEN];
    uint8_t rx_payload_len = read_serial_payload(rx_payload, FRAME_RX_PAYLOAD_LEN);

    tFrameStats fs = {};
    fill_frame_stats(&fs, tx_seq_no++);

    pack_rxframe(&rxFrame, &fs, rx_payload, rx_payload_len);

    irq_status = 0;
    sx.SendFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO_MS);
    (void)wait_for_tx_done(SEND_FRAME_TMO_MS + 2);

    set_connected();
}


//------------------------------------------------------------------------------
// Init and main loop
//------------------------------------------------------------------------------

static void init_hw(void)
{
    delay_init();
    systembootloader_init();
    timer_init();

    leds_init();
    button_init();

    serial.InitOnce();
    serial.Init();
#ifdef USE_COM
    config_port.Init();
#endif
    fan.Init();
#ifdef USE_DEBUG
    dbg.Init();
#endif

    sx.Init();
    sx2.Init();

    setup_init();

    if (pairing_mode) {
        apply_pairing_phrase_config();
        pairing_deadline_ms = millis32() + TRX_PAIRING_TIMEOUT_MS;
        pairing_token = (uint8_t)(uid_to_u32() ^ millis32());
        pairing_phrase_valid = false;
        pairing_restart = false;
    } else {
        pairing_phrase_valid = false;
        pairing_restart = false;
    }

    // Unified firmware is dedicated to transparent serial transfer.
    Setup.Rx.SerialLinkMode = SERIAL_LINK_MODE_TRANSPARENT;

    serial.SetBaudRate(Config.SerialBaudrate);
#ifdef USE_COM
    info.Init();
    tasks.Init();
    cli.Init(&config_port, Config.frame_rate_ms);
#endif

    if (!sx.isOk()) { FAILALWAYS(BLINK_RD_GR_OFF, "Sx not ok"); }
    if (!sx2.isOk()) { FAILALWAYS(BLINK_GR_RD_OFF, "Sx2 not ok"); }

    IF_SX(sx.StartUp(&Config.Sx));
    IF_SX2(sx2.StartUp(&Config.Sx2));

    fhss.Init(&Config.Fhss, &Config.Fhss2);
    fhss.Start();

    sx.SetRfFrequency(fhss.GetCurrFreq());
    sx2.SetRfFrequency(fhss.GetCurrFreq2());

    rfpower.Init();
    stats.Init(Config.LQAveragingPeriod, Config.frame_rate_hz, Config.frame_rate_ms);

    prepare_rc_defaults();

    recv_arq.Init();
    tx_seq_no = 0;
    set_disconnected();

    trx_role = discover_role();

    leds.Init();
    if (pairing_mode) {
        leds.SetToBind();
    }
    resetSysTask();
}


void main_loop(void)
{
INITCONTROLLER_ONCE
    stack_check_init();
RESTARTCONTROLLER
    init_hw();
INITCONTROLLER_END

    if (doSysTask()) {
        if (link_tmo_cnt) {
            link_tmo_cnt--;
        } else {
            set_disconnected();
        }

        leds.Tick_ms(connected());
        fan.SetPower(sx.RfPower_dbm());
        fan.Tick_ms();

        // Long-press button to start pairing (random bind phrase exchange).
        static uint32_t pair_press_start_ms = 0;
        if (button_pressed()) {
            if (pair_press_start_ms == 0) pair_press_start_ms = millis32();
            if ((uint32_t)(millis32() - pair_press_start_ms) >= 2000) {
                pairing_mode = true;
                pair_press_start_ms = 0;
                GOTO_RESTARTCONTROLLER;
            }
        } else {
            pair_press_start_ms = 0;
        }
    }

#ifdef USE_COM
    cli.Do();
    switch (tasks.Task()) {
        case TX_TASK_PARAM_STORE:
            setup_store_to_EEPROM();
            break;
        case TX_TASK_PARAM_RELOAD:
            setup_reload();
            GOTO_RESTARTCONTROLLER;
            break;
        case TX_TASK_RX_PARAM_SET:
            GOTO_RESTARTCONTROLLER;
            break;
        case MAIN_TASK_RESTART_CONTROLLER:
            GOTO_RESTARTCONTROLLER;
            break;
        case MAIN_TASK_BIND_START:
            pairing_mode = true;
            GOTO_RESTARTCONTROLLER;
            break;
    }
#endif

    if (pairing_mode) {
        if (trx_role == TRX_ROLE_MASTER) {
            pairing_master_step();
        } else {
            pairing_slave_step();
        }
        if (pairing_restart) {
            pairing_restart = false;
            GOTO_RESTARTCONTROLLER;
        }
        return;
    }

    if (trx_role == TRX_ROLE_MASTER) {
        master_step();
    } else {
        slave_step();
    }
}

#endif // FIRMWARE_MATEK_MR900_30_G431KB
