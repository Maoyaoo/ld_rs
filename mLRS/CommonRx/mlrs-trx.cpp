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

uint8_t trx_dbg_role = TRX_ROLE_MASTER;
uint8_t trx_dbg_pairing = 0;
uint32_t trx_dbg_tx_done_hits = 0;
uint32_t trx_dbg_rx_done_hits = 0;
uint32_t trx_dbg_tx_timeouts = 0;
uint32_t trx_dbg_rx_timeouts = 0;
uint32_t trx_dbg_role_switches = 0;
uint32_t trx_dbg_ser_in_bytes = 0;
uint32_t trx_dbg_ser_out_bytes = 0;
uint16_t trx_dbg_last_irq = 0;
uint8_t trx_dbg_last_status = 0;
uint16_t trx_dbg_last_deverr = 0;
uint16_t trx_dbg_send_tmo_ms = 0;

// Pairing (random bind phrase exchange)
#define TRX_PAIRING_PHRASE        "ldrs.0"
#define TRX_PAIRING_TIMEOUT_MS    180000U
#define TRX_PAIRING_MAGIC         0x52494150UL  // 'PAIR'
#define TRX_DISCOVERY_MAGIC       0x4D4C5253UL  // 'MLRS'
// Keep TX timeout comfortably above practical on-air duration on SX1262 at 19 Hz.
// This prevents false TX timeout on some modules while preserving deterministic waits.
#define TRX_SEND_FRAME_TMO_MS     ((uint16_t)(SEND_FRAME_TMO_MS + 20U))
// Shorter UI/role timeout for unified transparent link.
#define TRX_CONNECT_TIMEOUT_MS     500U

typedef enum {
    TRX_PAIRING_REQ = 1,
    TRX_PAIRING_ACK = 2,
} TRX_PAIRING_TYPE_ENUM;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t type;
    uint8_t token;
    uint32_t uid;
    char phrase[6];
    uint16_t crc;
} tTrxPairingMsg;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t uid;
    uint16_t crc;
} tTrxDiscoveryFrame;

static volatile uint16_t irq_status;
#ifdef USE_SX2
static volatile uint16_t irq2_status;
#endif

static bool link_connected;
static uint32_t link_tmo_deadline_ms;

static uint8_t tx_seq_no;
static tReceiveArq recv_arq;

static bool pairing_mode;
static bool pairing_restart;
static uint32_t pairing_deadline_ms;
static bool pairing_phrase_valid;
static char pairing_phrase[6 + 1];
static uint8_t pairing_token;
static uint32_t pairing_rand_state;
static uint32_t pairing_next_cycle_ms;
static uint32_t local_uid;
static uint32_t role_retry_deadline_ms;
static uint32_t role_retry_rand_state;
static bool serial_once_initialized;

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

static uint32_t rand32_step(uint32_t* state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

static void schedule_role_retry_deadline(uint16_t base_ms)
{
    uint16_t jitter_ms = (uint16_t)(rand32_step(&role_retry_rand_state) & 0x01FFU); // 0..511
    role_retry_deadline_ms = millis32() + base_ms + jitter_ms;
}

static void generate_pairing_phrase(char out[6 + 1])
{
    uint32_t seed = local_uid ^ millis32();
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
    // Pairing must use deterministic RF settings so two modules with different stored
    // params can still discover each other.
    Setup.Common[Config.ConfigId].FrequencyBand = SETUP_RF_BAND;
    Setup.Common[Config.ConfigId].Mode = MODE_19HZ;
    Setup.Common[Config.ConfigId].Ortho = SETUP_RF_ORTHO;
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
    Setup.Common[Config.ConfigId].FrequencyBand = SETUP_RF_BAND;
    Setup.Common[Config.ConfigId].Mode = MODE_19HZ;
    Setup.Common[Config.ConfigId].Ortho = SETUP_RF_ORTHO;
    setup_sanitize_config(Config.ConfigId);
    setup_configure_config(Config.ConfigId);
    setup_store_to_EEPROM();
    pairing_mode = false;
    trx_dbg_pairing = 0;
    pairing_phrase_valid = false;
}

static bool wait_for_rx_done(uint16_t timeout_ms)
{
    uint32_t t0 = millis32();
    while ((uint32_t)(millis32() - t0) <= timeout_ms + 1U) {
        uint16_t irqs = irq_status;
        irq_status = 0;
        // Fallback to polling radio IRQ status in case DIO1/EXTI path is not firing.
        // This keeps link operation robust on boards where EXTI routing is flaky.
        irqs |= sx.GetAndClearIrqStatus(SX_IRQ_ALL);
        if (irqs) trx_dbg_last_irq = irqs;

        if (irqs & SX_IRQ_RX_DONE) {
            trx_dbg_rx_done_hits++;
            return true;
        }
        if (irqs & SX_IRQ_TIMEOUT) {
            trx_dbg_rx_timeouts++;
            trx_dbg_last_status = sx.GetLastStatus();
            trx_dbg_last_deverr = sx.GetDeviceError();
            return false;
        }
        delay_us(50);
    }
    trx_dbg_rx_timeouts++;
    trx_dbg_last_status = sx.GetLastStatus();
    trx_dbg_last_deverr = sx.GetDeviceError();
    return false;
}

static bool wait_for_tx_done(uint16_t timeout_ms)
{
    uint32_t t0 = millis32();
    while ((uint32_t)(millis32() - t0) <= timeout_ms + 1U) {
        uint16_t irqs = irq_status;
        irq_status = 0;
        // Fallback to polling radio IRQ status in case DIO1/EXTI path is not firing.
        irqs |= sx.GetAndClearIrqStatus(SX_IRQ_ALL);
        if (irqs) trx_dbg_last_irq = irqs;

        if (irqs & SX_IRQ_TX_DONE) {
            trx_dbg_tx_done_hits++;
            return true;
        }
        if (irqs & SX_IRQ_TIMEOUT) {
            trx_dbg_tx_timeouts++;
            trx_dbg_last_status = sx.GetLastStatus();
            trx_dbg_last_deverr = sx.GetDeviceError();
            return false;
        }
        delay_us(50);
    }
    trx_dbg_tx_timeouts++;
    trx_dbg_last_status = sx.GetLastStatus();
    trx_dbg_last_deverr = sx.GetDeviceError();
    return false;
}


//------------------------------------------------------------------------------
// Role discovery (pairing)
//------------------------------------------------------------------------------

static bool parse_discovery_msg(const uint8_t* payload, uint8_t len, uint32_t* peer_uid)
{
    if (len < sizeof(tTrxDiscoveryFrame)) return false;

    tTrxDiscoveryFrame d;
    memcpy(&d, payload, sizeof(d));
    if (d.magic != TRX_DISCOVERY_MAGIC) return false;
    if (d.crc != crc16_ccitt((uint8_t*)&d, sizeof(d) - 2)) return false;

    *peer_uid = d.uid;
    return true;
}

static void send_discovery(uint32_t uid)
{
    tTrxDiscoveryFrame d = {};
    d.magic = TRX_DISCOVERY_MAGIC;
    d.uid = uid;
    d.crc = crc16_ccitt((uint8_t*)&d, sizeof(d) - 2);

    tFrameStats fs = {};
    fs.seq_no = tx_seq_no++;
    fs.ack = recv_arq.AckSeqNo();
    fs.antenna = ANTENNA_1;
    fs.transmit_antenna = ANTENNA_1;
    fs.rssi = stats.GetLastRssi();
    fs.tx_fhss_index_band = 0;
    fs.tx_fhss_index = 0;
    fs.LQ_rc = 0;
    fs.LQ_serial = 0;
    pack_txframe(&txFrame, &fs, &rcData, (uint8_t*)&d, sizeof(d));

    irq_status = 0;
    sx.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, TRX_SEND_FRAME_TMO_MS);
    (void)wait_for_tx_done(TRX_SEND_FRAME_TMO_MS + 2);
}

static bool recv_discovery(uint32_t* peer_uid, uint16_t rx_timeout_ms)
{
    irq_status = 0;
    sx.SetToRx(rx_timeout_ms);
    if (!wait_for_rx_done(rx_timeout_ms)) {
        return false;
    }

    sx.ReadFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN);
    if (check_txframe(&txFrame) != CHECK_OK) return false;
    return parse_discovery_msg(txFrame.payload, txFrame.status.payload_len, peer_uid);
}

static TRX_ROLE_ENUM discover_role(uint16_t discover_ms)
{
    const uint32_t my_uid = local_uid;
    uint32_t prng = my_uid ^ (millis32() * 2654435761UL) ^ 0x9E3779B9UL;
    const uint32_t t_end = millis32() + discover_ms;

    while ((int32_t)(millis32() - t_end) < 0) {
        uint32_t peer_uid = 0;

        if (recv_discovery(&peer_uid, 8)) {
            if (peer_uid == my_uid) continue;
            return (my_uid > peer_uid) ? TRX_ROLE_MASTER : TRX_ROLE_SLAVE;
        }

        send_discovery(my_uid);

        uint16_t backoff_ms = 2U + (rand32_step(&prng) & 0x0FU);
        uint32_t t_backoff_end = millis32() + backoff_ms;
        while ((int32_t)(millis32() - t_backoff_end) < 0 &&
               (int32_t)(millis32() - t_end) < 0) {
            if (!recv_discovery(&peer_uid, 4)) continue;
            if (peer_uid == my_uid) continue;
            return (my_uid > peer_uid) ? TRX_ROLE_MASTER : TRX_ROLE_SLAVE;
        }
    }

    // If nothing was discovered (single module powered), default to master.
    return TRX_ROLE_MASTER;
}

static void set_trx_role(TRX_ROLE_ENUM role)
{
    if (trx_role != role) trx_dbg_role_switches++;
    trx_role = role;
    trx_dbg_role = (uint8_t)role;
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
    link_tmo_deadline_ms = millis32() + TRX_CONNECT_TIMEOUT_MS;
    schedule_role_retry_deadline(800U);
}

static void set_disconnected(void)
{
    link_connected = false;
    link_tmo_deadline_ms = 0;
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
    trx_dbg_ser_in_bytes += n;
    return n;
}

static void process_rx_payload(uint8_t* payload, uint8_t len, uint8_t seq_no)
{
    recv_arq.Received(seq_no);
    if (!recv_arq.AcceptPayload()) return;

    // Never leak internal control frames (role discovery / pairing) into serial passthrough.
    if (len == sizeof(tTrxDiscoveryFrame)) {
        tTrxDiscoveryFrame d = {};
        memcpy(&d, payload, sizeof(d));
        if ((d.magic == TRX_DISCOVERY_MAGIC) &&
            (d.crc == crc16_ccitt((uint8_t*)&d, sizeof(d) - 2))) {
            return;
        }
    }
    if (len == sizeof(tTrxPairingMsg)) {
        tTrxPairingMsg p = {};
        memcpy(&p, payload, sizeof(p));
        if ((p.magic == TRX_PAIRING_MAGIC) &&
            (p.crc == crc16_ccitt((uint8_t*)&p, sizeof(p) - 2))) {
            return;
        }
    }

    if (len) {
        serial.putbuf(payload, len);
        trx_dbg_ser_out_bytes += len;
    }
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

static bool pairing_handle_req_from_txframe(tTxFrame* const frame)
{
    if (check_txframe(frame) != CHECK_OK) return false;

    tTrxPairingMsg req = {};
    if (!parse_pairing_msg(frame->payload, frame->status.payload_len, &req)) return false;
    if (req.type != TRX_PAIRING_REQ) return false;
    if (req.uid == local_uid) return false;
    if (req.uid < local_uid) return false; // deterministic tie-break: higher UID request wins.

    tTrxPairingMsg ack = {};
    ack.magic = TRX_PAIRING_MAGIC;
    ack.type = TRX_PAIRING_ACK;
    ack.token = req.token;
    ack.uid = req.uid; // echo requester UID to disambiguate concurrent handshakes.
    memcpy(ack.phrase, req.phrase, 6);
    ack.crc = crc16_ccitt((uint8_t*)&ack, sizeof(ack) - 2);

    tFrameStats fs = {};
    fill_frame_stats(&fs, tx_seq_no++);
    pack_rxframe(&rxFrame, &fs, (uint8_t*)&ack, sizeof(ack));

    irq_status = 0;
    sx.SendFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN, TRX_SEND_FRAME_TMO_MS);
    (void)wait_for_tx_done(TRX_SEND_FRAME_TMO_MS + 2);

    pairing_commit(req.phrase);
    pairing_restart = true;
    return true;
}

static bool pairing_handle_req_frame(void)
{
    sx.ReadFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN);
    return pairing_handle_req_from_txframe(&txFrame);
}

static void pairing_step(void)
{
    uint32_t now = millis32();
    if ((int32_t)(now - pairing_deadline_ms) > 0) {
        pairing_mode = false;
        trx_dbg_pairing = 0;
        pairing_restart = true;
        return;
    }

    if ((int32_t)(now - pairing_next_cycle_ms) < 0) return;
    pairing_next_cycle_ms = now + Config.frame_rate_ms + (rand32_step(&pairing_rand_state) & 0x07U);

    uint16_t rx_listen_ms = 8U + (rand32_step(&pairing_rand_state) & 0x07U);
    irq_status = 0;
    sx.SetToRx(rx_listen_ms);
    if (wait_for_rx_done((uint16_t)(rx_listen_ms + 3U))) {
        if (pairing_handle_req_frame()) return;
    }

    if (!pairing_phrase_valid) {
        generate_pairing_phrase(pairing_phrase);
        pairing_phrase_valid = true;
    }

    tTrxPairingMsg msg = {};
    msg.magic = TRX_PAIRING_MAGIC;
    msg.type = TRX_PAIRING_REQ;
    msg.token = pairing_token;
    msg.uid = local_uid;
    memcpy(msg.phrase, pairing_phrase, 6);
    msg.crc = crc16_ccitt((uint8_t*)&msg, sizeof(msg) - 2);

    tFrameStats fs = {};
    fill_frame_stats(&fs, tx_seq_no++);
    pack_txframe(&txFrame, &fs, &rcData, (uint8_t*)&msg, sizeof(msg));

    irq_status = 0;
    sx.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, TRX_SEND_FRAME_TMO_MS);
    if (!wait_for_tx_done(TRX_SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        return;
    }

    irq_status = 0;
    sx.SetToRx(TRX_SEND_FRAME_TMO_MS);
    if (!wait_for_rx_done(TRX_SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        return;
    }

    sx.ReadFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN);
    if (check_rxframe(&rxFrame) != CHECK_OK) {
        // If we got a TX frame while waiting for ACK, peer likely sent its REQ
        // in our ACK window. Handle it so pairing can still converge.
        if (pairing_handle_req_from_txframe((tTxFrame*)&rxFrame)) return;
        recv_arq.FrameMissed();
        return;
    }

    tTrxPairingMsg ack = {};
    if (!parse_pairing_msg(rxFrame.payload, rxFrame.status.payload_len, &ack)) return;
    if (ack.type != TRX_PAIRING_ACK) return;
    if (ack.uid != local_uid) return;
    if (ack.token != pairing_token) return;
    if (memcmp(ack.phrase, pairing_phrase, 6) != 0) return;

    pairing_commit(pairing_phrase);
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
    sx.SendFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN, TRX_SEND_FRAME_TMO_MS);
    if (!wait_for_tx_done(TRX_SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        return;
    }

    irq_status = 0;
    sx.SetToRx(TRX_SEND_FRAME_TMO_MS);
    if (!wait_for_rx_done(TRX_SEND_FRAME_TMO_MS + 2)) {
        recv_arq.FrameMissed();
        if (!connected()) {
            serial.flush();
            next_cycle_ms += 2U + (rand32_step(&role_retry_rand_state) & 0x0FU);
        }
        return;
    }

    sx.ReadFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN);
    if (check_rxframe(&rxFrame) != CHECK_OK) {
        // Self-heal: if we received a TX frame here, the peer is also in master role.
        if (check_txframe((tTxFrame*)&rxFrame) == CHECK_OK) {
            tTxFrame* peer_tx = (tTxFrame*)&rxFrame;
            uint8_t tx_payload[FRAME_TX_PAYLOAD_LEN];
            uint8_t tx_payload_len = peer_tx->status.payload_len;
            for (uint8_t i = 0; i < tx_payload_len; i++) tx_payload[i] = peer_tx->payload[i];
            process_rx_payload(tx_payload, tx_payload_len, peer_tx->status.seq_no);

            uint8_t rx_payload[FRAME_RX_PAYLOAD_LEN];
            uint8_t rx_payload_len = read_serial_payload(rx_payload, FRAME_RX_PAYLOAD_LEN);
            tFrameStats fs2 = {};
            fill_frame_stats(&fs2, tx_seq_no++);
            pack_rxframe(&rxFrame, &fs2, rx_payload, rx_payload_len);

            irq_status = 0;
            sx.SendFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN, TRX_SEND_FRAME_TMO_MS);
            (void)wait_for_tx_done(TRX_SEND_FRAME_TMO_MS + 2);

            set_trx_role(TRX_ROLE_SLAVE);
            set_connected();
            return;
        }
        recv_arq.FrameMissed();
        next_cycle_ms += 2U + (rand32_step(&role_retry_rand_state) & 0x0FU);
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
    const uint16_t slave_listen_tmo_ms = (uint16_t)(TRX_SEND_FRAME_TMO_MS + 3U);

    irq_status = 0;
    sx.SetToRx(slave_listen_tmo_ms);
    if (!wait_for_rx_done((uint16_t)(slave_listen_tmo_ms + 3U))) {
        return;
    }

    sx.ReadFrame((uint8_t*)&txFrame, FRAME_TX_RX_LEN);
    if (check_txframe(&txFrame) != CHECK_OK) {
        // Self-heal: if we received an RX frame while listening, peer is likely also in slave role.
        if (check_rxframe((tRxFrame*)&txFrame) == CHECK_OK) {
            set_trx_role(TRX_ROLE_MASTER);
            tRxFrame* peer_rx = (tRxFrame*)&txFrame;
            process_rx_payload(peer_rx->payload, peer_rx->status.payload_len, peer_rx->status.seq_no);
            set_connected();
        }
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
    sx.SendFrame((uint8_t*)&rxFrame, FRAME_TX_RX_LEN, TRX_SEND_FRAME_TMO_MS);
    (void)wait_for_tx_done(TRX_SEND_FRAME_TMO_MS + 2);

    set_connected();
}

static void role_resync_step(void)
{
    if (pairing_mode) return;
    if (connected()) {
        schedule_role_retry_deadline(800U);
        return;
    }

    if ((int32_t)(millis32() - role_retry_deadline_ms) < 0) return;

    // Keep this slice short; long blocking discovery causes visible LED stutter.
    set_trx_role(discover_role(40U));
    role_retry_deadline_ms = millis32() + 120U;
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

    if (!serial_once_initialized) {
        serial.InitOnce();
        serial_once_initialized = true;
    }
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
    local_uid = uid_to_u32();

    // Soft restart (GOTO_RESTARTCONTROLLER) does not zero BSS; reset debug counters explicitly.
    trx_dbg_tx_done_hits = 0;
    trx_dbg_rx_done_hits = 0;
    trx_dbg_tx_timeouts = 0;
    trx_dbg_rx_timeouts = 0;
    trx_dbg_role_switches = 0;
    trx_dbg_ser_in_bytes = 0;
    trx_dbg_ser_out_bytes = 0;
    trx_dbg_last_irq = 0;
    trx_dbg_last_status = 0;
    trx_dbg_last_deverr = 0;
    irq_status = 0;

    if (pairing_mode) {
        apply_pairing_phrase_config();
        pairing_deadline_ms = millis32() + TRX_PAIRING_TIMEOUT_MS;
        pairing_token = (uint8_t)(local_uid ^ millis32());
        pairing_rand_state = local_uid ^ millis32() ^ 0xA5A55A5AU;
        pairing_next_cycle_ms = 0;
        pairing_phrase_valid = false;
        pairing_restart = false;
    } else {
        pairing_rand_state = 0;
        pairing_next_cycle_ms = 0;
        pairing_phrase_valid = false;
        pairing_restart = false;
    }
    trx_dbg_pairing = pairing_mode ? 1 : 0;

    // Unified firmware is dedicated to transparent serial transfer.
    Setup.Rx.SerialLinkMode = SERIAL_LINK_MODE_TRANSPARENT;

    serial.SetBaudRate(Config.SerialBaudrate);
    serial.flush();
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
    trx_dbg_send_tmo_ms = TRX_SEND_FRAME_TMO_MS;
    role_retry_rand_state = local_uid ^ millis32() ^ 0xC3C33C3CU;

    if (pairing_mode) {
        set_trx_role(TRX_ROLE_MASTER);
    } else {
        set_trx_role(discover_role(3000U));
    }
    schedule_role_retry_deadline(700U);

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

    // Keep timeout tracking independent of SysTick task cadence in this blocking loop design.
    if (link_connected && ((int32_t)(millis32() - link_tmo_deadline_ms) > 0)) {
        set_disconnected();
    }

    // Drive LED state every loop to avoid role-dependent blink cadence differences.
    leds.Tick_ms(connected());

    if (doSysTask()) {
        fan.SetPower(sx.RfPower_dbm());
        fan.Tick_ms();
        role_resync_step();

        // Long-press button to start pairing (random bind phrase exchange).
        static uint32_t pair_press_start_ms = 0;
        if (button_pressed()) {
            if (pair_press_start_ms == 0) pair_press_start_ms = millis32();
            if ((uint32_t)(millis32() - pair_press_start_ms) >= 2000) {
                pairing_mode = true;
                trx_dbg_pairing = 1;
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
            trx_dbg_pairing = 1;
            GOTO_RESTARTCONTROLLER;
            break;
    }
#endif

    if (pairing_mode) {
        pairing_step();
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
