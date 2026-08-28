/**
 * @file main.cpp
 * @brief Phase 2 Step 1 acceptance test for components/uwb_qm33120
 * (uwb::Qm33120): begin(), deviceId()/chipName()/isConnected(), PHY config
 * application, and sendFrame()/receiveFrame() over-the-air connectivity.
 *
 * Mirrors firmware/probe/main/main.c's structure (board selection via
 * Kconfig choice, boards/stamps3.h / boards/atoms3.h at the repo root for
 * pin numbers), but drives the new uwb::Qm33120 C++ class instead of the raw
 * uwb_port + Qorvo SDK calls that firmware/probe exercises directly.
 *
 * Two-board bring-up: flash one board with UWB_DEVTEST_ROLE_SENDER and a
 * second board with UWB_DEVTEST_ROLE_RECEIVER (see main/Kconfig.projbuild)
 * to verify end-to-end connectivity - the sender's periodic sendFrame() logs
 * TX ok/failed, the receiver's receiveFrame() loop logs each frame it gets.
 */
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uwb_port.h"
#include "uwb_status_led.h"
#include "uwb_qm33120.hpp"

#if CONFIG_UWB_DEVTEST_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3(pinout " BOARD_ATOMS3_PINOUT_NAME ")"
#elif CONFIG_UWB_DEVTEST_BOARD_STAMPFLY
#include "boards/stampfly.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPFLY_UWB_PORT_CONFIG
#define BOARD_NAME            "StampFly"
#define BOARD_STATUS_LED_GPIO BOARD_STAMPFLY_STATUS_LED_GPIO
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "M5StampS3A"
#define BOARD_STATUS_LED_GPIO BOARD_STAMPS3_STATUS_LED_GPIO
#endif

#if CONFIG_UWB_DEVTEST_ROLE_RECEIVER
#define ROLE_NAME "RECEIVER"
#else
#define ROLE_NAME "SENDER"
#endif

static const char* TAG = "uwb_devtest";

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/**
 * @brief boards 以下のヘッダの BOARD_UWB_PORT_CONFIG (uwb_port_config_t 用の
 * 指示付き初期化子リスト) を uwb::Config へコピーする。フィールド名は
 * 1:1 で対応している(components/uwb_qm33120/include/uwb_qm33120_types.hpp
 * の uwb::Config コメント参照)。
 */
static uwb::Config makeConfigFromBoard()
{
    const uwb_port_config_t port = BOARD_UWB_PORT_CONFIG;

    uwb::Config cfg;
    cfg.spi_host    = port.spi_host;
    cfg.pin_sck     = port.pin_sck;
    cfg.pin_mosi    = port.pin_mosi;
    cfg.pin_miso    = port.pin_miso;
    cfg.pin_cs      = port.pin_cs;
    cfg.pin_rst     = port.pin_rst;
    cfg.pin_irq     = port.pin_irq;
    cfg.pin_wakeup  = port.pin_wakeup;
    cfg.pin_gp7     = port.pin_gp7;
    cfg.spi_slow_hz = port.spi_slow_hz;
    cfg.spi_fast_hz = port.spi_fast_hz;
    cfg.init_spi_bus = port.init_spi_bus;
    // probe_retry_count / probe_retry_delay_ms / hard_reset_on_begin /
    // port_already_initialized は uwb::Config の既定値のまま (原本の
    // M5Stamp_UWBConfig 既定値と同じ 5 / 20 / true / false)。
    return cfg;
}

static unsigned pacSizeSymbols(uwb::PacSize pac)
{
    switch (pac) {
        case uwb::PacSize::Pac4:
            return 4;
        case uwb::PacSize::Pac16:
            return 16;
        case uwb::PacSize::Pac32:
            return 32;
        case uwb::PacSize::Pac8:
        default:
            return 8;
    }
}

static void logPhyConfig(const uwb::PhyConfig& phy)
{
    ESP_LOGI(TAG, "PHY config: channel=%u preamble=%u pac=%u dataRate=%s txAntDelay=%u rxAntDelay=%u",
             static_cast<unsigned>(phy.channel), static_cast<unsigned>(phy.preambleLength),
             pacSizeSymbols(phy.pacSize), phy.dataRate == uwb::DataRate::Rate6M8 ? "6.8Mbps" : "850kbps",
             phy.txAntennaDelay, phy.rxAntennaDelay);
}

/* runSender()/runReceiver() は選択された役割(Kconfig)の側だけをビルドする。
 * 両方を常に定義すると、選択されなかった側が -Wunused-function の対象になる。 */
#if CONFIG_UWB_DEVTEST_ROLE_RECEIVER

static void runReceiver(uwb::Qm33120& uwb)
{
    uint8_t rxBuf[128];

    while (1) {
        uwb::RxResult rx = uwb.receiveFrame(rxBuf, sizeof(rxBuf), 1000);
        if (rx.success) {
            ESP_LOGI(TAG, "RX ok: seq=%u src=0x%04X panId=0x%04X len=%u payload=\"%.*s\"", rx.sequence, rx.src,
                     rx.panId, (unsigned)rx.payloadLength, (int)rx.payloadLength, rxBuf);
        } else if (rx.error != uwb::Error::RxTimeout) {
            /* RxTimeout はセンダーが1秒周期のため待ち受け中に普通に起きる。ログを流さない。 */
            ESP_LOGW(TAG, "RX error: %s", uwb.lastErrorName());
        }
    }
}

#else

static void runSender(uwb::Qm33120& uwb)
{
    uwb::FrameConfig frame; // defaults: panId=0xDECA, src=0x0001, dst=0xFFFF (broadcast)
    uint32_t counter = 0;

    while (1) {
        char payload[32];
        const int n = snprintf(payload, sizeof(payload), "uwb-devtest-%lu", (unsigned long)counter++);

        uwb::TxResult tx = uwb.sendFrame(reinterpret_cast<const uint8_t*>(payload), static_cast<size_t>(n), frame, 100);
        if (tx.success) {
            ESP_LOGI(TAG, "TX ok: seq=%u elapsedMs=%lu payload=\"%s\"", tx.sequence, (unsigned long)tx.elapsedMs,
                     payload);
        } else {
            ESP_LOGW(TAG, "TX failed: error=%s", uwb.lastErrorName());
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

extern "C" void app_main(void)
{
#ifdef BOARD_STATUS_LED_GPIO
/* Heartbeat colour tells the role apart once the boards are placed and no
 * serial monitor is attached: TAG = green, ANCHOR = red (components/
 * uwb_status_led). Only the M5StampS3A carries a WS2812 on a known GPIO -
 * the AtomS3 has an LCD instead - so the heartbeat compiles out there.
 * 設置後にシリアルを繋がない状態でも役割が分かるよう、ハートビートの色で
 * タグ(緑)とアンカー(赤)を見分ける。フルカラー LED を持つのは M5StampS3A
 * だけ(AtomS3 は LCD)なので、それ以外ではハートビートごと消える。 */
    (void)uwb_status_led_start_role_heartbeat(BOARD_STATUS_LED_GPIO, UWB_STATUS_LED_ROLE_NONE);
#endif
    ESP_LOGI(TAG, "Phase 2 Step 1 uwb_qm33120 devtest, board=%s role=%s", BOARD_NAME, ROLE_NAME);

    uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    const uwb::PhyConfig phy; // defaults: ch9, preamble128, PAC8, 6.8Mbps (see uwb_qm33120_types.hpp)

    logPhyConfig(phy);

    if (!uwbDevice.begin(cfg, phy)) {
        ESP_LOGE(TAG, "begin() failed: error=%s", uwbDevice.lastErrorName());
        return;
    }

    ESP_LOGI(TAG, "deviceId=0x%08lX (expect 0x%08lX) chipName=%s isConnected=%d isInitialized=%d",
             (unsigned long)uwbDevice.deviceId(), (unsigned long)UWB_DEV_ID_EXPECTED, uwbDevice.chipName(),
             uwbDevice.isConnected(), uwbDevice.isInitialized());

    if (uwbDevice.deviceId() != UWB_DEV_ID_EXPECTED) {
        ESP_LOGE(TAG, "unexpected device id, aborting");
        return;
    }
    if (!uwbDevice.isInitialized()) {
        ESP_LOGE(TAG, "PHY init did not complete (isInitialized()==false), aborting");
        return;
    }
    ESP_LOGI(TAG, "begin() + PHY config OK");

#if CONFIG_UWB_DEVTEST_ROLE_RECEIVER
    runReceiver(uwbDevice);
#else
    runSender(uwbDevice);
#endif
}
