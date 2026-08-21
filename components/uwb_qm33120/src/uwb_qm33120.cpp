/**
 * @file uwb_qm33120.cpp
 * @brief Phase 2 Step 1: device management / PHY configuration / raw frame
 * send-receive for uwb::Qm33120.
 *
 * ESP-IDF port of third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp (M5Stack
 * Technology CO LTD, MIT). Function bodies are translated close to 1:1;
 * each one below carries a comment pointing at the corresponding line range
 * in the original file. See PROGRESS.md Phase 2 Step 1 for the full list of
 * intentional deviations; the main ones are:
 *
 *  - No SPIClass pointer, pinMode()/digitalWrite()/delay()/millis(): all SPI/GPIO
 *    access goes through components/uwb_port (uwb_port_init/_deinit/
 *    _hard_reset/_set_wakeup/_wakeup_device_with_io/_spi()), which was
 *    already implemented and verified in Phase 1. deca_sleep()/deca_usleep()/
 *    decamutexon()/decamutexoff() (the Qorvo SDK's required extern "C" hooks)
 *    are therefore also NOT redefined here - uwb_port.c already provides
 *    them; doing so again would be a duplicate-symbol link error.
 *  - No per-instance dwt_spi_s / SPISettings / setSlowRate()/setFastRate()/
 *    readFromSpi()/writeToSpi()/writeToSpiWithCrc()/wakeupDeviceWithIo()
 *    static trampolines (cpp:46-47, 120-122, 1434-1480, 1527-1548): uwb_port
 *    is a singleton that already implements struct dwt_spi_s and the wakeup
 *    pulse directly (uwb_port_spi(), uwb_port_wakeup_device_with_io()), so
 *    there is no per-instance dispatch left to do.
 *  - millis() -> a small nowMs() wrapper over esp_timer_get_time() (see
 *    uwb_qm33120_internal.hpp); the `(nowMs() - startMs) < timeoutMs`
 *    wraparound-safe idiom is otherwise unchanged.
 *
 * Phase 2 Step 2 addendum: `struct Qm33120::Impl` used to be defined inline
 * right above Qm33120::Qm33120() in this file (mirroring cpp:43-54, which
 * sits directly in M5Stamp_UWB.cpp since that original is a single
 * translation unit). It has been moved, unchanged, to
 * src/uwb_qm33120_impl.hpp so that src/uwb_qm33120_twr.cpp (Step 2's TWR
 * methods, a second translation unit of this same class) can also see the
 * complete type and access _impl->tx_sequence / _impl->tx_antenna_delay /
 * _impl->initialized the same way the original's member functions do. This
 * is the only change Step 2 made to this Step 1 file - see PROGRESS.md
 * Phase 2 Step 2.
 */
#include "uwb_qm33120.hpp"

#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uwb_port.h"

extern "C" {
#include "deca_device_api.h"
#include "deca_interface.h"
/* Matches M5Stamp_UWB.cpp:15 / firmware/probe/main/main.c: USE_DRV_DW3720 is
 * not defined for this build (see components/qm33120w_sdk/CMakeLists.txt),
 * so the extern declaration in deca_device_api.h is compiled out and must be
 * repeated here. */
extern const struct dwt_driver_s dw3720_driver;
}

#include "uwb_qm33120_impl.hpp"
#include "uwb_qm33120_internal.hpp"

static const char* TAG = "Qm33120";

namespace uwb {

/* ------------------------------------------------------------------------
 * PHY設定変換ヘルパ（cpp:58-262 の static 関数群を1:1移植。ロジック変更なし）
 * ------------------------------------------------------------------------ */
namespace {

uint16_t pacSymbols(PacSize pacSize)
{
    switch (pacSize) {
        case PacSize::Pac4:
            return 4;
        case PacSize::Pac16:
            return 16;
        case PacSize::Pac32:
            return 32;
        case PacSize::Pac8:
        default:
            return 8;
    }
}

uint16_t sfdSymbols(SfdType sfdType)
{
    switch (sfdType) {
        case SfdType::DW16:
            return 16;
        case SfdType::IEEE4A:
        case SfdType::IEEE4Z:
        case SfdType::DW8:
        default:
            return 8;
    }
}

uint16_t makeSfdTimeout(const PhyConfig& phy)
{
    // docs/REIMPL_PLAN.md R8: 実際の式・丸めは uwb::detail::sfdTimeoutFromPhy()
    // (uwb_qm33120_units.hpp) に集約した（ESP-IDF非依存にしてホストテスト
    // 可能にするため）。phy.sfdTimeout==0 のときだけ自動計算する点、非0なら
    // そのまま使う点は変更なし。0 をそのまま dwt_config_t::sfdTO に渡すと
    // SDK既定の4161に落ちてしまうため、必ずここで計算してから渡す。
    const uint16_t preambleSymbols = static_cast<uint16_t>(phy.preambleLength);
    const uint16_t sfdLength       = sfdSymbols(phy.sfdType);
    const uint16_t pacLength       = pacSymbols(phy.pacSize);
    return detail::sfdTimeoutFromPhy(phy.sfdTimeout, preambleSymbols, sfdLength, pacLength);
}

dwt_uwb_bit_rate_e toDwtDataRate(DataRate dataRate)
{
    return dataRate == DataRate::Rate850K ? DWT_BR_850K : DWT_BR_6M8;
}

dwt_pac_size_e toDwtPacSize(PacSize pacSize)
{
    switch (pacSize) {
        case PacSize::Pac4:
            return DWT_PAC4;
        case PacSize::Pac16:
            return DWT_PAC16;
        case PacSize::Pac32:
            return DWT_PAC32;
        case PacSize::Pac8:
        default:
            return DWT_PAC8;
    }
}

dwt_sfd_type_e toDwtSfdType(SfdType sfdType)
{
    switch (sfdType) {
        case SfdType::IEEE4A:
            return DWT_SFD_IEEE_4A;
        case SfdType::DW16:
            return DWT_SFD_DW_16;
        case SfdType::IEEE4Z:
            return DWT_SFD_IEEE_4Z;
        case SfdType::DW8:
        default:
            return DWT_SFD_DW_8;
    }
}

dwt_sts_mode_e toDwtStsMode(StsMode stsMode)
{
    switch (stsMode) {
        case StsMode::Mode1:
            return DWT_STS_MODE_1;
        case StsMode::Mode2:
            return DWT_STS_MODE_2;
        case StsMode::Sdc:
            return DWT_STS_MODE_SDC;
        case StsMode::Off:
        default:
            return DWT_STS_MODE_OFF;
    }
}

dwt_sts_lengths_e toDwtStsLength(uint16_t stsLength)
{
    switch (stsLength) {
        case 16:
            return DWT_STS_LEN_16;
        case 32:
            return DWT_STS_LEN_32;
        case 128:
            return DWT_STS_LEN_128;
        case 256:
            return DWT_STS_LEN_256;
        case 512:
            return DWT_STS_LEN_512;
        case 1024:
            return DWT_STS_LEN_1024;
        case 2048:
            return DWT_STS_LEN_2048;
        case 64:
        default:
            return DWT_STS_LEN_64;
    }
}

dwt_pdoa_mode_e toDwtPdoaMode(PdoaMode pdoaMode)
{
    switch (pdoaMode) {
        case PdoaMode::Mode1:
            return DWT_PDOA_M1;
        case PdoaMode::Mode3:
            return DWT_PDOA_M3;
        case PdoaMode::Off:
        default:
            return DWT_PDOA_M0;
    }
}

uint8_t channelNumber(Channel channel)
{
    return static_cast<uint8_t>(channel);
}

bool isValidChannel(Channel channel)
{
    return (channel == Channel::Channel5) || (channel == Channel::Channel9);
}

/**
 * @brief チャネル別の推奨PHYプロファイル（TX スペクトラム設定）を返す。
 *
 * docs/REIMPL_PLAN.md R7: 旧実装は `phy.channel = channel` した後、switch の
 * 両分岐で同じ値を代入し直すだけの恒等関数だった（pgDelay/txPower は
 * 一切触っていなかった）。そのためヘッダの「チャネル別の推奨プロファイルを
 * 適用する」というコメントは嘘で、Channel5 を選んでも
 * PhyConfig のデフォルト値（ch9用の pgDelay=0x34, txPower=0xfefefefe）が
 * そのまま ch5 に適用されてしまっていた。
 *
 * 一次資料 docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/config_options.c:
 *   23-27  `txconfig_options`     = { PGdelay:0x34, power:0xfdfdfdfd, PGcount:0 }  (ch5用)
 *   29-33  `txconfig_options_ch9` = { PGdelay:0x34, power:0xfefefefe, PGcount:0 }  (ch9用)
 * これは Qorvo公式サンプル（例: ex_06a_ss_twr_initiator/ss_twr_initiator_sts.c:
 * 191-198）が `config_options.chan == 5` で切り替えて dwt_configuretxrf() に
 * 渡している実際の値そのもの。ここではその値をチャネルで切り替えて返す。
 *
 * 【M5Stamp UWB Module の実運用】docs/SURVEY_m5stamp_uwb_module.md より本機は ch9 固定
 * 運用（既定値もch9用）。Channel enum は将来の別ボード/実験用に残してあり、
 * Channel5 を明示選択した場合も正しい ch5 用TX設定が適用されるようにする
 * （enum を削って握り潰すより、正直に両対応させる方を選んだ。R7選択肢(b)）。
 */
PhyConfig recommendedPHYProfile(Channel channel)
{
    PhyConfig phy;
    phy.channel = channel;

    switch (channel) {
        case Channel::Channel5:
            phy.pgDelay = 0x34;
            phy.txPower = 0xfdfdfdfd;
            break;
        case Channel::Channel9:
        default:
            phy.pgDelay = 0x34;
            phy.txPower = 0xfefefefe;
            break;
    }

    return phy;
}

bool usesDefaultNonChannelFields(const PhyConfig& phy)
{
    const PhyConfig defaults;
    return (phy.preambleLength == defaults.preambleLength) && (phy.pacSize == defaults.pacSize) &&
           (phy.txPreambleCode == defaults.txPreambleCode) && (phy.rxPreambleCode == defaults.rxPreambleCode) &&
           (phy.sfdType == defaults.sfdType) && (phy.dataRate == defaults.dataRate) &&
           (phy.stsMode == defaults.stsMode) && (phy.stsLength == defaults.stsLength) &&
           (phy.pdoaMode == defaults.pdoaMode) && (phy.sfdTimeout == defaults.sfdTimeout) &&
           (phy.phrMode == defaults.phrMode) && (phy.phrRate == defaults.phrRate) &&
           (phy.pgDelay == defaults.pgDelay) && (phy.txPower == defaults.txPower) &&
           (phy.txAntennaDelay == defaults.txAntennaDelay) && (phy.rxAntennaDelay == defaults.rxAntennaDelay) &&
           (phy.enableLnaPa == defaults.enableLnaPa);
}

PhyConfig resolvePHYConfig(const PhyConfig& phy)
{
    if (usesDefaultNonChannelFields(phy)) {
        return recommendedPHYProfile(phy.channel);
    }
    return phy;
}

dwt_config_t toDwtConfig(const PhyConfig& phy)
{
    dwt_config_t config   = {};
    config.chan           = channelNumber(phy.channel);
    config.txPreambLength = static_cast<uint16_t>(phy.preambleLength);
    config.rxPAC          = toDwtPacSize(phy.pacSize);
    config.txCode         = phy.txPreambleCode;
    config.rxCode         = phy.rxPreambleCode;
    config.sfdType        = toDwtSfdType(phy.sfdType);
    config.dataRate       = toDwtDataRate(phy.dataRate);
    config.phrMode        = static_cast<dwt_phr_mode_e>(phy.phrMode);
    config.phrRate        = static_cast<dwt_phr_rate_e>(phy.phrRate);
    config.sfdTO          = makeSfdTimeout(phy);
    config.stsMode        = toDwtStsMode(phy.stsMode);
    config.stsLength      = toDwtStsLength(phy.stsLength);
    config.pdoaMode       = toDwtPdoaMode(phy.pdoaMode);
    return config;
}

dwt_txconfig_t toDwtTxConfig(const PhyConfig& phy)
{
    dwt_txconfig_t config = {};
    config.PGdly          = phy.pgDelay;
    config.power           = phy.txPower;
    config.PGcount         = 0;
    return config;
}

/* uwb::Config -> uwb_port_config_t 変換。原本には存在しない（原本は
 * SPIClass* をそのまま使うため変換不要だった）。begin() 内でのみ使用。 */
uwb_port_config_t toPortConfig(const Config& config)
{
    uwb_port_config_t port_cfg = {};
    port_cfg.spi_host          = config.spi_host;
    port_cfg.pin_sck           = config.pin_sck;
    port_cfg.pin_mosi          = config.pin_mosi;
    port_cfg.pin_miso          = config.pin_miso;
    port_cfg.pin_cs            = config.pin_cs;
    port_cfg.pin_rst           = config.pin_rst;
    port_cfg.pin_irq           = config.pin_irq;
    port_cfg.pin_wakeup        = config.pin_wakeup;
    port_cfg.pin_gp7           = config.pin_gp7;
    port_cfg.spi_slow_hz       = config.spi_slow_hz;
    port_cfg.spi_fast_hz       = config.spi_fast_hz;
    port_cfg.init_spi_bus      = config.init_spi_bus;
    return port_cfg;
}

} // namespace

/* PImpl の定義（cpp:43-54 相当）は uwb_qm33120_impl.hpp へ移動した
 * （src/uwb_qm33120_twr.cpp と共有するため。理由はファイル先頭コメント参照）。 */

Qm33120* Qm33120::_active = nullptr;

Qm33120::Qm33120() : _impl(new Impl())
{
}

Qm33120::~Qm33120()
{
    end();
    delete _impl;
}

bool Qm33120::begin(const Config& config, const PhyConfig& phy)
{
    // cpp:400-403
    if ((_active != nullptr) && (_active != this)) {
        setError(Error::Busy);
        return false;
    }

    // cpp:405-408: 原本は (config.spi==nullptr || pin_cs==UNUSED) を弾く。
    // ここでは uwb_port_init() に必須の4ピンで代替する
    // (port_already_initialized==true の場合は uwb_port_init() 自体を呼ばないので検証不要)。
    if (!config.port_already_initialized &&
        ((config.pin_cs == UWB_PORT_PIN_UNUSED) || (config.pin_sck == UWB_PORT_PIN_UNUSED) ||
         (config.pin_mosi == UWB_PORT_PIN_UNUSED) || (config.pin_miso == UWB_PORT_PIN_UNUSED))) {
        setError(Error::InvalidConfig);
        return false;
    }

    // cpp:410-414
    _impl->config      = config;
    _impl->connected   = false;
    _impl->initialized = false;
    _impl->device_id   = 0;
    _active             = this;

    // cpp:416-432 (cleanupBeginFailure ラムダ)。pin_cs/pin_wakeup を直接
    // digitalWrite() する代わりに uwb_port_set_wakeup(false) を使う
    // (CSは常にuwb_port側がidle highへ戻す実装になっているため、原本のような
    // 明示的なdigitalWrite(pin_cs, HIGH)は不要 - components/uwb_port/src/uwb_port.c
    // のuwb_spi_xfer()参照)。加えて、自分でuwb_port_init()した場合のみ
    // uwb_port_deinit()で解放する。
    auto cleanupBeginFailure = [this]() {
        const Error error = _impl->last_error;
        if (_active == this) {
            _active = nullptr;
        }
        uwb_port_set_wakeup(false);
        if (_impl->port_owned) {
            uwb_port_deinit();
            _impl->port_owned = false;
        }
        _impl->connected   = false;
        _impl->initialized = false;
        _impl->device_id   = 0;
        setError(error);
        return false;
    };

    // cpp:434-459 (pinMode/digitalWrite/SPI.begin) 相当: uwb_port_init() が
    // GPIO設定・SPIバス初期化・slow/fastデバイス登録・DMAスクラッチバッファ確保を
    // まとめて行う。
    if (!_impl->config.port_already_initialized) {
        uwb_port_config_t port_cfg = toPortConfig(_impl->config);
        const esp_err_t err        = uwb_port_init(&port_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uwb_port_init failed: %s", esp_err_to_name(err));
            // 原本に対応物は無いが、Errorの中で意味が最も近い(原本でも定義
            // されているだけで実際には使われていなかった)SpiNotReadyを使う。
            _impl->last_error = Error::SpiNotReady;
            return cleanupBeginFailure();
        }
        _impl->port_owned = true;
    } else {
        _impl->port_owned = false;
    }

    // cpp:461: setSpiRate(spi_slow_hz) 相当。uwb_port_init() は直後を
    // slowレート(s_spi_active = s_spi_slow)にして返すので、ここでの
    // 明示的な呼び出しは不要（components/uwb_port/src/uwb_port.c 参照）。

    // cpp:463-467: hardReset()を条件付きで実行後、wakeupDeviceWithIoImpl()を
    // 無条件で1回呼ぶ。
    if (_impl->config.hard_reset_on_begin) {
        hardReset();
    }
    uwb_port_wakeup_device_with_io();

    // cpp:469-475: probe()より前段の、生SPIでのDevice ID素読みリトライ。
    for (uint8_t i = 0; i < _impl->config.probe_retry_count; ++i) {
        const uint32_t dev_id = readRawDeviceId();
        if ((dev_id != 0x00000000UL) && (dev_id != 0xFFFFFFFFUL)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(_impl->config.probe_retry_delay_ms));
    }

    // cpp:477-485
    if (!probe()) {
        return cleanupBeginFailure();
    }
    if (!init(phy)) {
        return cleanupBeginFailure();
    }
    return true;
}

void Qm33120::end()
{
    // cpp:488-516
    if (_impl == nullptr) {
        return;
    }

    const bool wasActive = (_active == this);
    const bool hadDriver = wasActive || _impl->connected || _impl->initialized;

    if (wasActive && _impl->initialized) {
        dwt_forcetrxoff();
    }
    if (wasActive) {
        _active = nullptr;
    }
    if (hadDriver) {
        uwb_port_set_wakeup(false);
    }
    // IRQ（起床信号）の無効化は、uwb_portそのものの所有権(port_owned)とは
    // 別に、このクラス(Qm33120)が責任を持つ。init()でuwb_port_irq_enable()
    // を呼ったのはこのクラス自身なので、port_already_initialized==true
    // （port_owned==false、uwb_port自体は他の所有者のもの）の場合でも
    // ここで必ず無効化する。port_owned==trueの場合は後続の
    // uwb_port_deinit()内でも呼ばれるが(uwb_port.c 要件5)、
    // uwb_port_irq_disable()は冪等なので二重呼び出しは無害。
    uwb_port_irq_disable();
    _impl->irq_active = false;
    // 原本に対応物は無い: begin()でuwb_port_init()を自分で呼んでいた場合のみ、
    // ここでuwb_port_deinit()して解放する(port_already_initialized==trueで
    // 始めた場合は他の所有者のリソースなので触らない)。
    if (_impl->port_owned) {
        uwb_port_deinit();
        _impl->port_owned = false;
    }

    _impl->connected   = false;
    _impl->initialized = false;
    _impl->device_id   = 0;
    setError(Error::Ok);
}

bool Qm33120::init(const PhyConfig& phy)
{
    // cpp:518-560. ロジックは変更なし。
    if (!_impl->connected) {
        setError(Error::ProbeFailed);
        return false;
    }
    if (_impl->device_id != kQm33120DeviceId) {
        setError(Error::DeviceIdMismatch);
        return false;
    }
    if (!isValidChannel(phy.channel)) {
        setError(Error::InvalidArgument);
        return false;
    }

    const PhyConfig resolvedPHY = resolvePHYConfig(phy);

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        _impl->initialized = false;
        setError(Error::InitFailed);
        return false;
    }

    dwt_config_t dwtConfig = toDwtConfig(resolvedPHY);
    if (dwt_configure(&dwtConfig) != DWT_SUCCESS) {
        _impl->initialized = false;
        setError(Error::ConfigFailed);
        return false;
    }

    dwt_txconfig_t txConfig = toDwtTxConfig(resolvedPHY);
    dwt_configuretxrf(&txConfig);
    dwt_setrxantennadelay(resolvedPHY.rxAntennaDelay);
    dwt_settxantennadelay(resolvedPHY.txAntennaDelay);
    _impl->tx_antenna_delay = resolvedPHY.txAntennaDelay;
    if (resolvedPHY.enableLnaPa) {
        dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    }

    // --- IRQ（起床信号）の有効化 (docs/IRQ_POLICY.md) ---
    // PHY設定が終わった後に行う。dwt_setcallbacks()/dwt_isr()は意図的に
    // 使わない（復号経路をポーリングと1本に保つため。docs/IRQ_POLICY.md
    // 「ポーリング経路は劣化版ではなく正規の動作モードとして保守する」）。
    // 有効化に失敗してもinit()自体は失敗させず、ポーリングのまま続行する
    // （タスクB要件どおり）。use_irqがfalseならここは何もせず、
    // _impl->irq_activeはfalseのまま = 従来(IRQ導入前)と完全に同じ経路になる。
    _impl->irq_active = false;
    if (_impl->config.use_irq) {
        const esp_err_t irqErr = uwb_port_irq_enable();
        if (irqErr == ESP_OK) {
            // RX完了(RXFCG)・RXタイムアウト・RXエラーで起床する。
            // ビットマスクの出典は各待ちループ(uwb_qm33120_twr.cpp)が
            // dwt_readsysstatuslo()で判定している条件と同一
            // (DWT_INT_RXFCG_BIT_MASK / SYS_STATUS_ALL_RX_TO /
            // SYS_STATUS_ALL_RX_ERR、いずれもlo側32bitに収まる)。
            dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR, 0,
                              DWT_ENABLE_INT);
            _impl->irq_active = true;
        } else {
            ESP_LOGW(TAG, "uwb_port_irq_enable failed (%s); falling back to polling", esp_err_to_name(irqErr));
        }
    }

    _impl->initialized = true;
    setError(Error::Ok);
    return true;
}

bool Qm33120::probe()
{
    // cpp:562-586. dwt_spi_s は uwb_port_spi() が返す単一インスタンスをそのまま使う
    // (原本は _impl->dwt_spi、SPI関数ポインタが _active 経由でこのインスタンスに
    // ディスパッチしていたが、uwb_portは元からグローバル単一インスタンスなので
    // ディスパッチ不要)。wakeup_device_with_io も同様にuwb_portの関数を直接渡す。
    struct dwt_driver_s* drivers[]     = {const_cast<struct dwt_driver_s*>(&dw3720_driver)};
    struct dwt_probe_s probe_interface = {};
    probe_interface.dw                    = nullptr;
    probe_interface.spi                   = uwb_port_spi();
    probe_interface.wakeup_device_with_io = uwb_port_wakeup_device_with_io;
    probe_interface.driver_list           = drivers;
    probe_interface.dw_driver_num         = sizeof(drivers) / sizeof(drivers[0]);

    for (uint8_t i = 0; i < _impl->config.probe_retry_count; ++i) {
        if (dwt_probe(&probe_interface) == DWT_SUCCESS) {
            _impl->connected = true;
            _impl->device_id = dwt_readdevid();
            setError(Error::Ok);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(_impl->config.probe_retry_delay_ms));
    }

    _impl->connected = false;
    _impl->device_id = readRawDeviceId();
    setError(Error::ProbeFailed);
    return false;
}

void Qm33120::hardReset(uint32_t reset_low_ms, uint32_t startup_ms)
{
    // cpp:588-600 相当。RST の駆動シーケンスそのものは uwb_port_hard_reset() に
    // 実装済み (pin_rst==UNUSEDならstartup_msだけ待つ点も含め同一)。
    uwb_port_hard_reset(reset_low_ms, startup_ms);
}

uint32_t Qm33120::deviceId() const
{
    return _impl->device_id;
}

uint32_t Qm33120::readRawDeviceId()
{
    // cpp:607-618. readFromSpiImpl() の代わりに uwb_port_spi()->readfromspi() を
    // 直接呼ぶ（呼び出し規約・ヘッダ/ボディ長の意味は同一）。
    uint8_t header    = 0x00;
    uint8_t buffer[4] = {0};

    struct dwt_spi_s* spi = uwb_port_spi();
    if (spi->readfromspi(sizeof(header), &header, sizeof(buffer), buffer) != DWT_SUCCESS) {
        return 0;
    }

    return (static_cast<uint32_t>(buffer[3]) << 24UL) | (static_cast<uint32_t>(buffer[2]) << 16UL) |
           (static_cast<uint32_t>(buffer[1]) << 8UL) | static_cast<uint32_t>(buffer[0]);
}

const char* Qm33120::chipName() const
{
    // cpp:620-626
    if (_impl->device_id == kQm33120DeviceId) {
        return "QM33120/DW3720";
    }
    return "Unknown";
}

TxResult Qm33120::sendFrame(const char* payload, const FrameConfig& frame, uint32_t timeoutMs)
{
    // cpp:628-637
    if (payload == nullptr) {
        TxResult result;
        result.error = Error::InvalidArgument;
        setError(result.error);
        return result;
    }
    return sendFrame(reinterpret_cast<const uint8_t*>(payload), strlen(payload), frame, timeoutMs);
}

TxResult Qm33120::sendFrame(const uint8_t* payload, size_t length, const FrameConfig& frame, uint32_t timeoutMs)
{
    // cpp:639-702. ロジックは変更なし。millis()->detail::nowMs()、delay(1)->vTaskDelay(pdMS_TO_TICKS(1))。
    TxResult result;

    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }
    if ((payload == nullptr) || (length == 0) || (length > 116)) {
        result.error = Error::InvalidArgument;
        setError(result.error);
        return result;
    }

    uint8_t txFrame[128]   = {0};
    const size_t frameLen  = 9 + length;
    const uint8_t sequence = frame.useSequence ? frame.sequence : ++_impl->tx_sequence;
    txFrame[0]              = 0x41;
    txFrame[1]              = 0x88;
    txFrame[2]              = sequence;
    detail::set16le(&txFrame[3], frame.panId);
    detail::set16le(&txFrame[5], frame.dst);
    detail::set16le(&txFrame[7], frame.src);
    memcpy(&txFrame[9], payload, length);

    detail::stopRadioAndClearIoStatus();

    if (dwt_writetxdata(static_cast<uint16_t>(frameLen), txFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearTxStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(static_cast<uint16_t>(frameLen + FCS_LEN), 0, frame.ranging ? 1 : 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        detail::stopRadioAndClearTxStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }

    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < timeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_TXFRS_BIT_MASK) != 0) {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            result.success   = true;
            result.sequence  = txFrame[2];
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = Error::Ok;
            setError(Error::Ok);
            return result;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    detail::stopRadioAndClearTxStatus();
    result.error = Error::TxTimeout;
    setError(result.error);
    return result;
}

RxResult Qm33120::receiveFrame(uint8_t* payload, size_t payloadSize, uint32_t timeoutMs)
{
    // cpp:704-783. ロジックは変更なし。
    RxResult result;

    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }
    if ((payload == nullptr) || (payloadSize == 0)) {
        result.error = Error::InvalidArgument;
        setError(result.error);
        return result;
    }

    detail::stopRadioAndClearIoStatus();

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        detail::stopRadioAndClearRxStatus();
        result.error = Error::RxStartFailed;
        setError(result.error);
        return result;
    }
    // 受信待ちループへ入る直前に、取りこぼした通知を捨てる
    // (docs/IRQ_POLICY.md、uwb_qm33120_twr.cpp 冒頭コメント参照)。IRQが
    // 無効なら何もしない。
    uwb_port_irq_clear_pending();

    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < timeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rawFrame[128]   = {0};
            uint8_t rangingBit      = 0;
            const uint16_t frameLen = dwt_getframelength(&rangingBit);

            if (frameLen > sizeof(rawFrame)) {
                dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);
                result.error = Error::RxBufferTooSmall;
                setError(result.error);
                return result;
            }

            dwt_readrxdata(rawFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            if (!detail::parseShortAddressFrame(rawFrame, frameLen, result)) {
                result.elapsedMs = detail::nowMs() - startMs;
                result.error     = Error::FrameParseFailed;
                setError(result.error);
                return result;
            }
            result.ranging = (rangingBit != 0);
            if (result.payloadLength > payloadSize) {
                result.elapsedMs = detail::nowMs() - startMs;
                result.error     = Error::RxBufferTooSmall;
                setError(result.error);
                return result;
            }

            memcpy(payload, &rawFrame[9], result.payloadLength);
            result.success   = true;
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = Error::Ok;
            setError(Error::Ok);
            return result;
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        // IRQが来るか1ms経つまで待つ。IRQ無効時はvTaskDelay(pdMS_TO_TICKS(1))
        // と完全に等価（uwb_port_irq_wait()のnote参照。docs/IRQ_POLICY.md）。
        (void)uwb_port_irq_wait(1);
    }

    detail::stopRadioAndClearRxStatus();
    result.elapsedMs = detail::nowMs() - startMs;
    result.error     = Error::RxTimeout;
    setError(result.error);
    return result;
}

bool Qm33120::isConnected() const
{
    return _impl->connected;
}

bool Qm33120::isInitialized() const
{
    return _impl->initialized;
}

bool Qm33120::irqActive() const
{
    return _impl->irq_active;
}

Error Qm33120::lastError() const
{
    return _impl->last_error;
}

const char* Qm33120::lastErrorName() const
{
    // cpp:1383-1427
    switch (_impl->last_error) {
        case Error::Ok:
            return "OK";
        case Error::InvalidConfig:
            return "INVALID_CONFIG";
        case Error::SpiNotReady:
            return "SPI_NOT_READY";
        case Error::ProbeFailed:
            return "PROBE_FAILED";
        case Error::DeviceIdMismatch:
            return "DEVICE_ID_MISMATCH";
        case Error::InitFailed:
            return "INIT_FAILED";
        case Error::ConfigFailed:
            return "CONFIG_FAILED";
        case Error::TxDataFailed:
            return "TX_DATA_FAILED";
        case Error::TxStartFailed:
            return "TX_START_FAILED";
        case Error::TxTimeout:
            return "TX_TIMEOUT";
        case Error::RxStartFailed:
            return "RX_START_FAILED";
        case Error::RxTimeout:
            return "RX_TIMEOUT";
        case Error::RxError:
            return "RX_ERROR";
        case Error::RxBufferTooSmall:
            return "RX_BUFFER_TOO_SMALL";
        case Error::FrameParseFailed:
            return "FRAME_PARSE_FAILED";
        case Error::RangeTimestampInvalid:
            return "RANGE_TIMESTAMP_INVALID";
        case Error::RangeFrameMismatch:
            return "RANGE_FRAME_MISMATCH";
        case Error::InvalidArgument:
            return "INVALID_ARGUMENT";
        case Error::Busy:
            return "BUSY";
        default:
            return "UNKNOWN";
    }
}

const Config& Qm33120::config() const
{
    return _impl->config;
}

void Qm33120::setError(Error error)
{
    _impl->last_error = error;
}

} // namespace uwb
