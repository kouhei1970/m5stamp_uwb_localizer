/**
 * @file uwb_port_console_guard.c
 * @brief USB ホスト不在時 (給電専用アダプタ・モバイルバッテリ) にコンソール出力を
 *        捨てるための共通処理。詳細は uwb_port.h の宣言コメントを参照。
 *        Drops console output while no USB host is attached (power-only
 *        adapter / power bank). See uwb_port.h for the rationale.
 */
#include "uwb_port.h"

#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"
#include "sdkconfig.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"

bool uwb_port_usb_host_connected(void)
{
    return usb_serial_jtag_is_connected();
}

/** ESP_LOGx が使う vprintf。ホストがいなければ何も書かない。
 *  vprintf used by ESP_LOGx; drops everything while no host is attached. */
static int guarded_vprintf(const char* fmt, va_list ap)
{
    if (!usb_serial_jtag_is_connected()) {
        return 0;
    }
    return vprintf(fmt, ap);
}

void uwb_port_console_guard_init(void)
{
    esp_log_set_vprintf(&guarded_vprintf);
}

#else /* UART コンソール等: 読み手の有無で詰まらないので何もしない。
         UART consoles never block on a missing reader; no-op. */

bool uwb_port_usb_host_connected(void) { return true; }
void uwb_port_console_guard_init(void) {}

#endif
