#include "dw_anchor.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_hci.h"
#include "ble_conn_params.h"
#include "board.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "led.h"
#include "ble_lbs.h"
#include "battery.h"
#include "device_manager.h"
#include "app_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "app_trace.h"
#include "nrf_deca.h"
#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "mpu.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "commons.h"


void beacon_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    deca_twr_responder_send_rtls_beacon();
}
