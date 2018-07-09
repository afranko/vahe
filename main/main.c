
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
#include "dw_tag.h"
#include "dw_anchor.h"
#include "nrf_drv_uart.h"
#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "mpu.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "commons.h"

/* Includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"


//  debugging with gdb: ./JLinkGDBServer -if SWD -device nRF51822
//  ./arm-none-eabi-gdb --tui /home/strauss/munka/ble-decawave/nRF51_SDK_10.0.0_dc26b5e/examples/ble-decawave-tag/ble_app_deca/dev_deca_bt/s110/armgcc/_build/nrf51822_xxac_s110_res.out
//	(gdb) target remote localhost:2331
//	(gdb) load /home/strauss/munka/ble-decawave/nRF51_SDK_10.0.0_dc26b5e/examples/ble-decawave-tag/ble_app_deca/dev_deca_bt/s110/armgcc/_build/nrf51822_xxac_s110_res.out
//	(gdb) monitor reset/go/halt
//	(gdb) monitor memU8 <memory_address>


const char intArray[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

static uint16_t                          	 	m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_bas_t                              	bas;                                       /**< Structure used to identify the battery service. */
static dm_application_instance_t                m_app_handle;                              /**< Application identifier allocated by device manager */

APP_TIMER_DEF(m_battery_timer_id);
APP_TIMER_DEF(m_measurement_timer_id);
APP_TIMER_DEF(m_beacon_timer_id);
APP_TIMER_DEF(m_mpu_timer_id);
APP_TIMER_DEF(m_ts_timer_id);           //NOTE+TODO

//#define USE_UART

void simple_uart_putstring(const uint8_t *str)
{
#ifdef USE_UART
    nrf_drv_uart_tx(str, strlen((const char*)str));
#endif
}

void simple_uart_puthex(uint8_t hex)
{
#ifdef USE_UART
    nrf_drv_uart_tx((uint8*)&intArray[(hex&0xF0) >> 4], 1);
    nrf_drv_uart_tx((uint8*)&intArray[(hex&0x0F)], 1);
#endif
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
	simple_uart_putstring((const uint8_t *)"Error handler: ");

	switch(error_code)
	{
		case NRF_ERROR_INVALID_ADDR: simple_uart_putstring((const uint8_t *)"Invalid pointer supplied.\n\r"); break;
		case NRF_ERROR_INVALID_PARAM: simple_uart_putstring((const uint8_t *)"Invalid parameter(s) supplied, service handle, Vendor Specific UUIDs, lengths, and permissions need to adhere to the constraints..\n\r"); break;
		case NRF_ERROR_INVALID_STATE: simple_uart_putstring((const uint8_t *)"Invalid state to perform operation, a service context is required..\n\r"); break;
		case NRF_ERROR_FORBIDDEN: simple_uart_putstring((const uint8_t *)"Forbidden value supplied, certain UUIDs are reserved for the stack..\n\r"); break;
		case NRF_ERROR_NO_MEM: simple_uart_putstring((const uint8_t *)"Not enough memory to complete operation..\n\r"); break;
		case NRF_ERROR_DATA_SIZE: simple_uart_putstring((const uint8_t *)"Invalid data size(s) supplied, attribute lengths are restricted by @ref BLE_GATTS_ATTR_LENS_MAX..\n\r"); break;
		case NRF_ERROR_SOC_NVIC_INTERRUPT_NOT_AVAILABLE: simple_uart_putstring((const uint8_t *)"IRQn is not available for the application."); break;
		case NRF_ERROR_SOC_NVIC_INTERRUPT_PRIORITY_NOT_ALLOWED: simple_uart_putstring((const uint8_t *)"The interrupt priority is not available for the application."); break;
		case BLE_ERROR_INVALID_CONN_HANDLE: simple_uart_putstring((const uint8_t *)"Invalid Connection Handle.\n\r"); break;
		case BLE_ERROR_INVALID_ATTR_HANDLE: simple_uart_putstring((const uint8_t *)"Invalid attribute handle(s) supplied. Only attributes added directly by the application are available to notify and indicate.\n\r"); break;
		case BLE_ERROR_GATTS_INVALID_ATTR_TYPE: simple_uart_putstring((const uint8_t *)"Invalid attribute type(s) supplied, only characteristic values may be notified and indicated.\n\r"); break;
		case NRF_ERROR_NOT_FOUND: simple_uart_putstring((const uint8_t *)"Attribute not found.\n\r"); break;
		case NRF_ERROR_BUSY: simple_uart_putstring((const uint8_t *)"Procedure already in progress.\n\r"); break;
		case BLE_ERROR_GATTS_SYS_ATTR_MISSING: simple_uart_putstring((const uint8_t *)"System attributes missing, use @ref sd_ble_gatts_sys_attr_set to set them to a known value.\n\r"); break;
		case BLE_ERROR_NO_TX_BUFFERS: simple_uart_putstring((const uint8_t *)"There are no available buffers to send the data, applies only to notifications.\n\r"); break;
		default: simple_uart_putstring((const uint8_t *)"Unknown error ..\n\r"); break;
	}

	while(1){}
}

static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

static void sleep_mode_enter(void)
{
    uint32_t err_code;
    uint32_t count;

    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);

    if (count != 0)
    {
        return;
    }

    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        break;
    case BLE_ADV_EVT_IDLE:
        sleep_mode_enter();
        break;
    default:
        break;
    }
}

static void timers_init(void)
{
    uint32_t err_code;

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    simple_uart_putstring((const uint8_t *)"Timer Init ok.\n\r");

    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_measurement_timer_id, APP_TIMER_MODE_REPEATED, measurement_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_beacon_timer_id, APP_TIMER_MODE_REPEATED, beacon_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_mpu_timer_id, APP_TIMER_MODE_REPEATED, mpu_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ts_timer_id, APP_TIMER_MODE_REPEATED, ts_timeout_handler);   //NOTE+TODO
    APP_ERROR_CHECK(err_code);

    init_rx_timeout_timer();
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    uint16_t address = getTagAddress();
    uint8_t device_name[5];

    device_name[0] = intArray[(address&0xF000) >> 12];
    device_name[1] = intArray[(address&0x0F00) >> 8];
    device_name[2] = intArray[(address&0xF0) >> 4];
    device_name[3] = intArray[(address&0x0F)];
    device_name[4] = 0;

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)device_name, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


typedef struct
{
    uint8_t type;       // device type, 0x01 => Anchor, 0x02 => Tag
    uint8_t mac[4];     // UWB MAC address (ID)
    uint8_t version;
    uint8_t tx_power;   // The 2's complement of the calibrated Tx Power  (<RSSI @ 1m>)
} manuf_info_t;

static void advertising_init(void)
{
    uint32_t        err_code;
    ble_advdata_t   advdata;
    uint8_t         flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] =
    {
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    uint8_t mac[] = {0x01, 0x02, 0x03, 0x04};
    manuf_info_t manuf_info;
    manuf_info.type = 0x01;
    memcpy(&manuf_info.mac, &mac, sizeof(mac));
    manuf_info.version = 0x01;
    manuf_info.tx_power = 0xC5;

    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = 0x0345;
    manuf_specific_data.data.p_data        = (uint8_t *) &manuf_info;
    manuf_specific_data.data.size          = sizeof(manuf_info_t);

    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags		            = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    advdata.p_manuf_specific_data   = &manuf_specific_data;
    UNUSED_VARIABLE(manuf_specific_data);

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

static bool m_measurement_timer_on = false;
static bool m_mpu_timer_on = false;

static void measurement_update_handler(ble_lbs_t *p_lbs, uint8_t new_state)
{
    uint32_t err_code;

    switch(new_state)
    {
    case 	START_RANGING:
        if(!m_measurement_timer_on)
        {
//			nrf_gpio_pin_set(LED_0);
            err_code = app_timer_start(m_measurement_timer_id, RANGING_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            m_measurement_timer_on = true;
        }
        break;
    case 	START_MPU:
        if(!m_mpu_timer_on)
        {
            mpu_reset();
            err_code = app_timer_start(m_mpu_timer_id, MPU_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            m_mpu_timer_on = true;
        }
        break;
    case	STOP_RANGING:
        if(p_lbs->is_ranging_notification_enabled == false)
        {
//			nrf_gpio_pin_clear(LED_0);
            err_code = app_timer_stop(m_measurement_timer_id);
            APP_ERROR_CHECK(err_code);
            m_measurement_timer_on = false;
        }
        break;
    case	STOP_MPU:
        if(p_lbs->is_mpu_notification_enabled == false)
        {
            err_code = app_timer_stop(m_mpu_timer_id);
            APP_ERROR_CHECK(err_code);
            m_mpu_timer_on = false;
        }
        break;
    case 	ANCHOR_LIST_CHANGED:
        break;
    }
}

static void services_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    ble_lbs_init_t init;

    init.measurement_update_handler = measurement_update_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

    memset(&bas_init, 0, sizeof(bas_init));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
    }
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static uint32_t device_manager_evt_handler(dm_handle_t const* p_handle, dm_event_t const* p_event, ret_code_t event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}

static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t        err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_EVT_TX_COMPLETE:
//		p_ble_evt->evt.common_evt.params.tx_complete.count;
        break;

    case BLE_GAP_EVT_CONNECTED:
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        err_code = app_timer_stop(m_battery_timer_id);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_bas_on_ble_evt(&bas, p_ble_evt);
    ble_lbs_on_ble_evt(&m_lbs, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);

    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

static void ble_stack_init(void)
{
    uint32_t err_code;

    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

#if defined(S110) || defined(S130) || defined(S310)  || defined(S132)
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if defined(S130) || defined(S310) || defined(S132)
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif

    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static void uart_init(void)
{
    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.baudrate = (nrf_uart_baudrate_t)UART_BAUDRATE_BAUDRATE_Baud115200;
    config.hwfc = NRF_UART_HWFC_DISABLED;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    config.parity = NRF_UART_PARITY_EXCLUDED;
    config.pselcts = 0;
    config.pselrts = 0;
    config.pselrxd = SERIAL_RX_PIN;
    config.pseltxd = SERIAL_TX_PIN;

    nrf_drv_uart_init(&config, NULL);
}

static bool dwt_isr_in_progress = false;

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(pin == MPU_IRQ)
    {
        if(m_mpu_timer_on)
        {
            short int_status = 0;

            mpu_get_int_status(&int_status);

            if(int_status & MPU_INT_STATUS_DMP)
            {
                mpu_send_measurements();
            }
        }
    }
    else if(pin == DW1000_IRQ)
    {
        app_timer_cnt_get(&dw_timestamp);

        while(nrf_gpio_pin_read(25) == 1 && !dwt_isr_in_progress)
        {
            dwt_isr_in_progress = true;
//			nrf_gpio_pin_set(LED_1);
            dwt_isr();

            while(deca_twr_poll_msg()){}

//			nrf_gpio_pin_clear(LED_1);
            dwt_isr_in_progress = false;
        }
    }
}

static void gpiote_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    err_code = nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);

    nrf_drv_gpiote_in_config_t in_config2 = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    in_config2.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(MPU_IRQ, &in_config2, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(MPU_IRQ, true);
}


/******************************************************************************************************************************************************/


static void start_tag()
{
    bool erase_bonds = false;

    app_trace_init();
    timers_init();
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();

    uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    twi_init();
    //uint8_t data = 0x40; //MPU-hoz ezt a három sort ki kell kommentezni, meg felső részt vissza, meg az mpu opent
    //i2c_write(0x68, 0x6B, 1, &data);
    //twi_disable();
    mpu_open();

    simple_uart_putstring((const uint8_t *)"Start!\n\r");

    int t = deca_twr_initiator(response_receive_handler, rtls_beacon_receive_handler);
    gpiote_init();

    if(t != 0)
        nrf_gpio_pin_set(LED_1);

    //dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);
    //dwt_entersleep();

    simple_uart_putstring((const uint8_t *)"Reason: ");

    uint32_t reason = 0;
    uint32_t suc = sd_power_reset_reason_get(&reason);

    if(suc != NRF_SUCCESS)
        simple_uart_putstring((const uint8_t *)" failed ");

    simple_uart_puthex((reason>>24)&0xFF);
    simple_uart_puthex((reason>>16)&0xFF);
    simple_uart_puthex((reason>>8)&0xFF);
    simple_uart_puthex((reason)&0xFF);
    simple_uart_putstring((const uint8_t *)"\n\r");

    send_debug_message("Power on Baby!");

    sd_power_reset_reason_clr(0xFFFFFFFF);

    err_code = app_timer_start(m_ts_timer_id, TS_COUNT_INTERVAL, NULL); //NOTE+TODO
    APP_ERROR_CHECK(err_code);

    //NOTE+TODO
    m_lbs.is_ranging_notification_enabled = true;
    if(m_lbs.measurement_update_handler != NULL)
        m_lbs.measurement_update_handler(&m_lbs, START_RANGING);

    m_lbs.is_mpu_notification_enabled = true;
    if(m_lbs.measurement_update_handler != NULL)
        m_lbs.measurement_update_handler(&m_lbs, START_MPU);

    for (;;)
    {
        if(nrf_gpio_pin_read(25) == 1 && !dwt_isr_in_progress)
        {
            dwt_isr_in_progress = true;
//			nrf_gpio_pin_set(LED_1);
            dwt_isr();

            while(deca_twr_poll_msg()){}

//			nrf_gpio_pin_clear(LED_1);
            dwt_isr_in_progress = false;
        }

        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);

        err_code = ble_bas_battery_level_update(&bas, battery_in_percentage);
        if ( (err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != BLE_ERROR_NO_TX_BUFFERS) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) )
            APP_ERROR_HANDLER(err_code);
    }
}

static void start_anchor()
{
    simple_uart_putstring((const uint8_t *)"Start!\n\r");

    timers_init();
    ble_stack_init();

    deca_twr_responder();
    gpiote_init();
    dwt_rxenable(0);

    app_timer_start(m_beacon_timer_id, BEACON_INTERVAL, NULL);

    for (;;)
    {
        if(nrf_gpio_pin_read(25) == 1 && !dwt_isr_in_progress)
        {
            dwt_isr_in_progress = true;
//			nrf_gpio_pin_set(LED_1);
            dwt_isr();

            while(deca_twr_poll_msg()){}

//			nrf_gpio_pin_clear(LED_1);
            dwt_isr_in_progress = false;
        }
    }
}

void app_main()   //NOTE+TODO
{
    nrf_delay_ms(50);

    nrf_gpio_cfg_output(LED_0);
    nrf_gpio_cfg_output(LED_1);

    nrf_gpio_cfg_input(17, NRF_GPIO_PIN_PULLUP);

    nrf_gpio_pin_clear(LED_1);
    nrf_gpio_pin_clear(LED_0);
//#ifdef USE_UART   //TODO
    uart_init();
//#endif

    // battery GND off
    NRF_GPIO->PIN_CNF[BATTERY_GND_PIN] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |  (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

#ifdef	CALIB

    char logstr[128];

    if(nrf_gpio_pin_read(17) == 1)
    {
        for(int i=0;i < HISTOGRAM_SIZE; i++)
        {
            histogram[i].dist = -1;
            histogram[i].cnt = 0;
        }

        simple_uart_putstring((const uint8_t *)"\n\rDevice: DECA_BT TAG\n\r");
        simple_uart_putstring((const uint8_t *)"Calib start!\n\r");

        timers_init();
        ble_stack_init();

        deca_twr_initiator(calib_response_receive_handler, rtls_beacon_receive_handler);
        gpiote_init();

        dist_arrived = false;
        deca_twr_initiator_send_poll(CALIB_ANCHOR_ADDRESS);
        app_timer_start(m_rxtimeout_timer_id, RANGING_RX_TIMEOUT, NULL);

        for (;;)
        {
            if(nrf_gpio_pin_read(25) == 1 && !dwt_isr_in_progress)
            {
                dwt_isr_in_progress = true;
//				nrf_gpio_pin_set(LED_1);
                dwt_isr();

                while(deca_twr_poll_msg()){}

//				nrf_gpio_pin_clear(LED_1);
                dwt_isr_in_progress = false;
            }

            if(calib_cnt > 1000)
                break;

            if(dist_arrived)
            {
                dist_arrived = false;
                deca_twr_initiator_send_poll(CALIB_ANCHOR_ADDRESS);
                app_timer_start(m_rxtimeout_timer_id, RANGING_RX_TIMEOUT, NULL);
            }
        }

        avgDistance = avgDistance / 1000.0;
        uint32 avgCm = (uint32)(avgDistance);

        double variance = m_newS / (calib_cnt - 2);
        uint32 variance32 = (uint32)(variance);

        for(;;)
        {
            sprintf(logstr, "Avg distance: %lu cm\tVariance: %lu\n\ra=[", avgCm, variance32);
            simple_uart_putstring((const uint8_t *)logstr);

            for(int i=0; i < HISTOGRAM_SIZE; i++)
            {
                if(histogram[i].dist != -1)
                {
                    sprintf(logstr, "%d,\t%d;\n\r", histogram[i].dist, histogram[i].cnt);
                    simple_uart_putstring((const uint8_t *)logstr);
                }
            }

            simple_uart_putstring((const uint8_t *)"];\n\r");

            nrf_delay_ms(1000);
        }
    }
    else
    {
        simple_uart_putstring((const uint8_t *)"\n\rDevice: DECA_BT ANCHOR\n\r");

        start_anchor();
    }

#else

    #ifdef	EVBOARD
    simple_uart_putstring((const uint8_t *)"\n\rDevice: PCA10001\n\r");

    start_anchor();

    #else

    if(nrf_gpio_pin_read(17) == 1)
    {
        simple_uart_putstring((const uint8_t *)"\n\rDevice: DECA_BT TAG\n\r");

        start_tag();
    }
    else
    {
        simple_uart_putstring((const uint8_t *)"\n\rDevice: DECA_BT ANCHOR\n\r");

        start_anchor();
    }

    #endif
#endif
}
