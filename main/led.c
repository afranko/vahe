/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_hrs_eval_led led.c
 * @{
 * @ingroup ble_sdk_app_hrs_eval
 * @brief LED control for the HRS example application
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "board.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "led.h"
#include "app_util.h"

#define ADVERTISING_LED_PIN_NO               LED_1                                     /**< Is on when device is advertising. */
#define GPIO_OUTPUT_PIN_NUMBER				 LED_1	

static nrf_drv_timer_t timer = NRF_DRV_TIMER_INSTANCE(1);
void timer_dummy_handler(nrf_timer_event_t event_type, void * p_context){}

void led_init(void)
{
   ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&timer, NULL, timer_dummy_handler);
    APP_ERROR_CHECK(err_code);
#ifdef NRF51
    //Workaround for PAN-73.
    *(uint32_t *)0x40008C0C = 1;
#endif
}

void led_start(void)
{
    uint32_t compare_evt_addr;
    uint32_t gpiote_task_addr;
    nrf_ppi_channel_t ppi_channel;
    ret_code_t err_code;
    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    err_code = nrf_drv_gpiote_out_init(GPIO_OUTPUT_PIN_NUMBER, &config);
    APP_ERROR_CHECK(err_code);


    nrf_drv_timer_extended_compare(&timer, (nrf_timer_cc_channel_t)0, 200*1000UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
    APP_ERROR_CHECK(err_code);

    compare_evt_addr = nrf_drv_timer_event_address_get(&timer, NRF_TIMER_EVENT_COMPARE0);
    gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(GPIO_OUTPUT_PIN_NUMBER);

    err_code = nrf_drv_ppi_channel_assign(ppi_channel, compare_evt_addr, gpiote_task_addr);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(ppi_channel);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_task_enable(GPIO_OUTPUT_PIN_NUMBER);
	
    // Enable timer
    nrf_drv_timer_enable(&timer);	
}

void led_stop(void)
{
    // Disable the GPIOTE_CHAN_FOR_LED_TASK. This is because when an task has been configured
    // to operate on a pin, the pin can only be written from GPIOTE module. Attempting to write a
    // pin (using nrf_gpio_pin_clear() below for example) as a normal GPIO pin will have no effect.
    // Enable timer
    nrf_drv_timer_disable(&timer);	
    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
}

/**
 * @}
 */
