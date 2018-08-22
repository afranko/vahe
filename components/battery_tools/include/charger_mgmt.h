/*
 *
 */
#ifndef __CHARGER_MGMT_H
#define __CHARGER_MGMT_H


#include "driver/gpio.h"

#if (!defined CHR_INPUT) || (!defined CHR_EN1) || (!defined CHR_EN2)
    #error "Charger management pins are not defined!"
#endif

void chg_adc_isr_on_callback(void);

//TODO
inline void chg_adc_isr_off_callback(void) {
    ESP_LOGD(TAG, "Charger-ADC ISR OFF callback is called!");
    ESP_ERROR_CHECK(gpio_isr_handler_remove(CHR_INPUT_SEL));    //TODO ez se biztos, hogy jó így
}

#endif  /* __CHARGER_MGMT_H */
