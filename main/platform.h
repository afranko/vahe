#ifndef __PLATFORM_H_
#define __PLATFORM_H_

//#include <nrf_soc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define POLL_TX_DLY_UUS                 5000ll
#define POLL_RX_TO_RESP_TX_DLY_UUS      5000ll
#define RESP_RX_TO_FINAL_TX_DLY_UUS     5000ll
#define ANT_DLY_CH1                                             16414
#define TX_ANT_DLY                                              ANT_DLY_CH1             //16620 //16436 /* Default antenna delay values for 64 MHz PRF */
#define RX_ANT_DLY                                              ANT_DLY_CH1

#define __DEFINE_IRQ_FLAG	uint8_t is_nested_critical_region = 0

//#define __DISABLE_IRQ()   sd_nvic_critical_region_enter(&is_nested_critical_region)
#define __DISABLE_IRQ()   taskDISABLE_INTERRUPTS() //maybe megoldas
//#define __ENABLE_IRQ()    sd_nvic_critical_region_exit(is_nested_critical_region)
#define __ENABLE_IRQ()   taskENABLE_INTERRUPTS() //maybe megoldas

// This should be aligne at 32 bit
#define COMM_MAC_BUFFER_ELEMENT_SIZE    32
#define COMM_MAC_BUFFER_LENGTH          2
#define COMM_MAC_RX_QUEUE_LENGTH        5
#define COMM_MAC_TX_QUEUE_LENGTH        5

#endif
