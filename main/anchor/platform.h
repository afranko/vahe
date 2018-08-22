#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_system.h"

#define POLL_TX_DLY_UUS                 5000ll
#define POLL_RX_TO_RESP_TX_DLY_UUS      5000ll
#define RESP_RX_TO_FINAL_TX_DLY_UUS     5000ll
#define ANT_DLY_CH1                                             16414
#define TX_ANT_DLY                                              ANT_DLY_CH1             //16620 //16436 /* Default antenna delay values for 64 MHz PRF */
#define RX_ANT_DLY                                              ANT_DLY_CH1

#define __DEFINE_IRQ_FLAG	portMUX_TYPE myMutexX = portMUX_INITIALIZER_UNLOCKED;

#define __DISABLE_IRQ()   portENTER_CRITICAL(&myMutexX)
#define __ENABLE_IRQ()    portEXIT_CRITICAL(&myMutexX)

// This should be aligne at 32 bit
#define COMM_MAC_BUFFER_ELEMENT_SIZE    32
#define COMM_MAC_BUFFER_LENGTH          2
#define COMM_MAC_RX_QUEUE_LENGTH        5
#define COMM_MAC_TX_QUEUE_LENGTH        5

#endif
