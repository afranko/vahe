#ifndef DW_TAG_H_
#define DW_TAG_H_

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_system.h"

extern uint8_t mpu[];
extern int act_range;
extern int act_anchor;
extern int listen_to_beacon;
extern unsigned long dw_timestamp;
extern TimerHandle_t xTimers[5];

void battery_level_meas_timeout_handler(void * p_context);
void mpu_timeout_handler(void * p_context);
void measurement_timeout_handler(void);
void rxtimeout_timeout_handler(void);
void init_rx_timeout_timer();

void response_receive_handler(bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ);
void rtls_beacon_receive_handler(uint16_t src_address, uint16_t hop_address, uint8_t hop_count);
void mpu_send_measurements();

void ts_timeout_handler(); //NOTE+TODO
void send_debug_message(char message[]);

#endif /* DW_TAG_H_ */
