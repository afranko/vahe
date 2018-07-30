#ifndef DW_TAG_H_
#define DW_TAG_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nrf_deca.h"
#include "commons.h"
#include "send_tag_uwb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"




#define MAX_ANCHORS		10
typedef struct ble_lbs_s
{
	uint16_t                    service_handle;
	uint8_t                     uuid_type;
	uint16_t                    conn_handle;

	uint8_t							anchorcount;
    uint16_t						anchorlist[MAX_ANCHORS];
	uint8_t							anchor_notpresence[MAX_ANCHORS];
    uint16_t						anchor_hop_address[MAX_ANCHORS];
    uint8_t							anchor_hop_count[MAX_ANCHORS];

	bool						is_ranging_notification_enabled;
	bool						is_mpu_notification_enabled;
	uint8_t                     mpu_data[6];


} ble_lbs_t;

ble_lbs_t    m_lbs;

extern uint8_t mpu[];
extern int act_range;
extern int act_anchor;
extern int listen_to_beacon;
extern unsigned long dw_timestamp;


TimerHandle_t* timerhandlers;
//void xTimerStart_measurement(void);
//void xTimerStart_ts_timeout(void);
//void xTimerStart_rxtimeout(void);

//void xTimerStop_measurement(void);
//void xTimerStop_ts_timeout(void);
//void xTimerStop_rxtimeout(void);

void xTimerStart_manual(TimerHandle_t timerhandle);
void xTimerStop_manual(TimerHandle_t timerhandle);

void battery_level_meas_timeout_handler(void * p_context);
void mpu_timeout_handler(void * p_context);
void measurement_timeout_handler(void * p_context);
void rxtimeout_timeout_handler(void * p_context);
void init_rx_timeout_timer();

void response_receive_handler(bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ);
void rtls_beacon_receive_handler(uint16_t src_address, uint16_t hop_address, uint8_t hop_count);
void mpu_send_measurements();

void ts_timeout_handler(void * p_context); //NOTE+TODO
void send_debug_message(char message[]);

#endif /* DW_TAG_H_ */
