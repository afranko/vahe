#include "dw_tag.h"

#ifdef TAG_MODE

#include <string.h>
#include <stdio.h>

#include "ble_lbs.h"

#include "nrf_deca.h"

#include "decadriver/deca_device_api.h"
#include "send_tag_uwb.h"


extern SemaphoreHandle_t xSemaphore;



uint8_t err_cnter = 0;	//TODO + NOTE
//extern uint8_t last_subtype;

static int listen_to_beacon_cnt = BEACON_LISTEN_CNT-1;

//static void try_failed_ranging_again(void);

uint8_t mpu[]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int act_range = 0;
int act_anchor = 0;
int listen_to_beacon = 0;
unsigned long dw_timestamp = 0;

bool isMacSendEnabled   = true;   // NOTE: for mac_send
bool isMsgRangingInit   = false;  // NOTE: for mac_send
bool isMsgMpuInit   = false;  // NOTE: for mac_send

rangingMessage  rMessage;
mpuMessage      mMessage;
uint64_t        macTimeStamp    =   0;

static bool     rangTX          =   false;

uint8_t rSerializedMessage[200];

/*void try_failed_ranging_again(void)
{
	if(last_subtype != MAC_FRAME_ST_POLL) {
		tfra_func(m_lbs.anchorlist[act_anchor]);
	}
	else {
		deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
	}
	xTimerStart(xTimers[3], 0);
}*/

void rTimerCallback(TimerHandle_t pxTimer)
{
	//ets_printf("TIMER ");
	//ets_printf("%d", (uint32_t) pvTimerGetTimerID(pxTimer));
	//ets_printf(" has stopped!\n");
	rxtimeout_timeout_handler();
}


void mpu_timeout_handler(void * p_context) {}

void ts_timeout_handler()
{
    macTimeStamp++;
}

void measurement_timeout_handler()
{
	//if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
		if(m_lbs.is_ranging_notification_enabled)
		{
			//uint8_t tmp[600];				//TODO+NOTE
			//dwt_spicswakeup(tmp, 600);	//TODO+NOTE

			act_anchor = 0;
			act_range = 0;

			while(act_anchor < MAX_ANCHORS && m_lbs.anchorlist[act_anchor] == 0) { act_anchor++; }

			if(act_anchor == MAX_ANCHORS)
			{
	//			simple_uart_putstring((const uint8_t *)"A");
				deca_twr_initiator_listen_to_beacon();
				listen_to_beacon = 1;
				xTimerStart(xTimers[4], 0);	//NOTE+TODO TICKS!
			}
			else
			{
	//			simple_uart_putstring((const uint8_t *)"B");
				deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
				xTimerStart(xTimers[3], 0);	//NOTE+TODO TICKS!
			}
		}
		//xSemaphoreGive(xSemaphore);
	//}
}

void rxtimeout_timeout_handler(void)
{
	//if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
		deca_twr_rxtimeout();

		if(listen_to_beacon == 0)
		{
			if(err_cnter < 4) {
				++err_cnter;
				//try_failed_ranging_again();
				deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
				ets_printf("MALACOK AZ URBEN! ANCHOR: %x\n", m_lbs.anchorlist[act_anchor]);	//Pigs in the air!
				xTimerStart(xTimers[3], 0);
			}
			else {
				err_cnter = 0;
				ets_printf("RANGING FAILED! ANCHOR: %x\n", m_lbs.anchorlist[act_anchor]);	//Dirty words if it failed...
				response_receive_handler(false, m_lbs.anchorlist[act_anchor], 0, 0);
			}
		}
		else
		{
			listen_to_beacon = 0;

			//dwt_entersleep();
		}
		//xSemaphoreGive(xSemaphore);
	//}
}

void init_rx_timeout_timer()
{
	xTimers[3] = xTimerCreate("ranging_timer", RANGING_RX_TICKNUM/portTICK_PERIOD_MS, pdFALSE , (void*)3, rTimerCallback);
	xTimers[4] = xTimerCreate("beacon_timer", BEACON_RX_TICKNUM/portTICK_PERIOD_MS, pdFALSE , (void*)4, rTimerCallback);
}

void response_receive_handler(bool success, uint16_t src_address, uint16_t distance, uint8_t RxQ)
{
	xTimerStop(xTimers[3], 0);
	xTimerStop(xTimers[4], 0);

    if(!isMsgRangingInit)
    {
        init_ranging_msg(&rMessage, getTagAddress());
		init_http_send();
        isMsgRangingInit = true;
    }

    ets_printf("%X", (src_address>>8)&0xFF);
    ets_printf("%X", (src_address)&0xFF);
    ets_printf(" initiator_response_receive_handler called\n\r");

    if(!success)
    {
        m_lbs.anchor_notpresence[act_anchor]++;

        if(m_lbs.anchor_notpresence[act_anchor] > 3)
        {
            m_lbs.anchorlist[act_anchor] = 0;
            m_lbs.anchorcount--;

			ble_lbs_anchorlist_update(&m_lbs);
        }
    }
    else
        m_lbs.anchor_notpresence[act_anchor] = 0;

    uint32_t pressure = 0xACDCACDC;
    int32_t temperature = 0;

    ets_printf("A: %x, D: %d \n", src_address, distance);

    /* MESSAGE RACKING */
    if(isMacSendEnabled && success)
        racking_ranging(&rMessage, src_address, distance, RxQ);

    act_range++;

    act_anchor++;
    while(act_anchor < MAX_ANCHORS && m_lbs.anchorlist[act_anchor] == 0) { act_anchor++; }

    /* NOTE: SEND TO ANCHOR */
    if(act_anchor == MAX_ANCHORS) //NOTE+TODO
    {
        temperature = 0xABCDACDC;
        set_ranging_pt(&rMessage, (int32_t)pressure, temperature);

        if(!rangTX)
        {
            rangTX = true;
            pts_ranging_uwb(&rMessage, macTimeStamp, rSerializedMessage);
            rangTX = false;
        }
    }

    if(act_anchor < MAX_ANCHORS)
    {
        deca_twr_initiator_send_poll(m_lbs.anchorlist[act_anchor]);
        xTimerStart(xTimers[3], 0);
    }
    else if(listen_to_beacon_cnt == 0)
    {
        deca_twr_initiator_listen_to_beacon();
        listen_to_beacon = 1;
        listen_to_beacon_cnt = BEACON_LISTEN_CNT-1;
        xTimerStart(xTimers[4], 0);	//TODO+NOTE 0-s timer
    }
    else
    {
        listen_to_beacon_cnt--;
        deca_twr_rxtimeout();
    }
}

void rtls_beacon_receive_handler(uint16_t src_address, uint16_t hop_address, uint8_t hop_count)
{
    int alreadyIn = 0;
    for(int i=0; i < MAX_ANCHORS; i++)
    {
        if(m_lbs.anchorlist[i] == src_address)
        {
            m_lbs.anchor_hop_address[i] = hop_address;
            m_lbs.anchor_hop_count[i] = hop_count;
            m_lbs.anchor_notpresence[i] = 0;
            alreadyIn = 1;
            break;
        }
    }

    if(alreadyIn == 0)
    {
        for(int i=0; i < MAX_ANCHORS; i++)
        {
            if(m_lbs.anchorlist[i] == 0)
            {
                m_lbs.anchorlist[i] = src_address;
                m_lbs.anchor_hop_address[i] = hop_address;
                m_lbs.anchor_hop_count[i] = hop_count;
                m_lbs.anchor_notpresence[i] = 0;
                m_lbs.anchorcount++;
                break;
            }
        }
        ble_lbs_anchorlist_update(&m_lbs);
    }
}

#endif
